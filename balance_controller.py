"""
轮腿平衡控制器 — LQR + VMC + 腿长控制
======================================
K矩阵变腿长LQR → T_wheel, T_Leg
腿长PD → F_Leg
VMC Jacobianᵀ → T_rear, T_front
"""

import mujoco as mj
import numpy as np
import time
import math
import mujoco.viewer
from robot_config import (
    init_model, fk, fk_deriv, ik, vmc_jacobian,
    phi_to_q, q_to_phi, phi_dot,
    get_pitch, get_pitch_dot, get_acc, get_k_mat,
    M, G, R, MW,
)

XML_PATH = "COD-2026RoboMaster-Balance copy.xml"
model, data, act_ids, act_idx, joint_addr, freejoint_adr, wheel_body = init_model(XML_PATH)

# ======================== 输入 ========================
LEG_LENGTH_REF = 0.30

# ======================== 增益 ========================
KP_LEG    = 1200.0; KD_LEG    = 300.0   # 腿长 PD (变腿长, 提高响应)
KP_THETA  = 6.0;    KD_THETA  = 0.2     # 抗劈叉 (leg_fun.m)
TORQUE_LIMIT = 3.14

# ======================== 轮子极性 ========================
WHEEL_POL = {"Right": 1, "Left": -1}

# ======================== 轮子积分 ========================
class LegData:
    def __init__(self):
        self.x  = 0.0
        self.x0 = None
        self.FN = 100.0        # 地面支持力 [N], 初始预设着地
        self.grounded = True
        self.theta = 0.0       # 等效摆角 [rad]
        self.theta_dot = 0.0

leg_R = LegData()
leg_L = LegData()

# ======================== 主循环 ========================
def main():
    phi1_ref, phi4_ref = ik(LEG_LENGTH_REF)

    print("=" * 55)
    print(f"  LQR+VMC  L_ref={LEG_LENGTH_REF:.3f}m  IK→phi1={math.degrees(phi1_ref):.0f}° phi4={math.degrees(phi4_ref):.0f}°")
    print("=" * 55)

    q_init = {}
    q_init["Right","rear"], q_init["Right","front"] = phi_to_q("Right", phi1_ref, phi4_ref)
    q_init["Left", "rear"], q_init["Left", "front"] = phi_to_q("Left",  phi1_ref, phi4_ref)
    for (side,jn), qref in q_init.items():
        data.qpos[joint_addr[side][jn]["qpos"]] = qref
    mj.mj_forward(model, data)

    # 诊断: 初始化后左右腿初始状态
    print("--- 初始化后 (mj_forward) ---")
    for side in ["Right","Left"]:
        qr = data.qpos[joint_addr[side]["rear"]["qpos"]]
        qf = data.qpos[joint_addr[side]["front"]["qpos"]]
        p1, p4 = q_to_phi(side, qr, qf)
        L, theta, phi2, phi3, phi5 = fk(p1, p4)
        print(f"  [{side}] raw rear={math.degrees(qr):+.2f}° front={math.degrees(qf):+.2f}°  "
              f"phi1={math.degrees(p1):.1f}° phi4={math.degrees(p4):.1f}°  "
              f"L={L:.4f}m theta={math.degrees(theta):.1f}°")
    # 诊断: 用相同 phi 计算 VMC (左右应一致, 仅极性不同)
    phi_test = phi1_ref, phi4_ref
    L_t, theta_t, phi2_t, phi3_t, phi5_t = fk(*phi_test)
    J_t = vmc_jacobian(phi1_ref, phi2_t, phi3_t, phi4_ref, phi5_t, L_t)
    K_t = get_k_mat(L_t, grounded=True)
    F_t = 0.5*M*G*math.cos(theta_t) + KP_LEG*(LEG_LENGTH_REF - L_t*math.cos(theta_t))
    print(f"  IK target: phi1={math.degrees(phi1_ref):.0f}° phi4={math.degrees(phi4_ref):.0f}° → L={L_t:.4f}m")
    print(f"  Jᵀ = {J_t.tolist()}")
    print(f"  F_Leg(ideal)={F_t:.1f}N")
    print("------------------------------")

    last_print = time.time()

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            pitch     = get_pitch(data, freejoint_adr)
            pitch_dot = get_pitch_dot(data, freejoint_adr)

            # ==== 第一遍: FK + LQR, 收集状态 ====
            state = {}
            for side in ["Right","Left"]:
                a  = joint_addr[side]
                ld = leg_R if side == "Right" else leg_L
                qr  = data.qpos[a["rear"]["qpos"]];   qdr = data.qvel[a["rear"]["dof"]]
                qf  = data.qpos[a["front"]["qpos"]];  qdf = data.qvel[a["front"]["dof"]]
                qw  = data.qpos[a["wheel"]["qpos"]];  qdw = data.qvel[a["wheel"]["dof"]]
                phi1, phi4 = q_to_phi(side, qr, qf)
                phi1_d, phi4_d = phi_dot(side, qdr, qdf)
                L, theta, theta_d, L_d, phi2, phi3, phi5, phi2_d = \
                    fk_deriv(phi1, phi4, phi1_d, phi4_d, pitch, pitch_dot)
                if ld.x0 is None: ld.x0 = qw
                ld.x  = R * (qw - ld.x0); x_dot = R * qdw
                ld.grounded = ld.FN >= 20.0
                K_mat  = get_k_mat(L, grounded=ld.grounded)
                X = np.array([theta, theta_d, ld.x, x_dot, pitch, pitch_dot])
                u = K_mat @ (-X)
                Height   = L * math.cos(theta)
                Height_d = L_d * math.cos(theta) - L * math.sin(theta) * theta_d
                F_Leg = 0.5*M*G*math.cos(theta) \
                      + KP_LEG*(LEG_LENGTH_REF - Height) + KD_LEG*(0.0 - Height_d)
                state[side] = (phi1,phi4,phi1_d,phi4_d, L,theta,theta_d,L_d,phi2,phi3,phi5,
                               u[0],u[1],F_Leg, qw,qdw, qr,qf,qdr,qdf)
                ld.theta = theta; ld.theta_dot = theta_d

            # ==== 抗劈叉 ====
            theta_err     = leg_R.theta - leg_L.theta
            theta_dot_err = leg_R.theta_dot - leg_L.theta_dot
            anti = KP_THETA*theta_err + KD_THETA*theta_dot_err

            # ==== 第二遍: VMC + 输出 ====
            mon = {}
            VMC_SIGN = {"Right": (1, -1), "Left": (-1, 1)}
            for side in ["Right","Left"]:
                ld = leg_R if side == "Right" else leg_L
                (phi1,phi4,phi1_d,phi4_d, L,theta,theta_d,L_d,phi2,phi3,phi5,
                 T_wheel_raw,T_Leg,F_Leg, qw,qdw, qr,qf,qdr,qdf) = state[side]

                # 抗劈叉修正 T_Leg
                if side == "Right": T_Leg += anti
                else:               T_Leg -= anti

                # VMC
                rs, fs = VMC_SIGN[side]
                J   = vmc_jacobian(phi1, phi2, phi3, phi4, phi5, L)
                Tj  = J @ np.array([F_Leg, T_Leg])
                T_rear_raw  = rs * (-Tj[0])
                T_front_raw = fs * (-Tj[1])
                T_wheel = WHEEL_POL[side] * T_wheel_raw * 0.9

                # FN
                acc = get_acc(data, freejoint_adr)
                yM_ddot = acc[0]*math.sin(pitch) + acc[2]*math.cos(pitch)
                P = F_Leg*math.cos(theta) + T_Leg*math.sin(theta)/max(L, 0.01)
                ld.FN = MW*yM_ddot + P + MW*G
                mon[side] = (F_Leg, T_Leg, T_rear_raw, T_front_raw, L, theta)

                # clip + 输出
                data.ctrl[act_ids[act_idx[side,"rear"]]]  = np.clip(T_rear_raw,  -TORQUE_LIMIT, TORQUE_LIMIT)
                data.ctrl[act_ids[act_idx[side,"front"]]] = np.clip(T_front_raw, -TORQUE_LIMIT, TORQUE_LIMIT)
                data.ctrl[act_ids[2 if side=="Right" else 5]] = np.clip(T_wheel, -TORQUE_LIMIT, TORQUE_LIMIT)

            mj.mj_step(model, data)
            viewer.sync()
            time.sleep(0.001)

            now = time.time()
            if now - last_print >= 1.0:
                ld_R = leg_R; ld_L = leg_L
                print(f"[t={data.time:.1f}s] pitch={math.degrees(pitch):.1f}°  "
                      f"FN_R={ld_R.FN:.0f}N{'着' if ld_R.grounded else '腾'} "
                      f"FN_L={ld_L.FN:.0f}N{'着' if ld_L.grounded else '腾'}")
                for side in ["Right","Left"]:
                    qr = data.qpos[joint_addr[side]["rear"]["qpos"]]
                    qf = data.qpos[joint_addr[side]["front"]["qpos"]]
                    p1, p4 = q_to_phi(side, qr, qf)
                    F_Leg_v, T_Leg_v, T_r, T_f, L_v, theta_v = mon[side]
                    print(f"  [{side}] L={L_v:.3f}m theta={math.degrees(theta_v):.1f}°  "
                          f"F_Leg={F_Leg_v:.1f}N T_Leg={T_Leg_v:.1f}Nm  "
                          f"T_rear={T_r:+.2f} T_front={T_f:+.2f}  "
                          f"phi1={math.degrees(p1):.1f}° phi4={math.degrees(p4):.1f}°")
                last_print = now

if __name__ == "__main__":
    main()
