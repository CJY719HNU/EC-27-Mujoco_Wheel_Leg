"""
VMC + LQR 轮腿平衡控制器
=========================
F_Leg 腿长PD + T_Leg(LQR) → VMC Jᵀ → T_rear/T_front → LQR → T_wheel
"""

import mujoco as mj
import numpy as np
import time
import math
import mujoco.viewer
from robot_config import (
    init_model, fk_deriv, ik, vmc_jacobian,
    phi_to_q, q_to_phi, phi_dot,
    get_pitch, get_pitch_dot, get_acc, lookup_k_mat, build_k_table,
    M, G, R, MW,
)

XML_PATH = "COD-2026RoboMaster-Balance copy.xml"
model, data, act_ids, act_idx, joint_addr, freejoint_adr, _ = init_model(XML_PATH)

LEG_LENGTH_REF = 0.20
KP_LEG = 300.0;  KD_LEG = 100.0
KP_THETA = 6.0;  KD_THETA = 0.2
TORQUE_LIMIT = 100.0
WHEEL_POL = {"Right": 1, "Left": -1}
VMC_SIGN = {"Right": (1, -1), "Left": (-1, 1)}

class LegData:
    def __init__(self):
        self.x = 0.0; self.x0 = None
        self.FN = 100.0; self.grounded = True
        self.theta = 0.0; self.theta_dot = 0.0

leg_R, leg_L = LegData(), LegData()

def main():
    phi1_ref, phi4_ref = ik(LEG_LENGTH_REF)
    q_tgt = {}
    q_tgt["Right","rear"], q_tgt["Right","front"] = phi_to_q("Right", phi1_ref, phi4_ref)
    q_tgt["Left", "rear"], q_tgt["Left", "front"] = phi_to_q("Left",  phi1_ref, phi4_ref)

    for (side,jn), qref in q_tgt.items():
        data.qpos[joint_addr[side][jn]["qpos"]] = qref
    mj.mj_forward(model, data)

    build_k_table()  # 预计算 K 查表
    last_print = time.time()

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            pitch     = get_pitch(data, freejoint_adr)
            pitch_dot = get_pitch_dot(data, freejoint_adr)
            acc       = get_acc(data, freejoint_adr)

            state = {}
            for side in ["Right","Left"]:
                a  = joint_addr[side]; ld = leg_R if side=="Right" else leg_L
                qr  = data.qpos[a["rear"]["qpos"]];   qdr = data.qvel[a["rear"]["dof"]]
                qf  = data.qpos[a["front"]["qpos"]];  qdf = data.qvel[a["front"]["dof"]]
                qw  = data.qpos[a["wheel"]["qpos"]];  qdw = data.qvel[a["wheel"]["dof"]]
                phi1, phi4 = q_to_phi(side, qr, qf)
                phi1_d, phi4_d = phi_dot(side, qdr, qdf)
                L, theta, theta_d, L_d, phi2, phi3, phi5, _ = \
                    fk_deriv(phi1, phi4, phi1_d, phi4_d, pitch, pitch_dot)
                if ld.x0 is None: ld.x0 = qw
                ld.x = R*(qw - ld.x0); x_dot = R*qdw

                Height   = L*math.cos(theta)
                Height_d = L_d*math.cos(theta) - L*math.sin(theta)*theta_d
                F_Leg = 0.5*M*G*math.cos(theta) \
                      + KP_LEG*(LEG_LENGTH_REF - Height) + KD_LEG*(0.0 - Height_d)

                K_mat = lookup_k_mat(L, grounded=ld.grounded)
                X = np.array([theta, theta_d, ld.x, x_dot, pitch, pitch_dot])
                u = K_mat @ (-X)

                yM_ddot = acc[0]*math.sin(pitch) + acc[2]*math.cos(pitch)
                P = F_Leg*math.cos(theta) + u[1]*math.sin(theta)/max(L, 0.01)
                ld.FN = MW*yM_ddot + P + MW*G
                ld.grounded = ld.FN >= 20.0

                state[side] = (phi1,phi4,phi1_d,phi4_d, L,theta,theta_d,phi2,phi3,phi5,
                               u[0],u[1],F_Leg)
                ld.theta = theta; ld.theta_dot = theta_d

            anti = KP_THETA*(leg_R.theta - leg_L.theta) + KD_THETA*(leg_R.theta_dot - leg_L.theta_dot)

            for side in ["Right","Left"]:
                ld = leg_R if side=="Right" else leg_L
                (phi1,phi4,phi1_d,phi4_d, L,theta,theta_d,phi2,phi3,phi5,
                 T_wheel_raw,T_Leg,F_Leg) = state[side]

                if side == "Right": T_Leg += anti
                else:               T_Leg -= anti

                rs, fs = VMC_SIGN[side]
                J  = vmc_jacobian(phi1, phi2, phi3, phi4, phi5, L)
                Tj = J @ np.array([F_Leg, T_Leg])
                T_rear  = rs * (-Tj[0])
                T_front = fs * (-Tj[1])
                T_wheel = WHEEL_POL[side] * T_wheel_raw

                data.ctrl[act_ids[act_idx[side,"rear"]]]  = np.clip(T_rear,  -TORQUE_LIMIT, TORQUE_LIMIT)
                data.ctrl[act_ids[act_idx[side,"front"]]] = np.clip(T_front, -TORQUE_LIMIT, TORQUE_LIMIT)
                data.ctrl[act_ids[2 if side=="Right" else 5]] = np.clip(T_wheel, -TORQUE_LIMIT, TORQUE_LIMIT)

            mj.mj_step(model, data)
            viewer.sync()
            time.sleep(0.001)

            now = time.time()
            if now - last_print >= 1.0:
                print(f"[t={data.time:.1f}s] pitch={math.degrees(pitch):.1f}°  "
                      f"FN_R={leg_R.FN:.0f}N FN_L={leg_L.FN:.0f}N")
                last_print = now

if __name__ == "__main__":
    main()
