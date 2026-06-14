"""
VMC + LQR 轮腿平衡控制器 (精简版)
"""
import mujoco as mj, numpy as np, time, math, mujoco.viewer
from robot_config import (
    init_model, fk_deriv, ik, vmc_jacobian,
    phi_to_q, q_to_phi, phi_dot,
    get_pitch, get_pitch_dot, get_acc, get_yaw, get_yaw_dot,
    lookup_k_mat, build_k_table,
    M, G, MW,
)

model, data, act_ids, act_idx, joint_addr, freejoint_adr, wheel_body = \
    init_model("COD-2026RoboMaster-Balance copy.xml")

L_REF = 0.30; KP_L=1500.0; KD_L=400.0; TLIM=100.0
# 转向环 (yaw) PD 增益
KP_YAW = 30.0; KD_YAW = 1.0
WP = (1, -1)  # wheel_pol R,L
VS = ((1,-1),(-1,1))  # VMC_SIGN R,L
FIXED_BODY = freejoint_adr < 0  # 机体是否被固定

class LD:
    def __init__(s): s.FN=100.0; s.g=True; s.th=s.td=0.0; s.Tr=s.Tf=s.Tw=0.0

Rl, Ll = LD(), LD()

def main():
    p1r, p4r = ik(L_REF)
    qRr,qRf = phi_to_q("Right",p1r,p4r); qLr,qLf = phi_to_q("Left",p1r,p4r)
    data.qpos[joint_addr["Right"]["rear"]["qpos"]]  = qRr
    data.qpos[joint_addr["Right"]["front"]["qpos"]] = qRf
    data.qpos[joint_addr["Left"]["rear"]["qpos"]]   = qLr
    data.qpos[joint_addr["Left"]["front"]["qpos"]]  = qLf
    mj.mj_forward(model, data)
    build_k_table()
    x0_body = data.qpos[freejoint_adr] if not FIXED_BODY else 0.0  # 初始 x (里程计零点)
    yaw0 = get_yaw(data, freejoint_adr) if not FIXED_BODY else 0.0   # 目标 yaw
    lp = time.time()

    with mujoco.viewer.launch_passive(model, data) as v:
        while v.is_running():
            if FIXED_BODY:
                pitch = 0.0; pitch_d = 0.0; acc = [0.0, 0.0, 0.0]
            else:
                pitch = get_pitch(data, freejoint_adr)
                pitch_d = get_pitch_dot(data, freejoint_adr)
                acc = get_acc(data, freejoint_adr)

            # 里程计: 用 freejoint 绝对位置（仿真真值），避免轮子编码器积分误差
            if not FIXED_BODY:
                body_x  = data.qpos[freejoint_adr] - x0_body
                body_xd = data.qvel[freejoint_adr]
                yaw     = get_yaw(data, freejoint_adr)
                yaw_d   = get_yaw_dot(data, freejoint_adr)
            else:
                body_x = 0.0; body_xd = 0.0
                yaw = 0.0; yaw_d = 0.0

            # 转向环: yaw 偏离时左右轮差速
            yaw_err = yaw - yaw0
            Tyaw = KP_YAW * yaw_err + KD_YAW * yaw_d

            for side, ld, wp, (rs,fs) in [("Right",Rl,WP[0],VS[0]),("Left",Ll,WP[1],VS[1])]:
                a = joint_addr[side]
                qr,qdr = data.qpos[a["rear"]["qpos"]],  data.qvel[a["rear"]["dof"]]
                qf,qdf = data.qpos[a["front"]["qpos"]], data.qvel[a["front"]["dof"]]
                p1,p4 = q_to_phi(side,qr,qf); p1d,p4d = phi_dot(side,qdr,qdf)
                L,th,thd,Ld,p2,p3,p5,_ = fk_deriv(p1,p4,p1d,p4d,pitch,pitch_d)
                H=L*math.cos(th); Hd=Ld*math.cos(th)-L*math.sin(th)*thd
                #腿长控制PID + 重力前馈
                FL_FF = 50.0  # 前馈力 [N]
                FL = 0.5*M*G*math.cos(th) + FL_FF + KP_L*(L_REF-H) + KD_L*(0-Hd)
                yMdd=acc[0]*math.sin(pitch)+acc[2]*math.cos(pitch)
                ld.FN=MW*yMdd+FL*math.cos(th)+MW*G; ld.g=ld.FN>=20.0
                K=lookup_k_mat(L,ld.g)
                # 反馈矩阵
                u=-K@np.array([th,thd,body_x,body_xd,-pitch,-pitch_d])
                TL=u[1]
                J=vmc_jacobian(p1,p2,p3,p4,p5,L); Tj=J@[FL,TL]
                Tr=rs*(-Tj[0]); Tf=fs*(-Tj[1]); Tw=wp*u[0]; ld.Tr=Tr; ld.Tf=Tf; ld.Tw=Tw
                data.ctrl[act_ids[act_idx[side,"rear"]]]  = np.clip(Tr,-TLIM,TLIM)
                data.ctrl[act_ids[act_idx[side,"front"]]] = np.clip(Tf,-TLIM,TLIM)
                Tyaw_sign = -1 if side=="Right" else 1 #转向环
                data.ctrl[act_ids[2 if side=="Right" else 5]] = np.clip(Tw + Tyaw_sign * Tyaw, -TLIM, TLIM)
                ld.th=th; ld.td=thd

            # ---- 扰动: 定时对轮子施加水平力 ----
            # if FIXED_BODY:
            #     t = data.time
            #     cycle = t % PERT_INTERVAL
            #     if cycle < PERT_DURATION and t - last_pert_time > PERT_DURATION:
            #         last_pert_time = t
            #     if last_pert_time > 0 and t - last_pert_time < PERT_DURATION:
            #         sign = 1.0 if (int(t / PERT_INTERVAL) % 2 == 0) else -1.0
            #         fx = sign * PERT_FORCE
            #         data.xfrc_applied[wheel_body["Right"]][0] = fx
            #         data.xfrc_applied[wheel_body["Left"]][0]  = fx
            #     else:
            #         data.xfrc_applied[wheel_body["Right"]][0] = 0.0
            #         data.xfrc_applied[wheel_body["Left"]][0]  = 0.0

            mj.mj_step(model, data); v.sync(); time.sleep(0.001)
            now=time.time()
            if now-lp>=1.0:
                print(f"[t={data.time:.1f}s] pitch={math.degrees(pitch):.1f} yaw={math.degrees(yaw):.1f} yaw_tq={Tyaw:+.1f}  FN_R={Rl.FN:.0f}N FN_L={Ll.FN:.0f}N")
                print(f"  R: Tr={Rl.Tr:+.1f} Tf={Rl.Tf:+.1f} Tw={Rl.Tw:+.1f}  th={math.degrees(Rl.th):.1f}")
                print(f"  L: Tr={Ll.Tr:+.1f} Tf={Ll.Tf:+.1f} Tw={Ll.Tw:+.1f}  th={math.degrees(Ll.th):.1f}")
                lp=now

if __name__=="__main__": main()
