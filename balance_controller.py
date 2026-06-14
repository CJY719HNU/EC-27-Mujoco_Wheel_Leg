"""
VMC + LQR 轮腿平衡控制器 (精简版)
"""
import mujoco as mj, numpy as np, time, math, mujoco.viewer
from robot_config import (
    init_model, fk_deriv, ik, vmc_jacobian,
    phi_to_q, q_to_phi, phi_dot,
    get_pitch, get_pitch_dot, get_acc, lookup_k_mat, build_k_table,
    M, G, R, MW,
)

model, data, act_ids, act_idx, joint_addr, freejoint_adr, _ = \
    init_model("COD-2026RoboMaster-Balance copy.xml")

L_REF = 0.20; KP_L=1200.0; KD_L=200.0; KP_T=12.0; KD_T=0.2; TLIM=100.0
WP = (1, -1)  # wheel_pol R,L
VS = ((1,-1),(-1,1))  # VMC_SIGN R,L

class LD:
    def __init__(s): s.x=s.x0=0.0; s.FN=100.0; s.g=True; s.th=s.td=0.0

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
    lp = time.time()

    with mujoco.viewer.launch_passive(model, data) as v:
        while v.is_running():
            pitch = get_pitch(data, freejoint_adr)
            pitch_d = get_pitch_dot(data, freejoint_adr)
            acc = get_acc(data, freejoint_adr)

            for side, ld, wp, (rs,fs) in [("Right",Rl,WP[0],VS[0]),("Left",Ll,WP[1],VS[1])]:
                a = joint_addr[side]
                qr,qdr = data.qpos[a["rear"]["qpos"]],  data.qvel[a["rear"]["dof"]]
                qf,qdf = data.qpos[a["front"]["qpos"]], data.qvel[a["front"]["dof"]]
                qw,qdw = data.qpos[a["wheel"]["qpos"]], data.qvel[a["wheel"]["dof"]]
                p1,p4 = q_to_phi(side,qr,qf); p1d,p4d = phi_dot(side,qdr,qdf)
                L,th,thd,Ld,p2,p3,p5,_ = fk_deriv(p1,p4,p1d,p4d,pitch,pitch_d)
                if ld.x0==0.0 and qw!=0: ld.x0=qw
                ld.x=R*(qw-ld.x0); xd=R*qdw
                H=L*math.cos(th); Hd=Ld*math.cos(th)-L*math.sin(th)*thd
                FL=0.5*M*G*math.cos(th)+KP_L*(L_REF-H)+KD_L*(0-Hd)
                yMdd=acc[0]*math.sin(pitch)+acc[2]*math.cos(pitch)
                ld.FN=MW*yMdd+FL*math.cos(th)+MW*G; ld.g=ld.FN>=20.0
                if not ld.g: ld.x0=qw
                K=lookup_k_mat(L,ld.g)
                # 反馈矩阵
                u=-K@np.array([th,thd,ld.x,xd,-pitch,-pitch_d])
                #这是
                TL=u[1] #+(KP_T*(Rl.th-Ll.th)+KD_T*(Rl.td-Ll.td))*(-1 if side=="Right" else -1)
                J=vmc_jacobian(p1,p2,p3,p4,p5,L); Tj=J@[FL,TL]
                Tr=rs*(-Tj[0]); Tf=fs*(-Tj[1]); Tw=wp*u[0]
                data.ctrl[act_ids[act_idx[side,"rear"]]]  = np.clip(Tr,-TLIM,TLIM)
                data.ctrl[act_ids[act_idx[side,"front"]]] = np.clip(Tf,-TLIM,TLIM)
                data.ctrl[act_ids[2 if side=="Right" else 5]] = np.clip(Tw,-TLIM,TLIM)
                ld.th=th; ld.td=thd

            mj.mj_step(model, data); v.sync(); time.sleep(0.001)
            now=time.time()
            if now-lp>=1.0:
                print(f"[t={data.time:.1f}s] pitch={math.degrees(pitch):.1f}  FN_R={Rl.FN:.0f}N FN_L={Ll.FN:.0f}N  th_R={math.degrees(Rl.th):.1f} th_L={math.degrees(Ll.th):.1f}")
                lp=now

if __name__=="__main__": main()
