"""
轮腿平衡控制器 — 腿长输入 + IK + PD
====================================
输入目标腿长 L_ref → IK解算 phi1/phi4 → PD
约束: phi1 + phi4 = 180° (对称腿构型)
"""

import mujoco as mj
import numpy as np
import time
import math
import mujoco.viewer

XML_PATH = "COD-2026RoboMaster-Balance copy.xml"

model = mj.MjModel.from_xml_path(XML_PATH)
data = mj.MjData(model)

# ======================== 输入: 目标腿长 ========================
LEG_LENGTH_REF = 0.30      # 目标等效腿长 [m]

# ======================== PD 增益 ========================
KP = 300.0; KD = 3.0; TORQUE_LIMIT = 3.14

# ======================== 物理参数 ========================
L_ACT, L_SLV, L5 = 0.21, 0.25, 0.0

# ======================== phi ↔ raw ========================
def phi_to_q(side, phi1, phi4):
    if side == "Right": return math.pi - phi1, phi4
    else:               return phi1 - math.pi, -phi4

def q_to_phi(side, q_hip, q_sho):
    if side == "Right": return -q_hip + math.pi, q_sho
    else:               return q_hip + math.pi, -q_sho

# ======================== FK (给定 phi1, phi4 → L) ========================
def fk(phi1, phi4):
    l1,l2,l3,l4,l5 = L_ACT, L_SLV, L_SLV, L_ACT, L5
    xD = l5 + l4*math.cos(phi4); yD = l4*math.sin(phi4)
    xB = l1*math.cos(phi1);       yB = l1*math.sin(phi1)
    BD = math.sqrt((xD-xB)**2 + (yD-yB)**2)
    A0 = 2*l2*(xD-xB); B0 = 2*l2*(yD-yB); C0 = l2**2 + BD**2 - l3**2
    disc = max(0.0, A0**2 + B0**2 - C0**2)
    phi2 = 2*math.atan2(B0 + math.sqrt(disc), A0 + C0)
    xC = l1*math.cos(phi1) + l2*math.cos(phi2)
    yC = l1*math.sin(phi1) + l2*math.sin(phi2)
    phi3 = math.atan2(yC-yD, xC-xD)
    phi5 = math.atan2(yC, xC - l5/2)
    L = math.sqrt((xC - l5/2)**2 + yC**2)
    theta = phi5 - math.pi/2
    return L, theta, phi2, phi3, phi5

# ======================== IK (L → phi1, 约束 phi1+phi4=pi) ========================
def ik(L_ref):
    """给定腿长, 在 phi1 ∈ [90°, 180°] 范围内二分搜索, phi4 = pi - phi1"""
    lo, hi = math.radians(90), math.radians(180)
    for _ in range(30):
        mid = (lo + hi) / 2
        L_mid, _, _, _, _ = fk(mid, math.pi - mid)
        if L_mid < L_ref: hi = mid
        else: lo = mid
    phi1 = (lo + hi) / 2
    return phi1, math.pi - phi1

# ======================== 名称查询 ========================
def jnt_id(name): return mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, name)
def act_id(name): return mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, name)

ACT_NAMES = [
    "Right_front_joint_actuator","Right_rear_joint_actuator","Right_Wheel_joint_actuator",
    "Left_front_joint_actuator", "Left_rear_joint_actuator", "Left_Wheel_joint_actuator",
]
ACT_IDS = np.array([act_id(n) for n in ACT_NAMES])
ACT_IDX = {("Right","rear"):1, ("Right","front"):0, ("Left","rear"):4, ("Left","front"):3}

joint_addr = {}
for side in ["Right","Left"]:
    joint_addr[side] = {}
    for k in ["front","rear","wheel"]:
        jid = jnt_id(f"{side}_{k}_joint")
        joint_addr[side][k] = {"qpos": model.jnt_qposadr[jid], "dof": model.jnt_dofadr[jid]}

# ======================== 主循环 ========================
def main():
    phi1_ref, phi4_ref = ik(LEG_LENGTH_REF)

    q_tgt = {}
    q_tgt["Right","rear"], q_tgt["Right","front"] = phi_to_q("Right", phi1_ref, phi4_ref)
    q_tgt["Left", "rear"], q_tgt["Left", "front"] = phi_to_q("Left",  phi1_ref, phi4_ref)

    print("=" * 55)
    print(f"  L_ref = {LEG_LENGTH_REF:.3f}m")
    print(f"  IK → phi1 = {math.degrees(phi1_ref):.1f}°  phi4 = {math.degrees(phi4_ref):.1f}°")
    for side in ["Right","Left"]:
        print(f"  [{side}] raw rear={math.degrees(q_tgt[side,'rear']):+.0f}°  front={math.degrees(q_tgt[side,'front']):+.0f}°")
    print("=" * 55)

    for (side,jn), qref in q_tgt.items():
        data.qpos[joint_addr[side][jn]["qpos"]] = qref
    mj.mj_forward(model, data)
    last_print = time.time()

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            data.ctrl[ACT_IDS[2]] = 0.0; data.ctrl[ACT_IDS[5]] = 0.0

            for side in ["Right","Left"]:
                for jn in ["rear","front"]:
                    a = joint_addr[side][jn]
                    q  = data.qpos[a["qpos"]]
                    qd = data.qvel[a["dof"]]
                    tau = KP*(q_tgt[side,jn] - q) + KD*(0.0 - qd)
                    data.ctrl[ACT_IDS[ACT_IDX[side,jn]]] = np.clip(tau, -TORQUE_LIMIT, TORQUE_LIMIT)

            mj.mj_step(model, data)
            viewer.sync()
            time.sleep(0.001)

            now = time.time()
            if now - last_print >= 1.0:
                print(f"[t={data.time:.1f}s]")
                for side in ["Right","Left"]:
                    qr = data.qpos[joint_addr[side]["rear"]["qpos"]]
                    qf = data.qpos[joint_addr[side]["front"]["qpos"]]
                    p1, p4 = q_to_phi(side, qr, qf)
                    L, _, _, _, _ = fk(p1, p4)
                    print(f"  [{side}] L={L:.3f}m  phi1={math.degrees(p1):.1f}°  phi4={math.degrees(p4):.1f}°  "
                          f"| raw rear={math.degrees(qr):+.1f}° front={math.degrees(qf):+.1f}°")
                last_print = now

if __name__ == "__main__":
    main()
