"""
轮腿平衡控制器 — PD 角度控制 + 力矩极性标定
==============================================
外部设 phi1/phi4 目标角，内部转 raw joint，PD 产生力矩，经极性修正输出。
"""

import mujoco as mj
import numpy as np
import time
import math
import mujoco.viewer

XML_PATH = "COD-2026RoboMaster-Balance copy.xml"

model = mj.MjModel.from_xml_path(XML_PATH)
data = mj.MjData(model)

# ======================== phi 目标角 ========================
PHI1_TARGET = math.radians(150)
PHI4_TARGET = math.radians(30)

# ======================== 开环测试 ========================
TEST_REAR_TORQUE = 0     # !=0 时 rear 开环力矩, =0 时 PD 模式

# ======================== PD 增益 ========================
KP = 300.0
KD = 3.0
TORQUE_LIMIT = 3.14

# ======================== phi ↔ raw joint ========================
def phi_to_q(side, phi1, phi4):
    if side == "Right":
        return math.pi - phi1, phi4
    else:
        return phi1 - math.pi, -phi4

def q_to_phi(side, q_hip, q_sho):
    if side == "Right":
        return -q_hip + math.pi, q_sho
    else:
        return q_hip + math.pi, -q_sho

# ======================== 力矩极性 (开环测试标定) ========================
# +1: 正力矩→phi增大, -1: 正力矩→phi减小 (已对 raw joint 修正)
POLARITY = {
    ("Right", "rear"):  +1,
    ("Right", "front"): +1,
    ("Left",  "rear"):  +1,
    ("Left",  "front"): +1,
}

# ======================== 名称查询 ========================
def jnt_id(name):
    return mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, name)
def act_id(name):
    return mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, name)

ACT_NAMES = [
    "Right_front_joint_actuator", "Right_rear_joint_actuator", "Right_Wheel_joint_actuator",
    "Left_front_joint_actuator",  "Left_rear_joint_actuator",  "Left_Wheel_joint_actuator",
]
ACT_IDS = np.array([act_id(n) for n in ACT_NAMES])

# joint → actuator index
ACT_IDX = {
    ("Right", "rear"):  1, ("Right", "front"): 0,
    ("Left",  "rear"):  4, ("Left",  "front"): 3,
}

joint_addr = {}
for side in ["Right", "Left"]:
    joint_addr[side] = {}
    for k in ["front", "rear", "wheel"]:
        jid = jnt_id(f"{side}_{k}_joint")
        joint_addr[side][k] = {"qpos": model.jnt_qposadr[jid], "dof": model.jnt_dofadr[jid]}

# ======================== 目标 raw 角 ========================
q_target = {}
q_target["Right", "rear"],  q_target["Right", "front"] = phi_to_q("Right", PHI1_TARGET, PHI4_TARGET)
q_target["Left",  "rear"],  q_target["Left",  "front"] = phi_to_q("Left",  PHI1_TARGET, PHI4_TARGET)

# ======================== 主循环 ========================
def main():
    mode = f"开环 rear={TEST_REAR_TORQUE:+.1f}Nm" if TEST_REAR_TORQUE != 0 else "PD"
    print("=" * 50)
    print(f"  [{mode}]  phi1={math.degrees(PHI1_TARGET):.0f}°  phi4={math.degrees(PHI4_TARGET):.0f}°")
    for side in ["Right", "Left"]:
        print(f"  [{side}] raw rear={math.degrees(q_target[side,'rear']):+.0f}°  "
              f"front={math.degrees(q_target[side,'front']):+.0f}°")
    print(f"  极性: { {k: ('+' if v>0 else '-') for k,v in POLARITY.items()} }")
    print("=" * 50)

    # 初始化为目标角
    for (side, jn), qref in q_target.items():
        data.qpos[joint_addr[side][jn]["qpos"]] = qref
    mj.mj_forward(model, data)

    last_print = time.time()

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            # 轮子置零
            data.ctrl[ACT_IDS[2]] = 0.0
            data.ctrl[ACT_IDS[5]] = 0.0

            # PD + 极性 / 开环测试
            for side in ["Right", "Left"]:
                for jn in ["rear", "front"]:
                    adr = joint_addr[side][jn]
                    q  = data.qpos[adr["qpos"]]
                    qd = data.qvel[adr["dof"]]
                    if TEST_REAR_TORQUE != 0:
                        if jn == "rear":
                            tau = POLARITY[side, jn] * TEST_REAR_TORQUE  # 开环
                        else:
                            tau = 0.0                                    # front 不控
                    else:
                        tau_pd = KP * (q_target[side, jn] - q) + KD * (0.0 - qd)
                        tau = POLARITY[side, jn] * tau_pd                # PD
                    data.ctrl[ACT_IDS[ACT_IDX[side, jn]]] = np.clip(tau, -TORQUE_LIMIT, TORQUE_LIMIT)

            mj.mj_step(model, data)
            viewer.sync()
            time.sleep(0.001)

            # 每秒 print
            now = time.time()
            if now - last_print >= 1.0:
                print(f"[t={data.time:.1f}s]")
                for side in ["Right", "Left"]:
                    qr = data.qpos[joint_addr[side]["rear"]["qpos"]]
                    qf = data.qpos[joint_addr[side]["front"]["qpos"]]
                    p1, p4 = q_to_phi(side, qr, qf)
                    print(f"  [{side}] phi1={math.degrees(p1):.1f}°  phi4={math.degrees(p4):.1f}°  "
                          f"| raw rear={math.degrees(qr):+.1f}° front={math.degrees(qf):+.1f}°")
                last_print = now


if __name__ == "__main__":
    main()
