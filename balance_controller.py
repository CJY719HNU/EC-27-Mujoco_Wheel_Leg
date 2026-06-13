"""
开环力矩测试 — 只给 rear joint (phi1) 恒力矩, 观察 phi1 变化
"""

import mujoco as mj
import numpy as np
import time
import math
import mujoco.viewer

XML_PATH = "COD-2026RoboMaster-Balance copy.xml"

model = mj.MjModel.from_xml_path(XML_PATH)
data = mj.MjData(model)

# ======================== 测试参数 ========================
TEST_FRONT_TORQUE = 0   # 开环力矩 [Nm], 正值, 只发 front joint
PHI1_START = math.radians(170)   # 初始 rear 角
PHI4_START = math.radians(10)    # 初始 front 角 (仅初始位置, 不控)

# ======================== phi ↔ raw joint ========================
# 轴: Right rear=0 -1 0 front=0  1 0  |  Left rear=0  1 0 front=0 -1 0

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

joint_addr = {}
for side in ["Right", "Left"]:
    joint_addr[side] = {}
    for k in ["front", "rear", "wheel"]:
        jid = jnt_id(f"{side}_{k}_joint")
        joint_addr[side][k] = {"qpos": model.jnt_qposadr[jid], "dof": model.jnt_dofadr[jid]}

# ======================== 主循环 ========================
def main():
    q_hip_R, q_sho_R = phi_to_q("Right", PHI1_START, PHI4_START)
    q_hip_L, q_sho_L = phi_to_q("Left",  PHI1_START, PHI4_START)

    print("=" * 50)
    print(f"  开环 front 力矩测试: {TEST_FRONT_TORQUE:+.1f} Nm")
    print(f"  初始 phi1={math.degrees(PHI1_START):.0f}°  phi4={math.degrees(PHI4_START):.0f}°")
    print(f"  R rear raw={math.degrees(q_hip_R):+.0f}°  L rear raw={math.degrees(q_hip_L):+.0f}°")
    print("=" * 50)

    # 初始化为起始角
    data.qpos[joint_addr["Right"]["rear"]["qpos"]]  = q_hip_R
    data.qpos[joint_addr["Right"]["front"]["qpos"]] = q_sho_R
    data.qpos[joint_addr["Left"]["rear"]["qpos"]]   = q_hip_L
    data.qpos[joint_addr["Left"]["front"]["qpos"]]  = q_sho_L
    mj.mj_forward(model, data)

    last_print = time.time()

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            # 所有置零
            data.ctrl[:] = 0.0

            # 只给 front joint 开环力矩 (右前反)
            data.ctrl[ACT_IDS[0]] = -TEST_FRONT_TORQUE   # Right_front (反)
            data.ctrl[ACT_IDS[3]] =  TEST_FRONT_TORQUE   # Left_front

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
