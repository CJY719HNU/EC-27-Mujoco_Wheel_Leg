"""
架起来观察 — PD 保持 rear=-45° / front=45°，机体悬空
"""

import mujoco as mj
import numpy as np
import time
import math
import mujoco.viewer

XML_PATH = "COD-2026RoboMaster-Balance copy.xml"

model = mj.MjModel.from_xml_path(XML_PATH)
data = mj.MjData(model)

# ======================== 参考角 ========================
Q_HIP_REF  = math.radians(-45)   # rear_joint
Q_SHO_REF  = math.radians(45)    # front_joint

# ======================== 名称查询 ========================
def jnt_id(name):
    return mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, name)

def act_id(name):
    return mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, name)

ACT_NAMES = [
    "Right_front_joint_actuator",
    "Right_rear_joint_actuator",
    "Right_Wheel_joint_actuator",
    "Left_front_joint_actuator",
    "Left_rear_joint_actuator",
    "Left_Wheel_joint_actuator",
]
ACT_IDS = [act_id(n) for n in ACT_NAMES]

JOINT_ACT_MAP = [
    ("Right_front_joint", 0, Q_SHO_REF),
    ("Right_rear_joint",  1, -1*Q_HIP_REF),
    ("Left_front_joint",  3, -1*Q_SHO_REF),
    ("Left_rear_joint",   4, Q_HIP_REF),
]

joint_info = {}
for jname, aidx, qref in JOINT_ACT_MAP:
    jid = jnt_id(jname)
    joint_info[jname] = {
        "qpos_adr": model.jnt_qposadr[jid],
        "dof_adr":  model.jnt_dofadr[jid],
        "act_idx":  aidx,
        "q_ref":    qref,
    }

KP = 100.0
KD = 1.0

# ======================== 初始化 ========================
# base_link 固定在世界中 (freejoint 已注释)

for jname, _, qref in JOINT_ACT_MAP:
    data.qpos[joint_info[jname]["qpos_adr"]] = qref

mj.mj_forward(model, data)

# FK 验证
L_ACT, L_SLV, L5 = 0.21, 0.25, 0.0
for side in ["Right", "Left"]:
    qr = data.qpos[joint_info[f"{side}_rear_joint"]["qpos_adr"]]
    qf = data.qpos[joint_info[f"{side}_front_joint"]["qpos_adr"]]
    # 按轴方向: phi1(q=0)=180°, phi4(q=0)=0°
    if side == "Right":
        p1 = math.degrees(-qr + math.pi)       # rear  axis 0 -1 0
        p4 = math.degrees(qf)                  # front axis 0  1 0
    else:
        p1 = math.degrees(qr + math.pi)        # rear  axis 0  1 0
        p4 = math.degrees(-qf)                 # front axis 0 -1 0
    print(f"[{side}] rear={math.degrees(qr):+.0f}° phi1={p1:.0f}° | front={math.degrees(qf):+.0f}° phi4={p4:.0f}°")

print(f"\n机体固定, 观察 viewer ...")

# ======================== Viewer ========================
last_print = time.time()

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        data.ctrl[ACT_IDS[2]] = 0.0
        data.ctrl[ACT_IDS[5]] = 0.0
        for jname, _, _ in JOINT_ACT_MAP:
            info = joint_info[jname]
            q  = data.qpos[info["qpos_adr"]]
            qd = data.qvel[info["dof_adr"]]
            data.ctrl[ACT_IDS[info["act_idx"]]] = KP * (info["q_ref"] - q) + KD * (0.0 - qd)
        mj.mj_step(model, data)
        viewer.sync()
        time.sleep(0.001)

        # 每秒输出实际角度
        now = time.time()
        if now - last_print >= 1.0:
            for side in ["Right", "Left"]:
                qr = data.qpos[joint_info[f"{side}_rear_joint"]["qpos_adr"]]
                qf = data.qpos[joint_info[f"{side}_front_joint"]["qpos_adr"]]
                if side == "Right":
                    p1 = math.degrees(-qr + math.pi)       # rear  axis 0 -1 0
                    p4 = math.degrees(qf)                  # front axis 0  1 0
                else:
                    p1 = math.degrees(qr + math.pi)        # rear  axis 0  1 0
                    p4 = math.degrees(-qf)                 # front axis 0 -1 0
                print(f"[t={data.time:.1f}s] [{side}] rear={math.degrees(qr):+.2f}° phi1={p1:.1f}° | front={math.degrees(qf):+.2f}° phi4={p4:.1f}°")
            last_print = now
