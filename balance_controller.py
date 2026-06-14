"""
轮腿平衡控制器 — 腿长输入 + IK + PD + IMU
==========================================
引用 robot_config.py 的参数、运动学和 MuJoCo 初始化。
机体 freejoint 浮空，IMU 实时读取 pitch。
"""

import mujoco as mj
import numpy as np
import time
import math
import mujoco.viewer
from robot_config import (
    init_model, fk, ik, phi_to_q, q_to_phi,
    get_pitch, get_pitch_dot,
)

# ======================== 输入: 目标腿长 ========================
LEG_LENGTH_REF = 0.30      # 目标等效腿长 [m]

# ======================== 轮子力矩测试 ========================
TEST_WHEEL_TORQUE = 1.0     # !=0 时开环测试轮子, =0 时正常模式

# ======================== PD 增益 ========================
KP = 300.0; KD = 3.0; TORQUE_LIMIT = 3.14

# ======================== 初始化 MuJoCo ========================
XML_PATH = "COD-2026RoboMaster-Balance copy.xml"
model, data, act_ids, act_idx, joint_addr, freejoint_adr = init_model(XML_PATH)

# ======================== 主循环 ========================
def main():
    # --- IK: 腿长 → phi ---
    phi1_ref, phi4_ref = ik(LEG_LENGTH_REF)

    # --- phi → raw ---
    q_tgt = {}
    q_tgt["Right","rear"],  q_tgt["Right","front"] = phi_to_q("Right", phi1_ref, phi4_ref)
    q_tgt["Left", "rear"],  q_tgt["Left", "front"] = phi_to_q("Left",  phi1_ref, phi4_ref)

    mode = f"开环轮子={TEST_WHEEL_TORQUE:+.1f}Nm" if TEST_WHEEL_TORQUE != 0 else "PD"
    print("=" * 55)
    print(f"  [{mode}]  L_ref = {LEG_LENGTH_REF:.3f}m")
    print(f"  IK → phi1 = {math.degrees(phi1_ref):.1f}°  phi4 = {math.degrees(phi4_ref):.1f}°")
    for side in ["Right","Left"]:
        print(f"  [{side}] raw rear = {math.degrees(q_tgt[side,'rear']):+.0f}°  "
              f"front = {math.degrees(q_tgt[side,'front']):+.0f}°")
    print("=" * 55)

    # --- 初始化关节角 ---
    for (side, jn), qref in q_tgt.items():
        data.qpos[joint_addr[side][jn]["qpos"]] = qref
    mj.mj_forward(model, data)
    last_print = time.time()

    # --- Viewer ---
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            # 轮子 (测试 or 置零, 右手定则: 外侧看CCW为正)
            if TEST_WHEEL_TORQUE != 0:
                data.ctrl[act_ids[2]] = -TEST_WHEEL_TORQUE   # Right wheel (反)
                data.ctrl[act_ids[5]] =  TEST_WHEEL_TORQUE   # Left wheel
            else:
                data.ctrl[act_ids[2]] = 0.0
                data.ctrl[act_ids[5]] = 0.0

            # IMU 读 pitch (从 freejoint)
            pitch     = get_pitch(data, freejoint_adr)
            pitch_dot = get_pitch_dot(data, freejoint_adr)

            # PD 角度控制 (raw 空间)
            for side in ["Right","Left"]:
                for jn in ["rear","front"]:
                    a   = joint_addr[side][jn]
                    q   = data.qpos[a["qpos"]]
                    qd  = data.qvel[a["dof"]]
                    tau = KP * (q_tgt[side,jn] - q) + KD * (0.0 - qd)
                    data.ctrl[act_ids[act_idx[side,jn]]] = np.clip(tau, -TORQUE_LIMIT, TORQUE_LIMIT)

            mj.mj_step(model, data)
            viewer.sync()
            time.sleep(0.001)

            # 每秒打印
            now = time.time()
            if now - last_print >= 1.0:
                print(f"[t={data.time:.1f}s] pitch={math.degrees(pitch):.1f}°")
                for side in ["Right","Left"]:
                    qr = data.qpos[joint_addr[side]["rear"]["qpos"]]
                    qf = data.qpos[joint_addr[side]["front"]["qpos"]]
                    p1, p4 = q_to_phi(side, qr, qf)
                    L = fk(p1, p4)[0]
                    print(f"  [{side}] L={L:.3f}m  "
                          f"phi1={math.degrees(p1):.1f}°  phi4={math.degrees(p4):.1f}°  "
                          f"| raw rear={math.degrees(qr):+.1f}° front={math.degrees(qf):+.1f}°")
                last_print = now

if __name__ == "__main__":
    main()
