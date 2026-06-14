"""
机器人参数 & 运动学
===================
phi↔raw 变换 + FK + IK，供各控制器 import 使用。
"""

import math
import numpy as np
import mujoco as mj

# ======================== 五杆机构参数 ========================
L_ACT = 0.21               # 主动连杆长度 (rear_link / front_link) [m]
L_SLV = 0.25               # 从动连杆长度 (rear_child1 / front_child1+2+3) [m]
L5    = 0.0                # 髋/肩关节水平间距 [m]
M     = 15.040             # 机体质量 [kg]
G     = 9.81               # 重力加速度 [m/s²]
MW    = 0.36               # 轮子质量 [kg]
R     = 0.077              # 轮子半径 [m]

# ======================== 执行器名称 ========================
ACT_NAMES = [
    "Right_front_joint_actuator", "Right_rear_joint_actuator", "Right_Wheel_joint_actuator",
    "Left_front_joint_actuator",  "Left_rear_joint_actuator",  "Left_Wheel_joint_actuator",
]

# ======================== phi ↔ raw joint 坐标变换 ========================
# 基准: q=0 时 phi1=180°(正后方), phi4=0°(正前方)
#   Right: rear axis = 0 -1 0    front axis = 0  1 0
#   Left:  rear axis = 0  1 0    front axis = 0 -1 0

def phi_to_q(side, phi1, phi4):
    """phi角 → MuJoCo raw joint角"""
    if side == "Right":
        return math.pi - phi1,  phi4          # phi1 = -q + π
    else:
        return phi1 - math.pi, -phi4          # phi1 =  q + π

def q_to_phi(side, q_hip, q_sho):
    """MuJoCo raw joint角 → phi角"""
    if side == "Right":
        return -q_hip + math.pi,  q_sho
    else:
        return  q_hip + math.pi, -q_sho

def phi_dot(side, qd_hip, qd_sho):
    """raw 角速度 → phi角速度"""
    if side == "Right":
        return -qd_hip,  qd_sho
    else:
        return  qd_hip, -qd_sho

# ======================== 正运动学 (FK) ========================

def fk(phi1, phi4):
    """
    五杆机构正运动学 → (L, theta, phi2, phi3, phi5)
      L     等效腿长 [m]
      theta 等效摆角 (相对垂线) [rad]
      phi2  从动连杆角 (前) [rad]
      phi3  从动连杆角 (后) [rad]
      phi5  膝点相对髋中点角 [rad]
    """
    l1, l2, l3, l4, l5 = L_ACT, L_SLV, L_SLV, L_ACT, L5

    xD = l5 + l4 * math.cos(phi4)
    yD = l4 * math.sin(phi4)
    xB = l1 * math.cos(phi1)
    yB = l1 * math.sin(phi1)

    BD  = math.sqrt((xD - xB)**2 + (yD - yB)**2)
    A0  = 2 * l2 * (xD - xB)
    B0  = 2 * l2 * (yD - yB)
    C0  = l2**2 + BD**2 - l3**2
    disc = max(0.0, A0**2 + B0**2 - C0**2)
    phi2 = 2 * math.atan2(B0 + math.sqrt(disc), A0 + C0)

    xC = l1 * math.cos(phi1) + l2 * math.cos(phi2)
    yC = l1 * math.sin(phi1) + l2 * math.sin(phi2)
    phi3 = math.atan2(yC - yD, xC - xD)
    phi5 = math.atan2(yC, xC - l5 / 2)

    L     = math.sqrt((xC - l5 / 2)**2 + yC**2)
    theta = phi5 - math.pi / 2

    return L, theta, phi2, phi3, phi5


def fk_deriv(phi1, phi4, phi1_d, phi4_d, pitch=0.0, pitch_d=0.0, dt=0.0001):
    """
    FK + 预测步数值微分 → (L, theta, theta_dot, L_dot, phi2, phi3, phi5, phi2_dot)
    """
    L, alpha, phi2, phi3, phi5 = fk(phi1, phi4)
    theta = alpha - pitch

    phi1p = phi1 + phi1_d * dt
    phi4p = phi4 + phi4_d * dt
    Lp, alpha_p, phi2_p, _, _ = fk(phi1p, phi4p)
    theta_p = alpha_p - (pitch + pitch_d * dt)

    L_d     = (Lp - L) / dt
    theta_d = (theta_p - theta) / dt
    phi2_d  = (phi2_p - phi2) / dt

    return L, theta, theta_d, L_d, phi2, phi3, phi5, phi2_d


# ======================== 逆运动学 (IK) ========================

def ik(L_ref):
    """
    腿长 → phi1, phi4   (约束 phi1 + phi4 = π)
    二分搜索 phi1 ∈ [90°, 180°]
    """
    lo = math.radians(90)
    hi = math.radians(180)
    for _ in range(30):
        mid   = (lo + hi) / 2
        L_mid = fk(mid, math.pi - mid)[0]
        if L_mid < L_ref:
            hi = mid
        else:
            lo = mid
    phi1 = (lo + hi) / 2
    return phi1, math.pi - phi1


# ======================== VMC 雅可比 ========================

def vmc_jacobian(phi1, phi2, phi3, phi4, phi5, L):
    """
    Jᵀ: [F_Leg, T_Leg]ᵀ → [T_rear, T_front]ᵀ  (leg_fun.m 公式)
    """
    d32 = math.sin(phi3 - phi2)
    if abs(d32) < 1e-6:
        d32 = math.copysign(1e-6, d32)
    return np.array([
        [L_ACT * math.sin(phi5 - phi3) * math.sin(phi1 - phi2) / d32,
         L_ACT * math.cos(phi5 - phi3) * math.sin(phi1 - phi2) / L / d32],
        [L_ACT * math.sin(phi5 - phi2) * math.sin(phi3 - phi4) / d32,
         L_ACT * math.cos(phi5 - phi2) * math.sin(phi3 - phi4) / L / d32],
    ])


# ======================== 初始化 MuJoCo (供调用方传入) ========================

def init_model(xml_path):
    """加载 XML 并返回 model, data, 关节地址表, 执行器 ID"""
    model = mj.MjModel.from_xml_path(xml_path)
    data  = mj.MjData(model)

    def _id(obj, name):
        return mj.mj_name2id(model, obj, name)

    act_ids = np.array([_id(mj.mjtObj.mjOBJ_ACTUATOR, n) for n in ACT_NAMES])
    act_idx = {
        ("Right", "rear"): 1, ("Right", "front"): 0,
        ("Left",  "rear"): 4, ("Left",  "front"): 3,
    }

    joint_addr = {}
    for side in ["Right", "Left"]:
        joint_addr[side] = {}
        for k in ["front", "rear", "wheel"]:
            jid = _id(mj.mjtObj.mjOBJ_JOINT, f"{side}_{k}_joint")
            joint_addr[side][k] = {
                "qpos": model.jnt_qposadr[jid],
                "dof":  model.jnt_dofadr[jid],
            }

    # freejoint qpos 起始地址 (用于读 pitch)
    freejoint_id = _id(mj.mjtObj.mjOBJ_JOINT, "base_freejoint")
    freejoint_adr = model.jnt_qposadr[freejoint_id]

    # 轮子 body ID (用于离地检测)
    wheel_body = {
        "Right": _id(mj.mjtObj.mjOBJ_BODY, "Right_Wheel_link"),
        "Left":  _id(mj.mjtObj.mjOBJ_BODY, "Left_Wheel_link"),
    }

    return model, data, act_ids, act_idx, joint_addr, freejoint_adr, wheel_body


# ======================== K 矩阵 (变腿长 LQR) ========================
# usebodyvel.m 以等效腿长 L ∈ [0.20, 0.39]m 离散化 → 二次多项式拟合
# 每行 [p1, p2, p3]: K_i(L) = p1*L² + p2*L + p3
# 行 1~6  → 轮子力矩 T_wheel, 行 7~12 → 虚拟腿力矩 T_Leg

K_CONS = np.array([
    [ 45.778283,  -45.778071,   -9.396343],   # K1  theta     → T_wheel
    [  1.756041,   -2.338764,   -0.550447],   # K2  theta_dot → T_wheel
    [ 10.523162,   -8.954049,   -2.780402],   # K3  x         → T_wheel
    [ 10.678620,   -9.305726,   -3.198268],   # K4  x_dot     → T_wheel
    [ 69.592610,  -70.679983,   23.123773],   # K5  pitch     → T_wheel
    [  5.831394,   -6.022073,    2.078401],   # K6  pitch_dot → T_wheel
    [ 31.186652,  -37.751178,   14.599865],   # K7  theta     → T_Leg
    [  3.619870,   -3.816698,    1.346905],   # K8  theta_dot → T_Leg
    [ 20.883169,  -22.162118,    7.687936],   # K9  x         → T_Leg
    [ 23.518776,  -24.661049,    8.447341],   # K10 x_dot     → T_Leg
    [-80.560280,   67.814941,   38.985001],   # K11 pitch     → T_Leg
    [ -5.929814,    5.034073,    2.692640],   # K12 pitch_dot → T_Leg
])


def get_kk(L):
    """等效腿长 L → 12 维 K 向量: kk = K_CONS @ [L², L, 1]ᵀ"""
    ll = np.array([L**2, L, 1.0])
    return K_CONS @ ll


def get_k_mat(L, grounded=True):
    """
    2×6 K 矩阵:  [T_wheel, T_Leg]ᵀ = K_mat @ X_err
    X_err = [theta, theta_dot, x, x_dot, pitch, pitch_dot]ᵀ (ref - actual)
    """
    kk = get_kk(L)
    if grounded:
        return np.array([
            [kk[0], kk[1], kk[2], kk[3], kk[4], kk[5]],
            [kk[6], kk[7], kk[8], kk[9], kk[10], kk[11]],
        ])
    else:  # 离地 → 仅腿长方向
        return np.array([
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [kk[6], kk[7], 0.0, 0.0, 0.0, 0.0],
        ])


# ======================== IMU 读取 (从 freejoint 四元数) ========================

def get_pitch(data, freejoint_adr):
    """从 freejoint 四元数提取 pitch [rad]"""
    a = freejoint_adr
    w, x, y, z = data.qpos[a+3 : a+7]
    sp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    return math.asin(sp)

def get_pitch_dot(data, freejoint_adr):
    """从 freejoint 角速度读取 pitch 角速度 [rad/s]"""
    a = freejoint_adr
    return data.qvel[a+3 : a+6][1]  # y 轴 = pitch rate

def get_acc(data, freejoint_adr):
    """从 freejoint 加速度读取机体加速度 [m/s²]"""
    a = freejoint_adr
    return data.qacc[a : a+3]  # ax, ay, az
