"""
LQR 平衡控制器 —— 轮腿机器人

═══════════════════════════════════════════════════════════════════
                         数据流一览
═══════════════════════════════════════════════════════════════════

  MuJoCo Model ─┬─ mj_forward → qM (质量矩阵, 用于数值线性化)
                ├─ mj_forward → qfrc_bias (偏置力)
                ├─ mj_forward → subtree_com, xpos (几何参数辨识)
                └─ mj_forward → qacc (有限差分响应)

  IMU sensor ────┬─ data.sensor('orientation') → euler[1] = body pitch
                 └─ data.sensor('gyro')        → gyro[1]   = pitch rate

  VMC 正向运动学 ─┬─ vmc_r.theta / vmc_l.theta    ← 等效摆角 (用于LQR)
                 ├─ vmc_r.d_theta / vmc_l.d_theta ← 等效角速度
                 ├─ vmc_r.L0 / vmc_l.L0           ← 虚拟腿长 (高度PID)
                 └─ vmc_r.d_L0 / vmc_l.d_L0       ← 虚拟腿速

  VMC Jacobian ───┬─ j11, j12, j21, j22 (映射 F0,Tp → 关节力矩)
                  ├─ torque_set[1] = j11*F0 + j12*Tp → ctrl[0] (jAB 右前)
                  └─ torque_set[0] = j21*F0 + j22*Tp → ctrl[1] (jAG 右后)

  LQR 状态 (4D) ── [θ, θ̇, x, ẋ] = [theta_avg, dtheta_avg, x, d_x]

  LQR 控制 (2D) ── [wheel_torque, Tp]
                    wheel_torque → data.ctrl[4], ctrl[5] (两轮对称)
                    Tp           → vmc_r.Tp, vmc_l.Tp

  PID 高度 ─────── F0 = PID(L0_desired - L0_avg) → vmc_r.F0, vmc_l.F0

═══════════════════════════════════════════════════════════════════
"""

import numpy as np
from scipy.linalg import solve_continuous_are
import mujoco
import math
from Controller import PID


# ═══════════════════════════════════════════════════════════════════
# 数值线性化 —— 从 MuJoCo 模型直接差分得到 A, B
# ═══════════════════════════════════════════════════════════════════

def _quat_rotate_y(quat, angle):
    """绕世界 y 轴旋转四元数 angle 弧度."""
    half = angle / 2.0
    q_rot = np.array([math.cos(half), 0.0, math.sin(half), 0.0])
    out = np.zeros(4)
    mujoco.mju_mulQuat(out, q_rot, quat)
    return out


def linearize_mujoco(model, data, eps=1e-4):
    """
    数值线性化 MuJoCo 模型, 提取四阶降维系统 A(4x4), B(4x2).

    状态: [pitch, pitch_rate, x, dx]
    控制: [wheel_torque, Tp_nominal]

    方法:
      - 在模型当前位姿附近做有限差分
      - pitch 扰动: 绕世界 y 轴旋转基座四元数
      - pitch_rate 扰动: 修改 free joint 的角速度 y 分量
      - dx 扰动: 修改 free joint 的线速度 x 分量
      - ctrl 扰动: 直接修改 data.ctrl

    返回 (A, B)
    """
    nv = model.nv
    nu = model.nu

    # ── 保存当前状态 ──
    qpos0 = data.qpos.copy()
    qvel0 = data.qvel.copy()
    ctrl0 = data.ctrl.copy()

    # ── 标称加速度 ──
    mujoco.mj_forward(model, data)
    # free joint: qacc[0]=ax, qacc[5]=angular_acc_y (pitch accel)
    pitch_acc0 = data.qacc[5]
    fwd_acc0   = data.qacc[0]

    #
    #  A 矩阵 (4x4)
    #
    A = np.zeros((4, 4))
    A[0, 1] = 1.0   # dθ/dt = θ̇
    A[2, 3] = 1.0   # dx/dt = ẋ

    # --- ∂(θ̈, ẍ) / ∂θ : 扰动 pitch 角度 ---
    data.qpos[:] = qpos0
    data.qpos[3:7] = _quat_rotate_y(qpos0[3:7], eps)
    data.qvel[:] = qvel0
    data.ctrl[:] = ctrl0
    mujoco.mj_forward(model, data)
    A[1, 0] = (data.qacc[5] - pitch_acc0) / eps   # ∂θ̈/∂θ
    A[3, 0] = (data.qacc[0] - fwd_acc0) / eps     # ∂ẍ/∂θ

    # --- ∂(θ̈, ẍ) / ∂θ̇ : 扰动 pitch 角速度 ---
    data.qpos[:] = qpos0
    data.qvel[:] = qvel0
    data.qvel[5] += eps  # y-axis angular velocity
    data.ctrl[:] = ctrl0
    mujoco.mj_forward(model, data)
    A[1, 1] = (data.qacc[5] - pitch_acc0) / eps   # ∂θ̈/∂θ̇
    A[3, 1] = (data.qacc[0] - fwd_acc0) / eps     # ∂ẍ/∂θ̇

    # --- ∂(θ̈, ẍ) / ∂ẋ : 扰动前进速度 ---
    data.qpos[:] = qpos0
    data.qvel[:] = qvel0
    data.qvel[0] += eps  # x-axis linear velocity
    data.ctrl[:] = ctrl0
    mujoco.mj_forward(model, data)
    A[1, 3] = (data.qacc[5] - pitch_acc0) / eps   # ∂θ̈/∂ẋ
    A[3, 3] = (data.qacc[0] - fwd_acc0) / eps     # ∂ẍ/∂ẋ

    #
    #  B 矩阵 (4x2): 控制 = [wheel_torque, Tp]
    #
    B = np.zeros((4, 2))
    data.qpos[:] = qpos0
    data.qvel[:] = qvel0

    # --- ∂(θ̈, ẍ) / ∂(wheel_torque) : ctrl[4], ctrl[5] ---
    data.ctrl[:] = ctrl0
    data.ctrl[4] += eps
    data.ctrl[5] += eps
    mujoco.mj_forward(model, data)
    B[1, 0] = (data.qacc[5] - pitch_acc0) / eps
    B[3, 0] = (data.qacc[0] - fwd_acc0) / eps

    # --- ∂(θ̈, ẍ) / ∂(Tp) : 通过 ctrl[0:4] 近似 pitch torque ---
    # Tp 实际通过 VMC Jacobian 映射到关节力矩, 这里直接用关节 ctrl 近似
    #   右前 jAB=ctrl[0] ← j22*Tp,  右后 jAG=ctrl[1] ← j12*Tp
    #   左前 jIJ=ctrl[2] ← j22*Tp,  左后 jIO=ctrl[3] ← j12*Tp
    # 四关节同时 +eps 近似 Tp 方向的广义力
    data.ctrl[:] = ctrl0
    data.ctrl[0] += eps  # jAB
    data.ctrl[1] += eps  # jAG
    data.ctrl[2] += eps  # jIJ
    data.ctrl[3] += eps  # jIO
    mujoco.mj_forward(model, data)
    B[1, 1] = (data.qacc[5] - pitch_acc0) / eps
    B[3, 1] = (data.qacc[0] - fwd_acc0) / eps

    # ── 恢复原状态 ──
    data.qpos[:] = qpos0
    data.qvel[:] = qvel0
    data.ctrl[:] = ctrl0
    mujoco.mj_forward(model, data)

    return A, B


# ═══════════════════════════════════════════════════════════════════
# 解析模型 —— 从 MuJoCo 质量属性推导 2D 倒立摆
# ═══════════════════════════════════════════════════════════════════

def identify_params_from_model(model, data):
    """
    从 MuJoCo 模型提取物理参数, 构建解析 2D 轮式倒立摆的 A, B.

    返回: (A, B, param_dict)
    """
    # ── 质量 ──
    total_mass = sum(model.body_mass[i] for i in range(model.nbody))

    wheel_r_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'wheel_right')
    wheel_l_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'wheel_left')
    wheel_mass = model.body_mass[wheel_r_id] + model.body_mass[wheel_l_id]
    body_mass = total_mass - wheel_mass   # 车身+腿 (pendulum)

    # ── CoM 到轮轴距离 l ──
    mujoco.mj_forward(model, data)

    # 全机质心 (root subtree)
    com = data.subtree_com[0]
    # 左右轮轴中点
    wheel_r_pos = data.xpos[wheel_r_id]
    wheel_l_pos = data.xpos[wheel_l_id]
    wheel_axis = (wheel_r_pos + wheel_l_pos) / 2.0

    dx = com[0] - wheel_axis[0]
    dz = com[2] - wheel_axis[2]
    l_com = math.sqrt(dx**2 + dz**2)
    if l_com < 0.02:
        l_com = 0.25

    # ── Pitch 转动惯量 ──
    # 从质量矩阵提取 free joint 转动部分
    M_full = np.zeros((model.nv, model.nv))
    mujoco.mj_fullM(model, M_full, data.qM)
    I_from_M = M_full[5, 5]  # y 轴 pitch 分量

    # 点质量摆近似: I_point = body_mass * l_com^2
    I_point = body_mass * l_com**2

    # 使用两值中较合理者 (M_full 可能含无物理意义的超大值)
    I_total = I_from_M if 0.01 < I_from_M < 10 * I_point else I_point
    if I_total < 0.01:
        I_total = max(I_point, 0.05)

    # ── 轮半径 ──
    # 从轮轴离地高度估算 (轮轴 z = 轮半径, 如果轮触地)
    r_wheel_raw = abs(wheel_axis[2])
    r_wheel = r_wheel_raw if 0.005 < r_wheel_raw < 0.3 else 0.04

    # ── 构建 A, B ──
    M_t = total_mass
    m_b = body_mass
    l_c = l_com
    I_t = I_total
    r_w = r_wheel
    g = 9.81

    # 如果参数明显有问题, 回退到合理默认值
    if l_c < 0.05:
        l_c = 0.20        # 默认 CoM 高度 ~20cm
    if I_t < 0.05:
        I_t = m_b * l_c**2

    print(f'      [identify] M_total={M_t:.2f}  M_body={m_b:.2f}  '
          f'l_com={l_c:.4f}  I_from_M={I_from_M:.4f}  I_used={I_t:.4f}  '
          f'r_wheel={r_w:.4f}')

    # 耦合动力学:
    #   [M_t,    m_b*l ] [ẍ]   = [F_wheel           ]
    #   [m_b*l,  I_t   ] [θ̈]     [m_b*g*l*θ + T_pitch]
    #
    #  D = M_t * I_t - (m_b*l_c)^2

    D = M_t * I_t - (m_b * l_c)**2
    if abs(D) < 1e-8:
        D = 1e-8

    # θ̈ 方程系数
    a21 = M_t * m_b * g * l_c / D          # θ̈ / θ
    b21 = -m_b * l_c / D                    # θ̈ / F_wheel
    b22 = M_t / D                           # θ̈ / T_pitch

    # ẍ 方程系数
    a41 = -(m_b * l_c)**2 * g / D           # ẍ / θ
    b41 = I_t / D                           # ẍ / F_wheel
    b42 = -m_b * l_c / D                    # ẍ / T_pitch

    # 状态: [θ, θ̇, x, ẋ],  控制: [wheel_torque, T_pitch]
    # F_wheel = wheel_torque / r_wheel
    A = np.array([
        [0.0,   0.0, 0.0, 0.0],
        [a21,   0.0, 0.0, 0.0],
        [0.0,   0.0, 0.0, 1.0],
        [a41,   0.0, 0.0, 0.0],
    ])

    B = np.array([
        [0.0,         0.0],
        [b21 / r_w,   b22],
        [0.0,         0.0],
        [b41 / r_w,   b42],
    ])

    params = {
        'M_total': M_t, 'M_body': m_b, 'M_wheel': wheel_mass,
        'l_com': l_c, 'I_total': I_t, 'r_wheel': r_w,
        'D': D,
    }
    return A, B, params


# ═══════════════════════════════════════════════════════════════════
# LQR 控制器主类
# ═══════════════════════════════════════════════════════════════════

class LQRController:
    """
    LQR 平衡控制器.

    状态 (4D):  [θ (pitch rad),  θ̇ (rad/s),  x (m),  ẋ (m/s)]
    控制 (2D):  [wheel_torque (N·m),  Tp (N·m, VMC 切向力矩)]
    辅助 PID:   F0 (N, VMC 径向力, 控制腿长/高度)
    """

    def __init__(self, model, data,
                 method='analytical',  # 'analytical'(推荐) | 'numerical'
                 Q_diag=None,
                 R_diag=None,
                 flip_Tp=False,        # True = 翻转 Tp 符号 (快速极性调试)
                 flip_F0=False,        # True = 翻转 F0 符号
                 verbose=True):
        """
        Args:
            model, data: MuJoCo 模型和数据
            method:      'analytical' (推荐) — 从质量属性解析推导
                         'numerical' — MuJoCo 有限差分 (⚠ 可能因 VMC Jacobian
                                       交叉符号导致极性错误, 配合 diagnose_vmc_polarity 使用)
            Q_diag:  [θ_weight, θ̇_weight, x_weight, ẋ_weight]
            R_diag:  [wheel_torque_weight, Tp_weight]
            flip_Tp: 翻转 Tp 输出符号
            flip_F0: 翻转 F0 输出符号
            verbose: 打印诊断信息
        """
        self.model = model
        self.data = data
        self.method = method
        self.flip_Tp = flip_Tp
        self.flip_F0 = flip_F0
        self.verbose = verbose

        # ── 1. 辨识 A, B ──
        if method == 'numerical':
            if verbose:
                print('[LQR] 数值线性化 (MuJoCo finite-difference)...')
                print('      ⚠ 数值方法对所有关节施加同向扰动, 可能与 VMC Jacobian')
                print('        的交叉符号不匹配, 导致 Tp 极性错误 (劈叉).')
                print('        建议先跑 diagnose_vmc_polarity() 确认符号.')
            self.A, self.B = linearize_mujoco(model, data)
            self._info = {}
        else:
            if verbose:
                print('[LQR] 解析模型 (MuJoCo 质量属性)...')
            self.A, self.B, self._info = identify_params_from_model(model, data)

        # ── 2. LQR 权重 ──
        if Q_diag is None:
            Q_diag = [500.0,  # θ  (pitch angle)
                       10.0,  # θ̇ (pitch rate)
                       10.0,  # x  (forward position)
                       50.0]  # ẋ  (forward velocity)
        if R_diag is None:
            R_diag = [0.5,  # wheel_torque
                       1.0]  # Tp

        self.Q = np.diag(Q_diag)
        self.R = np.diag(R_diag)

        # ── 3. 求解 Riccati ──
        self.K = self._solve_lqr()

        # ── 4. 诊断 ──
        if verbose:
            eig_open = np.linalg.eigvals(self.A)
            eig_closed = np.linalg.eigvals(self.A - self.B @ self.K)
            print(f'      Q={Q_diag}')
            print(f'      R={R_diag}')
            print(f'      flip_Tp={flip_Tp}  flip_F0={flip_F0}')
            print(f'      Open-loop  poles: {np.array2string(eig_open, precision=2)}')
            print(f'      Closed-loop poles: {np.array2string(eig_closed, precision=2)}')
            print(f'      Gain K =\n{np.array2string(self.K, precision=3, suppress_small=True)}')
            if self._info:
                print(f'      M_total={self._info["M_total"]:.1f}kg  '
                      f'l_com={self._info["l_com"]:.3f}m  '
                      f'I_total={self._info["I_total"]:.3f}  '
                      f'r_wheel={self._info["r_wheel"]:.3f}m')

        # ── 5. 高度 PID ──
        self.height_pid = PID(p=80.0, i=0.0, d=5.0)
        self.L0_desired = None   # 首次运行时从 VMC 记录

    def _solve_lqr(self):
        """求解 Riccati, 失败时回退到手动 PD 增益."""
        try:
            P = solve_continuous_are(self.A, self.B, self.Q, self.R)
            K = np.linalg.solve(self.R, self.B.T @ P)
            return K
        except np.linalg.LinAlgError as e:
            # Riccati 无解 → 系统参数有误, 回退到解析 PD 增益
            if self.verbose:
                print(f'      ⚠ Riccati 无解 ({e}), 回退到手动 PD 增益')
                print(f'        A =\n{np.array2string(self.A, precision=4, suppress_small=True)}')
                print(f'        B =\n{np.array2string(self.B, precision=4, suppress_small=True)}')

            # 从 A 矩阵提取不稳定极点: 系统是 θ̈ = a21*θ + b22*Tp
            a21 = self.A[1, 0]          # θ̈/θ = 不稳定刚度
            b22 = self.B[1, 1]          # θ̈/Tp = 控制有效性
            b_wheel = self.B[3, 0]      # ẍ/τ_wheel

            if abs(a21) < 1e-4:
                a21 = 15.0              # 默认不稳定极点 ~3.9 rad/s
            if abs(b22) < 1e-4:
                b22 = 1.0
            if abs(b_wheel) < 1e-4:
                b_wheel = 1.0

            # 期望闭环极点: s² + 2ζωₙ s + ωₙ²
            wn = math.sqrt(a21) * 2.5   # 闭环频率 > 开环不稳定频率
            zeta = 0.8

            # PD 增益 (2D pitch 子系统)
            k_theta = (wn**2 + a21) / b22      # 抵消开环不稳定 + 加入闭环刚度
            k_dtheta = (2 * zeta * wn) / b22    # 阻尼
            k_x = 0.0                           # 位置反馈通过轮子
            k_dx = math.sqrt(self.Q[3, 3] / (self.R[0, 0] + 1e-6)) / (b_wheel + 1e-6)
            k_dx = np.clip(k_dx, 0.5, 20.0)

            K = np.array([
                [0.0,      0.0,       0.0, k_dx],   # wheel_torque
                [k_theta,  k_dtheta,  k_x, 0.0],    # Tp
            ])

            if self.verbose:
                print(f'        Fallback K (wn={wn:.1f}, ζ={zeta:.1f}):')
                print(f'        K =\n{np.array2string(K, precision=3, suppress_small=True)}')
            return K

    # ── 公开接口 ──

    def control(self, pitch, pitch_rate, forward_vel, forward_pos=0.0):
        """
        LQR 平衡控制.

        Args (来自 VMC + 里程计):
            pitch:       theta 均值 (rad), vmc_r.theta / vmc_l.theta
            pitch_rate:  d_theta 均值 (rad/s)
            forward_vel: d_x (m/s), 来自轮速里程计
            forward_pos: x (m)

        Returns:
            (wheel_torque, Tp):
                wheel_torque — 轮力矩 (N·m), 对称施加到两轮
                Tp           — VMC 切向力矩 (N·m), 填入 vmc_r.Tp 和 vmc_l.Tp
        """
        x = np.array([pitch, pitch_rate, forward_pos, forward_vel])
        u = -self.K @ x
        wheel_torque = float(u[0])
        Tp = float(u[1])
        if self.flip_Tp:
            Tp = -Tp
        return wheel_torque, Tp

    def control_height(self, L0_measured, L0_desired=None):
        """
        高度 PID: 输出 VMC 径向力 F0.

        Args:
            L0_measured: 当前虚拟腿长均值 (vmc_r.L0 + vmc_l.L0)/2
            L0_desired:  期望腿长, None 则使用记录的标称值

        Returns:
            F0 (N) — 填入 vmc_r.F0 和 vmc_l.F0
        """
        if L0_desired is None:
            if self.L0_desired is None:
                self.L0_desired = L0_measured  # 首次记录
            L0_desired = self.L0_desired
        F0 = self.height_pid.calc(L0_measured, L0_desired)
        if self.flip_F0:
            F0 = -F0
        return F0

    def reset(self):
        """重置高度 PID 积分."""
        self.height_pid.integral = 0.0
        self.height_pid.prev_error = 0.0
        self.L0_desired = None

    # ── 极性诊断 ──

    def diagnose_vmc_polarity(self, vmc_r, vmc_l):
        """
        诊断 VMC Jacobian 极性 —— 打印 Tp=1 和 F0=1 时各关节的力方向.

        调用时机: 在 Simulation.py 的 t2 周期内, VMC 已计算后调用一次即可.

        输出示例:
            --- VMC Polarity Diagnosis ---
            Tp=+1 → joints: [ +0.34  -0.12  +0.12  -0.34 ]
                     r_front  r_rear  l_front  l_rear
            F0=+1 → joints: [ +0.45  +0.56  +0.45  +0.56 ]
            Pitch torque from Tp=+1:  RIGHT: +0.34(f)+-0.12(r)  LEFT: +0.12(f)+-0.34(r)
            ⚠ 若左右符号相反 → Tp 会导致劈叉! 尝试 flip_Tp=True 或 flip 单侧 Tp.
        """
        # 临时保存
        F0_save_r, Tp_save_r = vmc_r.F0, vmc_r.Tp
        F0_save_l, Tp_save_l = vmc_l.F0, vmc_l.Tp

        # Tp 响应
        vmc_r.F0, vmc_r.Tp = 0, 1
        vmc_l.F0, vmc_l.Tp = 0, 1
        vmc_r.vmc_calc_torque()
        vmc_l.vmc_calc_torque()
        t_r = vmc_r.torque_set.copy()
        t_l = vmc_l.torque_set.copy()

        # F0 响应
        vmc_r.F0, vmc_r.Tp = 1, 0
        vmc_l.F0, vmc_l.Tp = 1, 0
        vmc_r.vmc_calc_torque()
        vmc_l.vmc_calc_torque()
        f_r = vmc_r.torque_set.copy()
        f_l = vmc_l.torque_set.copy()

        # 恢复
        vmc_r.F0, vmc_r.Tp = F0_save_r, Tp_save_r
        vmc_l.F0, vmc_l.Tp = F0_save_l, Tp_save_l
        vmc_r.vmc_calc_torque()
        vmc_l.vmc_calc_torque()

        # 实际 joint 映射: joint_torque = [r.t[1], r.t[0], l.t[0], l.t[1]]
        joint_Tp = [t_r[1], t_r[0], t_l[0], t_l[1]]
        joint_F0 = [f_r[1], f_r[0], f_l[0], f_l[1]]

        print('\n' + '='*70)
        print('  VMC Jacobian 极性诊断')
        print('='*70)
        print(f'  Tp=+1 → joints [r_f r_r l_f l_r]: '
              f'[{joint_Tp[0]:+7.3f} {joint_Tp[1]:+7.3f} '
              f'{joint_Tp[2]:+7.3f} {joint_Tp[3]:+7.3f}]')
        print(f'  F0=+1 → joints [r_f r_r l_f l_r]: '
              f'[{joint_F0[0]:+7.3f} {joint_F0[1]:+7.3f} '
              f'{joint_F0[2]:+7.3f} {joint_F0[3]:+7.3f}]')

        # 符号一致性检查
        r_f_r_sign = 'same' if joint_Tp[0] * joint_Tp[1] > 0 else 'OPPO'
        l_f_r_sign = 'same' if joint_Tp[2] * joint_Tp[3] > 0 else 'OPPO'
        print(f'  Tp 右前/右后符号: {r_f_r_sign}  左前/左后符号: {l_f_r_sign}')

        r_effective = joint_Tp[0] + joint_Tp[1]  # 右腿 Tp 等效
        l_effective = joint_Tp[2] + joint_Tp[3]  # 左腿 Tp 等效
        print(f'  Tp 右腿等效: {r_effective:+.3f}  左腿等效: {l_effective:+.3f}')
        if r_effective * l_effective < 0:
            print('  ⚠⚠⚠ 左右等效符号相反! Tp 会导致劈叉!')
            print('        尝试: lqr = LQRController(..., flip_Tp=True)')
            print('        或手动: vmc_l.Tp = -Tp (只翻转左腿)')
        else:
            print('  ✓ 左右等效符号一致, Tp 极性正常')

        # F0 检查
        f0_ok = all(j > 0 for j in joint_F0) or all(j < 0 for j in joint_F0)
        if not f0_ok:
            print('  ⚠ F0 四关节符号不一致, 可能导致腿不对称伸展!')
            print('       尝试: lqr = LQRController(..., flip_F0=True)')
        print('='*70 + '\n')

        return {
            'joint_Tp': joint_Tp,
            'joint_F0': joint_F0,
            'r_effective': r_effective,
            'l_effective': l_effective,
            'Tp_polarity_ok': r_effective * l_effective > 0,
            'F0_polarity_ok': f0_ok,
        }

    # ── 调试 ──

    def debug(self, theta, dtheta, x, dx,
              wheel_t=None, Tp=None, F0=None, L0=None):
        """返回一行诊断字符串."""
        w, t = self.control(theta, dtheta, dx, x)
        w = wheel_t if wheel_t is not None else w
        t_str = f'{Tp:6.2f}' if Tp is not None else f'{t:6.2f}'
        s = (f'[LQR] θ={math.degrees(theta):+5.1f}°  '
             f'θ̇={math.degrees(dtheta):+6.1f}°/s  '
             f'v={dx:+.3f} m/s  x={x:+.3f} m  '
             f'τ_w={w:+6.2f}  Tp={t_str}')
        if L0 is not None:
            s += f'  L0={L0:.4f}'
            if F0 is not None:
                s += f'  F0={F0:+6.1f}'
        return s
