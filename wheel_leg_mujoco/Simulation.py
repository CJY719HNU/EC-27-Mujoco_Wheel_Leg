import mujoco
import mujoco.viewer
import numpy as np
import time
from environment import *
from VMC import *
from keyboard import *
import math
from Controller import *
from LQR import LQRController

def main():
    
    TORQUE = 1  #为1时给力矩，为0是无力矩
    GBC486 = LegWheelRobot('MJCF/env.xml')
    i = 0
    t1 = 1
    t2 = 4
    t3 = 20
    vmc_r = leg_VMC()
    vmc_l = leg_VMC()
    keyboard = KeyboardController()

    # LQR 平衡控制器: 'analytical' = 质量属性解析 (推荐)
    #                 'numerical'  = MuJoCo有限差分 (可能因VMC Jacobian交叉符号劈叉)
    lqr = LQRController(GBC486.model, GBC486.data, method='analytical',
                        flip_Tp=False, flip_F0=False)
    polarity_checked = False


    while True:
        i = i + 1
        
        # 执行仿真步
        GBC486.step()  # 仿真的timestep是1ms，意味着每执行一次step仿真世界时间过去1ms
        #传感器数据获取
        if i % t1 == 0: 
            GBC486.sensor_read_data()
        # LQR + VMC 控制计算 (每 4ms)
        if i % t2 == 0:
            # ── VMC 正向运动学 ──
            vmc_r.vmc_calc_pos(
                phi1=GBC486.joint_pos[0]+math.pi,
                phi4=GBC486.joint_pos[1],
                pitch=GBC486.euler[1],
                gyro=GBC486.gyro[1],
            )
            vmc_l.vmc_calc_pos(
                phi1=GBC486.joint_pos[3]+math.pi,
                phi4=GBC486.joint_pos[2],
                pitch=-GBC486.euler[1],
                gyro=-GBC486.gyro[1],
            )

            # ── LQR 平衡控制 (4 维状态 → 2 维控制) ──
            # 状态: [θ, θ̇, x, ẋ]
            theta_avg = (vmc_r.theta + vmc_l.theta) * 0.5
            dtheta_avg = (vmc_r.d_theta + vmc_l.d_theta) * 0.5
            wheel_torque, Tp = lqr.control(
                theta_avg, dtheta_avg,
                GBC486.d_x, GBC486.x,
            )

            # ── 高度 PID → VMC 径向力 F0 ──
            L0_avg = (vmc_r.L0 + vmc_l.L0) * 0.5
            F0 = lqr.control_height(L0_avg)

            # ── 写入 VMC ──
            vmc_r.F0 = F0
            vmc_l.F0 = F0
            vmc_r.Tp = Tp
            vmc_l.Tp = Tp

            vmc_r.vmc_calc_torque()
            vmc_l.vmc_calc_torque()

            # ── 执行器赋值 ──
            GBC486.wheel_torque = [wheel_torque, wheel_torque]
            GBC486.joint_torque = [
                vmc_r.torque_set[1],  # 右前 jAB
                vmc_r.torque_set[0],  # 右后 jAG
                vmc_l.torque_set[0],  # 左前 jIJ
                vmc_l.torque_set[1],  # 左后 jIO
            ]
            GBC486.actuator_set_torque()

            # ── 首次运行: VMC Jacobian 极性诊断 ──
            if not polarity_checked:
                diag = lqr.diagnose_vmc_polarity(vmc_r, vmc_l)
                polarity_checked = True
                if not diag['Tp_polarity_ok']:
                    print('[FIX] 左右 Tp 等效符号相反! 重启时用 flip_Tp=True')
                    print('      或在 t2 块里改为: vmc_l.Tp = -Tp')

        #键盘控制指令输入,以及打印数据;运行频率低以降低仿真延迟
        if i % t3 == 0:
            cmd = keyboard.get_command()
            # print(vmc_r.L0,vmc_l.L0)



if __name__ == '__main__':
    main()