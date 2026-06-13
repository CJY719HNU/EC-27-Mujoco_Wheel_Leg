import mujoco as mj
import numpy as np
import time
import mujoco.viewer
import math
import turtle
model = mujoco.MjModel.from_xml_path("1.xml")
data = mujoco.MjData(model) #读取模型的各项数据如速度、位置
K = np.array([ -20.62, -30.89, -100.26,  -15.86]) #增益矩阵：
sensor_names = ["cart_pos", "cart_vel", "pole_angle", "pole_angvel"]
sensor_ids = [
    mj.mj_name2id(model, mj.mjtObj.mjOBJ_SENSOR, name)
    for name in sensor_names
]
#将mjcf中定义的传感器与id对应起来
data.qpos[1] = math.radians(2)  # 给杆初始的2° 偏角
mj.mj_forward(model, data)  #推进仿真一步
print(sensor_names)
print(sensor_ids)
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        x = np.array([data.sensordata[i] for i in sensor_ids]) # 读取传感器采集的四个状态变量
        u = -K @ x #输入负反馈
        data.ctrl[0] = u  #控制电机输入
        mujoco.mj_step(model, data) #推进仿真，步长为mjcf中的设置
        viewer.sync() #更新屏幕图像
        time.sleep(0.001) #设置仿真显示的速度