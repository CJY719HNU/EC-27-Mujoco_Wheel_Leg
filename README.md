王草凡的模型左右腿命名反了,以下统一称真正的左右腿

连杆长度质量对照：
front_link->大腿2：0.1134kg，0.073kg
front_child1_link->大腿连杆：0.135m，0.029kg
front_child2_link->大腿连杆2：0.202m，0.0486kg
front_child3_link->大腿连杆3：0.0966m，0.020187kg
rear_link->新大腿：0.21m，0.335kg
rear_child1_link->大腿连杆4：0.26m，0.251kg
wheel_link->轮子：0.077m半径，0.36kg

simulink仿真统一右手螺旋定则，故在xml控制需统一极性方向
pi-right_rear_joint = right_phi1
right_front_joint = right_phi4

right_rear_joint = left_phi1
-left_front_joint = left_phi4

left的wheel正转为逆时针
right的wheel正转为顺时针