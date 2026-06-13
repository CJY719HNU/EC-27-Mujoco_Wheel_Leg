function [T,T1,T2,Height,FN,body_v,theta,theta_dot,LegLength]= Left_Leg(x,x_dot,pitch,pitch_dot,Joint1_theta,Joint1_thetadot,Joint2_theta,Joint2_thetadot,LegLength_ref,x_ref,x_dot_ref,accx,accy,WheelVel,theta_error,theta_dot_error)
    
phi1 = -Joint1_theta + pi/2;
phi1_dot = -Joint1_thetadot;
phi4 = pi/2 -Joint2_theta;
phi4_dot = -Joint2_thetadot;

%% 全局变量
% global theta_dot_llast;
% global LegLength_dot_llast;
global FN_l;
global l_active_leg;
global l_slave_leg;
global joint_distance;
global mw;
global g;
global K_cons;
global R;
global M;

%% 结构参数
l1 = l_active_leg;
l4 = l_active_leg;
l2 = l_slave_leg;
l3 = l_slave_leg;
l5 = joint_distance;
 

%% 正运动学解算
xD = l5 + l4*cos(phi4);
yD = l4*sin(phi4);
xB = 0 + l1*cos(phi1);
yB = l1*sin(phi1);

BD = sqrt((xD-xB)^2+(yD-yB)^2); 
A0 = 2*l2*(xD-xB);
B0 = 2*l2*(yD-yB);
C0 = l2^2 + BD^2 - l3^2;
phi2 = 2*atan2(B0 + sqrt(A0^2+B0^2-C0^2),A0+C0);
xC = l1*cos(phi1) + l2*cos(phi2);
yC = l1*sin(phi1) + l2*sin(phi2);
phi3 = atan2(yC-yD,xC-xD);   % 稍后用于计算VMC

% theta and LegLength solve
phi5 = atan2(yC, xC-l5/2);
alpha = phi5 - pi/2;
theta = alpha - pitch;
LegLength = sqrt((xC-l5/2)^2+yC^2);
Height = LegLength*cos(theta);

%% 预测下一时刻
dt = 0.0001;
phi1_pre = phi1 + phi1_dot*dt;   
phi4_pre = phi4 + phi4_dot*dt;

% 重新计算腿长和角度
xD = l5 + l4*cos(phi4_pre);
yD = l4*sin(phi4_pre);
xB = 0 + l1*cos(phi1_pre);
yB = l1*sin(phi1_pre);

BD = sqrt((xD-xB)^2+(yD-yB)^2); 
A0 = 2*l2*(xD-xB);
B0 = 2*l2*(yD-yB);
C0 = l2^2 + BD^2 - l3^2;
phi2_pre = 2*atan2(B0 + sqrt(A0^2+B0^2-C0^2),A0+C0);
xC = l1*cos(phi1_pre) + l2*cos(phi2_pre);
yC = l1*sin(phi1_pre) + l2*sin(phi2_pre);

% theta_dot and LegLength solve
phi5_pre = atan2(yC, xC-l5/2);
phi2_dot = (phi2_pre-phi2)/dt;
theta_dot = (phi5_pre - pi/2 - (pitch+pitch_dot*dt) - theta)/dt;
LegLength_dot = (sqrt((xC-l5/2)^2+yC^2) - LegLength)/dt;
Height_dot = LegLength_dot*cos(theta) - LegLength*sin(theta)*theta_dot;


%% LQR_cal
% K多项式拟合
ll = [LegLength^2; LegLength; 1];
kk = K_cons*ll;

% 离地检测
if FN_l<20
    K_mat = [0 0 0 0 0 0;
            kk(7) kk(8) 0 0 0 0];
else
    K_mat = [kk(1) kk(2) kk(3) kk(4) kk(5) kk(6);
            kk(7) kk(8) kk(9) kk(10) kk(11) kk(12)];
end

% T T_leg solve
X_ref = [0 0 x_ref x_dot_ref 0 0]';
X = [theta theta_dot x x_dot pitch pitch_dot]';

X_err = X_ref - X;
u = K_mat*X_err;

T = -u(1);
T_Leg = u(2);


%% LegLength Control
Kp = 1200;
Kd = 300;
F_Leg = 0.5*M*g*cos(theta)+Kp*(LegLength_ref-Height) + Kd*(0-Height_dot);
% F_Leg = 0.5*M*g*cos(theta)+Kp*(LegLength_ref-LegLength) + Kd*(0-LegLength_dot);

%%抗劈叉控制
Kp_theta = 6;
Kd_theta = 0.2;

T_Leg = T_Leg+(Kp_theta*(theta_error)+Kd_theta*(theta_dot_error));


%% T1 and T2 solve
F = [F_Leg; T_Leg];
Trans_Jacobian = [l1*sin(phi5-phi3)*sin(phi1-phi2)/sin(phi3-phi2)   l1*cos(phi5-phi3)*sin(phi1-phi2)/LegLength/sin(phi3-phi2);
                  l4*sin(phi5-phi2)*sin(phi3-phi4)/sin(phi3-phi2)   l4*cos(phi5-phi2)*sin(phi3-phi4)/LegLength/sin(phi3-phi2)];
Tj = Trans_Jacobian*F;

T1 = -Tj(1);
T2 = -Tj(2);


%% 驱动轮支持力解算
% delta = 0.01;
% theta_ddot = (theta_dot-theta_dot_llast)/delta;
% leglen_ddot = (LegLength_dot-LegLength_dot_llast)/delta;
% theta_dot_llast = theta_dot;
% LegLength_dot_llast = LegLength_dot;

yM_ddot = accx*sin(pitch) + accy*cos(pitch);
% yw_ddot = yM_ddot - leglen_ddot*cos(theta) + 2*LegLength_dot*theta_dot*sin(theta) + LegLength*theta_ddot*sin(theta) + LegLength*theta_dot^2*cos(theta);

P = F_Leg*cos(theta) + T_Leg*sin(theta)/LegLength;
FN_l = mw*yM_ddot + P + mw*g;
FN = FN_l;

%% 轮速修正
wheel_w = -WheelVel + phi2_dot - pitch_dot;

% 计算髋关节处速度
body_v = wheel_w*R + LegLength*theta_dot + LegLength_dot*sin(theta);


end