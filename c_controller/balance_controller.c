/*
 * VMC + LQR 轮腿平衡控制器 (C 版本)
 * ====================================
 * F_Leg 腿长PD + T_Leg(LQR) → VMC Jᵀ → T_rear/T_front
 * LQR → T_wheel
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "robot_config.h"

// ======================== 参数 ========================
#define XML_PATH       "COD-2026RoboMaster-Balance copy.xml"
#define LEG_LENGTH_REF 0.20
#define KP_LEG         300.0
#define KD_LEG         100.0
#define KP_THETA       6.0
#define KD_THETA       0.2
#define TORQUE_LIMIT   100.0

// ======================== 每条腿状态 ========================
typedef struct {
    double x, x0;
    double FN;
    int    grounded;
    double theta, theta_dot;
} LegData;

// ======================== MuJoCo 句柄 ========================
static mjModel *m = NULL;
static mjData  *d = NULL;
static int act_ids[6];
static int act_idx_map[4];      // R_rear,R_front,L_rear,L_front
static int jnt_qr[2], jnt_qf[2];
static int jnt_dr[2], jnt_df[2];
static int jnt_qw[2], jnt_dw[2];
static int freejoint_adr;

// ======================== 初始化 ========================
static int init_model(const char *xml_path) {
    char err[1000] = {0};
    m = mj_loadXML(xml_path, NULL, err, sizeof(err));
    if (!m) { fprintf(stderr, "ERROR: load XML '%s'\n  %s\n", xml_path, err); return -1; }
    d = mj_makeData(m);
    if (!d) { fprintf(stderr, "ERROR: mj_makeData\n"); mj_deleteModel(m); m=NULL; return -1; }

    const char *anames[6] = {
        "Right_front_joint_actuator","Right_rear_joint_actuator","Right_Wheel_joint_actuator",
        "Left_front_joint_actuator", "Left_rear_joint_actuator", "Left_Wheel_joint_actuator",
    };
    for (int i=0;i<6;i++) {
        act_ids[i] = mj_name2id(m, mjOBJ_ACTUATOR, anames[i]);
        if (act_ids[i] < 0) { fprintf(stderr, "ERROR: actuator '%s'\n", anames[i]); return -1; }
    }
    act_idx_map[0]=1; act_idx_map[1]=0; act_idx_map[2]=4; act_idx_map[3]=3;

    #define GET_J(name, q, dv) { \
        int jid=mj_name2id(m,mjOBJ_JOINT,name); \
        if (jid<0) { fprintf(stderr,"ERROR: joint '%s'\n",name); return -1; } \
        *(q)=m->jnt_qposadr[jid]; *(dv)=m->jnt_dofadr[jid]; }
    GET_J("Right_rear_joint",  &jnt_qr[0],&jnt_dr[0]);
    GET_J("Right_front_joint", &jnt_qf[0],&jnt_df[0]);
    GET_J("Right_Wheel_joint", &jnt_qw[0],&jnt_dw[0]);
    GET_J("Left_rear_joint",   &jnt_qr[1],&jnt_dr[1]);
    GET_J("Left_front_joint",  &jnt_qf[1],&jnt_df[1]);
    GET_J("Left_Wheel_joint",  &jnt_qw[1],&jnt_dw[1]);
    #undef GET_J

    int jid = mj_name2id(m, mjOBJ_JOINT, "base_freejoint");
    freejoint_adr = (jid >= 0) ? m->jnt_qposadr[jid] : -1;
    return 0;
}

// ======================== 控制回调 ========================
static LegData leg_R = {0,0,100.0,1,0,0};
static LegData leg_L = {0,0,100.0,1,0,0};

void my_controller(const mjModel *m_model, mjData *d_data) {
    (void)m_model;
    double pitch=0,pitch_d=0,acc[3]={0,0,0};
    if (freejoint_adr>=0) {
        pitch=get_pitch(d_data,freejoint_adr);
        pitch_d=get_pitch_dot(d_data,freejoint_adr);
        get_acc(d_data,freejoint_adr,acc);
    }

    double wheel_pol[2]={1.0,-1.0};
    double S[2][13];

    // ==== pass 1: FK+LQR ====
    for(int leg=0;leg<2;leg++){
        int isR=(leg==0);
        LegData *ld=isR?&leg_R:&leg_L;
        double qr=d_data->qpos[isR?jnt_qr[0]:jnt_qr[1]];
        double qdr=d_data->qvel[isR?jnt_dr[0]:jnt_dr[1]];
        double qf=d_data->qpos[isR?jnt_qf[0]:jnt_qf[1]];
        double qdf=d_data->qvel[isR?jnt_df[0]:jnt_df[1]];
        double qw=d_data->qpos[isR?jnt_qw[0]:jnt_qw[1]];
        double qdw=d_data->qvel[isR?jnt_dw[0]:jnt_dw[1]];

        double phi1,phi4,phi1_d,phi4_d;
        q_to_phi(isR,qr,qf,&phi1,&phi4);
        phi_dot(isR,qdr,qdf,&phi1_d,&phi4_d);

        double Lv,theta,theta_d,L_d,phi2,phi3,phi5;
        fk_deriv(phi1,phi4,phi1_d,phi4_d,pitch,pitch_d,0.0001,
                 &Lv,&theta,&theta_d,&L_d,&phi2,&phi3,&phi5);

        if(ld->x0==0&&qw!=0)ld->x0=qw;
        ld->x=WHEEL_R*(qw-ld->x0);
        double x_dot=WHEEL_R*qdw;

        double H=Lv*cos(theta), Hd=L_d*cos(theta)-Lv*sin(theta)*theta_d;
        double F_Leg=0.5*M_BODY*GRAVITY*cos(theta)
                    +KP_LEG*(LEG_LENGTH_REF-H)+KD_LEG*(0-Hd);

        double yMdd=acc[0]*sin(pitch)+acc[2]*cos(pitch);
        double P_est=F_Leg*cos(theta);
        ld->FN=MW*yMdd+P_est+MW*GRAVITY;
        ld->grounded=(ld->FN>=20.0);

        double K[2][6];
        lookup_k_mat(Lv,ld->grounded,K);
        double X[6]={theta,theta_d,ld->x,x_dot,pitch,pitch_d};
        double u[2]={0,0};
        for(int i=0;i<2;i++)for(int j=0;j<6;j++)u[i]+=K[i][j]*(-X[j]);

        double P=F_Leg*cos(theta)+u[1]*sin(theta)/(Lv>0.01?Lv:0.01);
        ld->FN=MW*yMdd+P+MW*GRAVITY;
        ld->theta=theta; ld->theta_dot=theta_d;

        S[leg][0]=phi1;S[leg][1]=phi4;S[leg][2]=phi1_d;S[leg][3]=phi4_d;
        S[leg][4]=Lv;S[leg][5]=theta;S[leg][6]=theta_d;
        S[leg][7]=phi2;S[leg][8]=phi3;S[leg][9]=phi5;
        S[leg][10]=u[0];S[leg][11]=u[1];S[leg][12]=F_Leg;
    }

    // ==== 抗劈叉 ====
    double anti=KP_THETA*(leg_R.theta-leg_L.theta)+KD_THETA*(leg_R.theta_dot-leg_L.theta_dot);

    // ==== pass 2: VMC+output ====
    for(int leg=0;leg<2;leg++){
        int isR=(leg==0);
        double phi1=S[leg][0],phi4=S[leg][1],Lv=S[leg][4];
        double phi2=S[leg][7],phi3=S[leg][8],phi5=S[leg][9];
        double T_w_raw=S[leg][10],T_Leg=S[leg][11],F_Leg=S[leg][12];

        T_Leg+=(isR?anti:-anti);
        double rs=isR?1.0:-1.0, fs=isR?-1.0:1.0;
        double J[2][2];
        vmc_jacobian(phi1,phi2,phi3,phi4,phi5,Lv,J);
        double Tj0=J[0][0]*F_Leg+J[0][1]*T_Leg;
        double Tj1=J[1][0]*F_Leg+J[1][1]*T_Leg;

        d_data->ctrl[act_ids[act_idx_map[leg*2]]]  =clamp(rs*(-Tj0),-TORQUE_LIMIT,TORQUE_LIMIT);
        d_data->ctrl[act_ids[act_idx_map[leg*2+1]]]=clamp(fs*(-Tj1),-TORQUE_LIMIT,TORQUE_LIMIT);
        d_data->ctrl[act_ids[2+leg*3]]              =clamp(wheel_pol[leg]*T_w_raw,-TORQUE_LIMIT,TORQUE_LIMIT);
    }
}

// ======================== main ========================
int main(void) {
    if(init_model(XML_PATH)!=0) { fprintf(stderr,"FATAL: init failed\n"); return 1; }

    double phi1_ref,phi4_ref;
    ik(LEG_LENGTH_REF,&phi1_ref,&phi4_ref);
    printf("=== C Controller === L=%.3fm phi1=%.0f phi4=%.0f\n",
           LEG_LENGTH_REF,deg(phi1_ref),deg(phi4_ref));

    double qhr,qfr,qhl,qfl;
    phi_to_q(1,phi1_ref,phi4_ref,&qhr,&qfr);
    phi_to_q(0,phi1_ref,phi4_ref,&qhl,&qfl);
    d->qpos[jnt_qr[0]]=qhr; d->qpos[jnt_qf[0]]=qfr;
    d->qpos[jnt_qr[1]]=qhl; d->qpos[jnt_qf[1]]=qfl;
    mj_forward(m,d);
    build_k_table(0.08,0.45,0.002);

    mjcb_control = my_controller;

    printf("Simulating...\n");
    double last_print=0;
    for(int step=0;step<100000;step++){
        mj_step(m,d);
        if(d->time-last_print>=1.0){
            double pitch=(freejoint_adr>=0)?get_pitch(d,freejoint_adr):0;
            printf("[t=%.1fs] pitch=%.1fdeg  FN_R=%.0fN FN_L=%.0fN\n",
                   d->time,deg(pitch),leg_R.FN,leg_L.FN);
            last_print=d->time;
        }
    }

    mjcb_control=NULL;
    mj_deleteData(d);
    mj_deleteModel(m);
    printf("Done.\n");
    return 0;
}
