#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include "mujoco/mujoco.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

// ======================== 五杆机构参数 ========================
#define L_ACT 0.21
#define L_SLV 0.25
#define L5    0.0
#define M_BODY 15.040
#define GRAVITY 9.81
#define MW 0.36
#define WHEEL_R 0.077

// ======================== K 多项式系数 (12×3) ========================
static const double K_CONS[12][3] = {
    {121.483141, -117.615829,  -10.894560},
    { 14.523394,  -13.464968,   -1.561974},
    { 36.152404,  -33.115542,   -4.411052},
    { 30.335837,  -27.123183,   -5.447501},
    {168.439063, -176.814480,   65.023157},
    {  9.781195,  -10.806809,    4.790811},
    {-13.213574,   -4.182463,   14.127071},
    {  1.718731,   -4.012695,    2.524995},
    {  0.105623,   -6.399662,    5.503927},
    {  3.630516,   -9.039030,    5.752102},
    {-108.578863,  95.701099,  118.361837},
    { -7.033673,    6.785211,    6.383710},
};

// ======================== K 查表 ========================
#define K_TABLE_SIZE 200
typedef struct { double L; double K[2][6]; } KEntry;
static KEntry k_table_gnd[K_TABLE_SIZE];
static KEntry k_table_air[K_TABLE_SIZE];
static int k_table_len = 0;

static inline void build_k_table(double L_min, double L_max, double step) {
    k_table_len = 0;
    for (double L = L_min; L <= L_max + step*0.5; L += step) {
        double L2 = L*L;
        double kk[12];
        for (int i = 0; i < 12; i++)
            kk[i] = K_CONS[i][0]*L2 + K_CONS[i][1]*L + K_CONS[i][2];

        int n = k_table_len;
        k_table_gnd[n].L = L;
        k_table_gnd[n].K[0][0]=kk[0]; k_table_gnd[n].K[0][1]=kk[1];
        k_table_gnd[n].K[0][2]=kk[2]; k_table_gnd[n].K[0][3]=kk[3];
        k_table_gnd[n].K[0][4]=kk[4]; k_table_gnd[n].K[0][5]=kk[5];
        k_table_gnd[n].K[1][0]=kk[6]; k_table_gnd[n].K[1][1]=kk[7];
        k_table_gnd[n].K[1][2]=kk[8]; k_table_gnd[n].K[1][3]=kk[9];
        k_table_gnd[n].K[1][4]=kk[10];k_table_gnd[n].K[1][5]=kk[11];

        k_table_air[n].L = L;
        k_table_air[n].K[0][0]=0; k_table_air[n].K[0][1]=0;
        k_table_air[n].K[0][2]=0; k_table_air[n].K[0][3]=0;
        k_table_air[n].K[0][4]=0; k_table_air[n].K[0][5]=0;
        k_table_air[n].K[1][0]=kk[6];k_table_air[n].K[1][1]=kk[7];
        k_table_air[n].K[1][2]=0; k_table_air[n].K[1][3]=0;
        k_table_air[n].K[1][4]=0; k_table_air[n].K[1][5]=0;

        k_table_len++;
        if (k_table_len >= K_TABLE_SIZE) break;
    }
}

static inline void lookup_k_mat(double L, int grounded, double K_out[2][6]) {
    // nearest neighbor
    int best = 0; double best_d = fabs(k_table_gnd[0].L - L);
    for (int i = 1; i < k_table_len; i++) {
        double d = fabs(k_table_gnd[i].L - L);
        if (d < best_d) { best_d = d; best = i; }
    }
    KEntry *src = grounded ? &k_table_gnd[best] : &k_table_air[best];
    memcpy(K_out, src->K, sizeof(double)*12);
}

// ======================== phi ↔ raw joint ========================
// Right: rear 0 -1 0 front 0  1 0  |  Left: rear 0  1 0 front 0 -1 0

static inline void phi_to_q(int is_right, double phi1, double phi4, double *q_hip, double *q_sho) {
    if (is_right) { *q_hip = M_PI - phi1; *q_sho =  phi4; }
    else          { *q_hip = phi1 - M_PI; *q_sho = -phi4; }
}

static inline void q_to_phi(int is_right, double q_hip, double q_sho, double *phi1, double *phi4) {
    if (is_right) { *phi1 = -q_hip + M_PI; *phi4 =  q_sho; }
    else          { *phi1 =  q_hip + M_PI; *phi4 = -q_sho; }
}

static inline void phi_dot(int is_right, double qd_hip, double qd_sho, double *p1d, double *p4d) {
    if (is_right) { *p1d = -qd_hip; *p4d =  qd_sho; }
    else          { *p1d =  qd_hip; *p4d = -qd_sho; }
}

// ======================== 正运动学 (FK) ========================
static inline void fk(double phi1, double phi4, double *L, double *theta,
                       double *phi2, double *phi3, double *phi5) {
    double xD = L5 + L_ACT*cos(phi4), yD = L_ACT*sin(phi4);
    double xB = L_ACT*cos(phi1),      yB = L_ACT*sin(phi1);
    double BD = sqrt((xD-xB)*(xD-xB) + (yD-yB)*(yD-yB));
    double A0 = 2*L_SLV*(xD-xB), B0 = 2*L_SLV*(yD-yB);
    double C0 = L_SLV*L_SLV + BD*BD - L_SLV*L_SLV;
    double disc = A0*A0 + B0*B0 - C0*C0;
    if (disc < 0) disc = 0;
    *phi2 = 2*atan2(B0 + sqrt(disc), A0 + C0);
    double xC = L_ACT*cos(phi1) + L_SLV*cos(*phi2);
    double yC = L_ACT*sin(phi1) + L_SLV*sin(*phi2);
    *phi3 = atan2(yC - yD, xC - xD);
    *phi5 = atan2(yC, xC - L5/2);
    *L = sqrt((xC - L5/2)*(xC - L5/2) + yC*yC);
    *theta = *phi5 - M_PI/2;
}

static inline void fk_deriv(double phi1, double phi4, double phi1_d, double phi4_d,
                              double pitch, double pitch_d, double dt,
                              double *L, double *theta, double *theta_d, double *L_d,
                              double *phi2, double *phi3, double *phi5) {
    double alpha, phi2p, alpha_p, Lp, theta_p;
    fk(phi1, phi4, L, &alpha, phi2, phi3, phi5);
    *theta = alpha - pitch;
    fk(phi1+phi1_d*dt, phi4+phi4_d*dt, &Lp, &alpha_p, &phi2p, phi3, phi5);
    theta_p = alpha_p - (pitch + pitch_d*dt);
    *L_d = (Lp - *L)/dt;
    *theta_d = (theta_p - *theta)/dt;
}

// ======================== 逆运动学 (IK) ========================
static inline void ik(double L_ref, double *phi1, double *phi4) {
    double lo = M_PI/2, hi = M_PI, mid, Lm, dummy;
    for (int i = 0; i < 30; i++) {
        mid = (lo+hi)/2;
        fk(mid, M_PI-mid, &Lm, &dummy, &dummy, &dummy, &dummy);
        if (Lm < L_ref) hi = mid; else lo = mid;
    }
    *phi1 = (lo+hi)/2;
    *phi4 = M_PI - *phi1;
}

// ======================== VMC 雅可比 ========================
static inline void vmc_jacobian(double phi1, double phi2, double phi3, double phi4,
                                 double phi5, double L, double J[2][2]) {
    double d32 = sin(phi3-phi2);
    if (fabs(d32) < 1e-6) d32 = (d32 >= 0 ? 1e-6 : -1e-6);
    J[0][0] = L_ACT*sin(phi5-phi3)*sin(phi1-phi2)/d32;
    J[0][1] = L_ACT*cos(phi5-phi3)*sin(phi1-phi2)/L/d32;
    J[1][0] = L_ACT*sin(phi5-phi2)*sin(phi3-phi4)/d32;
    J[1][1] = L_ACT*cos(phi5-phi2)*sin(phi3-phi4)/L/d32;
}

// ======================== IMU 读取 ========================
static inline double get_pitch(const mjData *d, int freejoint_adr) {
    double w=d->qpos[freejoint_adr+3], x=d->qpos[freejoint_adr+4];
    double y=d->qpos[freejoint_adr+5], z=d->qpos[freejoint_adr+6];
    double sp = 2*(w*y - z*x);
    if (sp > 1) sp=1; if (sp < -1) sp=-1;
    return asin(sp);
}
static inline double get_pitch_dot(const mjData *d, int freejoint_adr) {
    return d->qvel[freejoint_adr+4];  // wy
}
static inline void get_acc(const mjData *d, int freejoint_adr, double acc[3]) {
    acc[0]=d->qacc[freejoint_adr+0]; acc[1]=d->qacc[freejoint_adr+1]; acc[2]=d->qacc[freejoint_adr+2];
}

// ======================== 工具 ========================
static inline double deg(double rad) { return rad*180.0/M_PI; }
static inline double radians(double deg) { return deg*M_PI/180.0; }
static inline double clamp(double x, double lo, double hi) { return x<lo?lo:(x>hi?hi:x); }

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_CONFIG_H */
