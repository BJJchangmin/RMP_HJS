#ifndef GLOBALS_H_
#define GLOBALS_H_

#include <mujoco/mujoco.h>
#include <stdbool.h>
#include <stdio.h>
#include <Math.h>

#include <iostream>
#include <eigen-master/Eigen/Core>
#include <eigen-master/Eigen/Dense>

using namespace std;
using namespace Eigen;


#define NDOF_TRUNK 3 // #(DoF) of trunk
#define NDOF_LEG 2   // #(DoF) of leg
#define NUM_LEG 4

/* Data Logging Variables */
FILE *fid;
int loop_index = 0;
const int data_frequency = 100; // frequency at which data is written to a file

char datapath[] = "../RMP/data/data_gyro.csv";
char filename[] = "../RMP/scene.xml";
char datafile[] = "data/data_gyro.csv";

const double Ts = 0.0001; // sampling period
const double g = 9.81;    // gravitational accel.
const double pi = 3.141592;


/* Trunk Dimension */
const double width_trunk = 0.25;
const double length_trunk = 0.66;
const double length_front = 0.34;
const double length_rear = 0.32;
const double height_trunk = 0.083;

double jacb_trunk_pos[NUM_LEG * NUM_LEG];
double jacb_trunk_pos_inv[NUM_LEG * NUM_LEG];
double jacb_trunk_vel[NUM_LEG * NDOF_TRUNK];

/* Trunk States */
double pos_trunk_act[6];
double vel_trunk_act[6];
double rot_mat_trunk[9];
double rot_mat_trunk_act[9];

double pos_trunk_des[NDOF_TRUNK];
double pos_trunk_des_old[NDOF_TRUNK];
double pos_trunk_est[NDOF_TRUNK];
double pos_trunk_est_old[NDOF_TRUNK];

double vel_trunk_des[NDOF_TRUNK];
double vel_trunk_des_old[NDOF_TRUNK];
double vel_trunk_est[NDOF_TRUNK];
double vel_trunk_est_old[NDOF_TRUNK];

double error_pos_trunk[NDOF_TRUNK];
double error_pos_trunk_old[NDOF_TRUNK];
double error_dot_pos_trunk[NDOF_TRUNK];
double error_dot_pos_trunk_old[NDOF_TRUNK];

double error_vel_trunk[NDOF_TRUNK];
double error_vel_trunk_old[NDOF_TRUNK];

double ctrl_input_RW_from_trunk[NUM_LEG];

/* Model Variables */
struct ParamModel_
{
    /* Trunk Parameter */
    double m_hip;         // mass of hip torso
    double m_trunk_front; // mass of front trunk
    double m_trunk_rear;  // mass of rear trunk
    double m_trunk;       // total mass of trunk
    double m_total;       // total robot mass

    double Ixx_trunk;
    double Iyy_trunk;
    double Izz_trunk;

    /* Leg Parameter */
    double L; // leg length : thigh and shank links' length are assumed to be the same

    double m_thigh; // mass of thigh link
    double m_shank; // mass of shank link
    double m_leg;   // mass of leg

    double d_thigh; // CoM pos of thigh w.r.t HFE
    double d_shank; // CoM pos of shank w.r.t KFE

    double Izz_thigh; // MoI(z) of thigh w.r.t its CoM
    double Izz_shank; // MoI(z) of shank w.r.t its CoM

    double Jzz_thigh; // MoI(z) of thigh w.r.t HFE
    double Jzz_shank; // MoI(z) of shank w.r.t KFE

    double JzzR_thigh; // RW 좌표계 상에서의 thigh inertia matrix
    double JzzR_shank; // RW 좌표계에서의 shank inertia matrix 
    double JzzR_couple; // RW 좌표계에서의 반대각행렬

    double MatInertia_bi[NDOF_LEG * NDOF_LEG];
    double MatInertia_RW[NDOF_LEG * NDOF_LEG];
};

struct StateModel_
{
    double foot_pos_act[3];

    /* Joint Coordinates */
    double q2;  // thm - thb
    Vector3d q; // Serial Coordinates
    Vector3d q_old;

    double q_bi[NDOF_LEG]; // biarticular joint angle
    double q_bi_old[NDOF_LEG];

    double qdot_bi[NDOF_LEG]; // biarticular joint angular vel (sensor)
    double qdot_bi_old[NDOF_LEG];
    double qddot_bi[NDOF_LEG]; // biarticular joint angular acc (sensor)

    double qdot_bi_tustin[NDOF_LEG]; // biarticular joint angular vel (derivative)
    double qdot_bi_tustin_old[NDOF_LEG];

    double qddot_bi_tustin[NDOF_LEG]; // biarticular joint angular acc (derivative)
    double qddot_bi_tustin_old[NDOF_LEG];

    double tau_bi[NDOF_LEG]; // (Biarticular) joint torques
    double tau_bi_old[NDOF_LEG];

    /* PD Controller */
    double error_pos[NDOF_LEG];
    double error_pos_old[NDOF_LEG];
    double error_dot_pos[NDOF_LEG];
    double error_dot_pos_old[NDOF_LEG];

    Vector3d error_vel;
    Vector3d error_vel_old;
    Vector3d error_dot_vel;
    Vector3d error_dot_vel_old;

    double vel_P_term[4][3];
    double vel_I_term[4][3];
    double vel_D_term[4][3];
    
    double vel_P_term_old[4][3];
    double vel_I_term_old[4][3];
    double vel_D_term_old[4][3];
    

    /* Rotating Workspace Coordinates */
    double r0;              // initial leg length -> this is used for trajectory generation
    double posRW[NDOF_LEG]; // RW position
    double posRW_old[NDOF_LEG];
    double posRW_ref[NDOF_LEG]; // RW position reference
    double posRW_ref_old[NDOF_LEG];

    Vector3d vel; // RW velocity
    Vector3d vel_old;
    Vector3d vel_ref; // RW velocity reference
    Vector3d vel_ref_old;

    double ctrl_input_RW[NDOF_LEG]; // control input
    double ctrl_input_RW_old[NDOF_LEG];

    /* Jacobian (Rotating Workspace) */
    Matrix3d jacbRW;
    Matrix3d jacbRW_trans;
    Matrix3d jacbRW_trans_inv;

    /* Rotating Workspace Disturbance Observer */
    double lhs_dob_LPF[NDOF_LEG];
    double lhs_dob_LPF_old[NDOF_LEG * NDOF_LEG];
    double rhs_dob_LPF[NDOF_LEG];
    double rhs_dob_LPF_old[NDOF_LEG * NDOF_LEG];

    double tauDist_hat[NDOF_LEG];

    /* Rotating Workspace Force Observer */
    double lhs_fob_LPF[NDOF_LEG];
    double lhs_fob_LPF_old[NDOF_LEG];
    double rhs_fob_LPF[NDOF_LEG];
    double rhs_fob_LPF_old[NDOF_LEG];

    double tauExt_hat[NDOF_LEG];   // estimated external torque
    double forceExt_hat[NDOF_LEG]; // estimated external force (e.g. GRF)
    double forceExt_hat_old[NDOF_LEG];
    double forceExt_hat_old2[NDOF_LEG];

};

// Controller Variables
struct ParamTuning_
{
     // Leg PD
    double Kp_hip;
    double Kd_hip;

    double Kp_pos_leg[NDOF_LEG];
    double Kd_pos_leg[NDOF_LEG];
    // double Ki_pos[NDOF_LEG];

    double Kp_vel_leg[NDOF_LEG];
    double Kd_vel_leg[NDOF_LEG];
    // double Ki_vel[NDOF_LEG];

    double freq_cut_D;
    double freq_cut_Qd;
    double freq_cut_Qf;
};

/* RMP */
Vector2d x_rmp;
Vector2d q_rmp;

#endif // globals_