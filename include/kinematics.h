#include "globals.h"
#include "filter.h"

void model_param_cal(const mjModel* m, mjData* d, ParamModel_* param_model, StateModel_* state_model)
{
    /* Trunk Parameters */
    // 무게 지정
    param_model->m_hip = 2.5;
    param_model->m_trunk_front = 10.;
    param_model->m_trunk_rear = 18.;
    param_model->m_trunk = 4 * param_model->m_hip + param_model->m_trunk_front + param_model->m_trunk_rear;

    /* Leg Parameters */
    param_model->L = 0.25; // 허벅지 링크 길이 = 종아리 링크 길이 
    param_model->d_thigh = 0.11017; // local position of CoM of thigh
    param_model->d_shank = 0.12997; // local position of CoM of shank

    param_model->m_thigh = 1.017; // mass of thigh link
    param_model->m_shank = 0.143; // mass of shank link
    param_model->m_leg = param_model->m_thigh + param_model->m_shank;
    param_model->m_total = param_model->m_trunk + 4 * param_model->m_leg;

    param_model->Izz_thigh = 0.0057;     // MoI of thigh w.r.t. CoM
    param_model->Izz_shank = 8.0318e-04; // MoI of shank w.r.t. CoM

    //평행축 정리 적용(회전축에 대해 정의된 inertia)
    param_model->Jzz_thigh =
        param_model->Izz_thigh + param_model->m_thigh * pow(param_model->d_thigh, 2); // MoI of thigh w.r.t. HFE : thigh
    param_model->Jzz_shank =
        param_model->Izz_shank + param_model->m_shank * pow(param_model->d_shank, 2); // MoI of thigh w.r.t. KFE : shank

    // M1 = Izz_1 + m_1*(d_1)^2 + m_2*L^2
    double M1 = param_model->Jzz_thigh + param_model->m_shank * pow(param_model->L, 2);
    // M2 = (m_2) * (d_2) * (L*cos(q(1)))
    double M2 = param_model->m_shank * param_model->d_shank * param_model->L * cos(state_model->q2);
    // M12 = Izz_2 + m_2*(d_2)^2
    double M12 = param_model->Jzz_shank;

    // 다리에 대한 inertia matrix(cartesian 좌표계상에서)
    param_model->MatInertia_bi[0] = M1;
    param_model->MatInertia_bi[1] = M12;
    param_model->MatInertia_bi[2] = M12;
    param_model->MatInertia_bi[3] = M2;

    // RW 좌표계로 inertia 변환
    param_model->JzzR_thigh = param_model->Jzz_thigh + param_model->m_shank * pow(param_model->L, 2) +
        param_model->Jzz_shank -
        2 * param_model->m_shank * param_model->d_shank * param_model->L * cos(state_model->q2);

    param_model->JzzR_couple =
        param_model->Jzz_thigh + param_model->m_shank * pow(param_model->L, 2) - param_model->Jzz_shank;
    param_model->JzzR_shank = param_model->Jzz_thigh + param_model->m_shank * pow(param_model->L, 2) +
        param_model->Jzz_shank +
        2 * param_model->m_shank * param_model->d_shank * param_model->L * cos(state_model->q2);

    // RW 좌표계로의 inertia matrix
    param_model->MatInertia_RW[0] =
        param_model->JzzR_thigh / (4 * pow(param_model->L, 2) * pow(sin(state_model->q2 / 2), 2));
    param_model->MatInertia_RW[1] = param_model->JzzR_couple / (2 * pow(param_model->L, 2) * sin(state_model->q2));
    param_model->MatInertia_RW[2] = param_model->JzzR_couple / (2 * pow(param_model->L, 2) * sin(state_model->q2));
    param_model->MatInertia_RW[3] =
        param_model->JzzR_shank / (4 * pow(param_model->L, 2) * pow(cos(state_model->q2 / 2), 2));
} // param_model parameter

void sensor_measure(const mjModel* m, mjData* d, StateModel_* state_model, ParamTuning_* param_tuning, int leg_no)
{

// joint position sensor data
    /*** (Serial) Joint position ***/
    state_model->q2 = d->sensordata[leg_no + 8];
    state_model->q[0] = d->sensordata[leg_no + 6];
    state_model->q[1] = d->sensordata[leg_no + 7]; // (relative) HFE angle
    state_model->q[2] = d->sensordata[leg_no + 8] + d->sensordata[leg_no + 8]; // (relative) KFE angle

    /*** Biarticular Transformation ***/ 
    state_model->q_bi[0] = d->sensordata[leg_no + 7];                             // (absolute) HFE angle
    state_model->q_bi[1] = d->sensordata[leg_no + 7] + d->sensordata[leg_no + 8]; // (absolute) KFE angle

// angular velocity data_ real data
    state_model->qdot_bi[0] = d->qvel[leg_no + 8];
    state_model->qdot_bi[1] = d->qvel[leg_no + 8] + d->qvel[leg_no + 9];

//angular acceleration data_real data
    state_model->qddot_bi[0] = d->qacc[leg_no + 8];
    state_model->qddot_bi[1] = d->qacc[leg_no + 8] + d->qacc[leg_no + 9];

    for (int i = 0; i < NDOF_LEG; i++)
    {
        state_model->qdot_bi_tustin[i] =
            tustin_derivative(state_model->q_bi[i], state_model->q_bi_old[i], state_model->qdot_bi_tustin_old[i],
                param_tuning->freq_cut_D);
        state_model->qddot_bi_tustin[i] =
            tustin_derivative(state_model->qdot_bi_tustin[i], state_model->qdot_bi_tustin_old[i],
                state_model->qddot_bi_tustin_old[i], param_tuning->freq_cut_D);
    }

    // trunk position
    for (int i = 0; i < 3; i++)
        pos_trunk_act[i] = d->sensordata[37 + i];

    // Trunk orientation (Euler angle) ir global coordinates
    double qw = d->sensordata[40], qx = d->sensordata[41], qy = d->sensordata[42], qz = d->sensordata[43];
    double sinr_cosp = 2 * (qw * qx + qy * qz);
    double cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
    pos_trunk_act[3] = pi / 2 + atan2(sinr_cosp, cosr_cosp);
    // pos_trunk_act[3] = atan2(sinr_cosp, cosr_cosp);
    // pos_trunk_act[3] *= 180 / pi;

    double sinp = sqrt(1 + 2 * (qw * qy - qx * qz));
    double cosp = sqrt(1 - 2 * (qw * qy - qx * qz));
    pos_trunk_act[4] = 2 * atan2(sinp, cosp) - pi / 2;
    // pos_trunk_act[4] *= 180 / pi;

    double siny_cosp = 2 * (qw * qz + qx * qy);
    double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
    pos_trunk_act[5] = atan2(siny_cosp, cosy_cosp);
    // pos_trunk_act[5] *= 180 / pi;

    // Trunk orientation (Rotation Matrix)
    double a = pos_trunk_act[3], b = pos_trunk_act[4], r = pos_trunk_act[5];
    rot_mat_trunk_act[0] = cos(b) * cos(r);
    rot_mat_trunk_act[1] = sin(a) * sin(b) * cos(r) - cos(a) * sin(r);
    rot_mat_trunk_act[2] = cos(a) * sin(b) * cos(r) + sin(a) * sin(r);
    rot_mat_trunk_act[3] = cos(b) * sin(r);
    rot_mat_trunk_act[4] = sin(a) * sin(b) * sin(r) + cos(a) * cos(r);
    rot_mat_trunk_act[5] = cos(a) * sin(b) * sin(r) - sin(a) * cos(r);
    rot_mat_trunk_act[6] = -sin(b);
    rot_mat_trunk_act[7] = sin(a) * cos(b);
    rot_mat_trunk_act[8] = cos(a) * cos(b);

    // trunk translational velocity
    for (int i = 0; i < 3; i++)
        vel_trunk_act[i] = d->sensordata[34 + i];

    // trunk angular velocity
    for (int i = 3; i < 6; i++)
        vel_trunk_act[i] = d->sensordata[65 + i]; // global frame
    // vel_trunk_act[i] = d->sensordata[i];    // body frame

    // foot position represented in global frame
    for (int i = 0; i < 3; i++)
        // state_model->foot_pos_act[i] = d->sensordata[leg_no + 44 + i] - pos_trunk_act[i];
        state_model->foot_pos_act[i] = d->sensordata[leg_no + 44 + i];
}

void jacobian3DRW(ParamModel_* param_model, StateModel_* state_model)
{
    /*** Rotating Workspace ***/

    state_model->jacbRW(0,0) = 0;
    state_model->jacbRW(0,1) = param_model->L * cos(state_model->q2 / 2);
    state_model->jacbRW(0,2) = param_model->L * cos(state_model->q2 / 2);
    state_model->jacbRW(1,0) = 2 * param_model->L * cos(state_model->q2 / 2);
    state_model->jacbRW(1,1) = 0;
    state_model->jacbRW(1,2) = 0;
    state_model->jacbRW(2,0) = 0;
    state_model->jacbRW(2,1) = param_model->L * sin(state_model->q2 / 2);
    state_model->jacbRW(2,2) = -param_model->L * sin(state_model->q2 / 2);
    
    state_model->jacbRW_trans = state_model->jacbRW.transpose(); 

    state_model->jacbRW_trans_inv = state_model->jacbRW_trans.inverse();

}

void fwdKinematics_cal(ParamModel_* param_model, StateModel_* state_model) 
{
    state_model->posRW[0] = 2 * param_model->L * cos((state_model->q_bi[1] - state_model->q_bi[0]) / 2); // r
    state_model->posRW[1] = (state_model->q_bi[0] + state_model->q_bi[1]) / 2;                           // qr


    state_model->vel = state_model->jacbRW * state_model->q;
    
    //mju_mulMatVec(state_model->velRW, state_model->jacbRW, state_model->qdot_bi_tustin, 2, 2);
}
