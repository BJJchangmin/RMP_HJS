#include <globals.h>


void state_update(StateModel_ *state_model)
{
    for (int i = 0; i < NDOF_LEG; i++)
    {
        // Joint coordinates update

        state_model->q_bi_old[i] = state_model->q_bi[i];
        state_model->qdot_bi_old[i] = state_model->qdot_bi[i];
        state_model->qdot_bi_tustin_old[i] = state_model->qdot_bi_tustin[i];
        state_model->qddot_bi_tustin_old[i] = state_model->qddot_bi_tustin[i];

        state_model->tau_bi_old[i] = state_model->tau_bi[i];

        // RW coordinates update
        state_model->posRW_old[i] = state_model->posRW[i];
        state_model->posRW_ref_old[i] = state_model->posRW_ref[i];
        
        // Feedback control update
        state_model->error_pos_old[i] = state_model->error_pos[i];
        state_model->error_dot_pos_old[i] = state_model->error_dot_pos[i];

        // state_model->ctrl_input_RW_old[i] = state_model->ctrl_input_RW[i];


    }

    for (int i = 0; i < 3; i++)
    {
        state_model->q_old[i] = state_model->q[i];

        state_model->vel_old[i] = state_model->vel[i];
        state_model->vel_ref_old[i] = state_model->vel_ref[i];

        state_model->error_vel_old[i] = state_model->error_vel[i];
        state_model->error_dot_vel_old[i] = state_model->error_dot_vel[i];
    }
}