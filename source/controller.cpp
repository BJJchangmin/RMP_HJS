#include "controller.h"


// controller::controller()
// {

// }

void controller::A()
{
    cout <<"A" << endl;
}




void controller::vel_gainset()
{
    int K = 0; // K=0 -> All gains are same
               // K=1 -> Gains are different for each leg
    double vel_P[3]; 
    double vel_I[3];   // x y z
    double vel_D[3];
    
    for (int i = 0; i < 3; i++)
    {
        vel_P[i] = 100;
        vel_I[i] = 0;
        vel_D[i] = 10;
    }
    switch(K){

    case '0':
    for(int i = 0; i < 4; i++){
        for(int j = 0; j < 3; j++){
        x_dot_P[i][j] = vel_P[i];
        x_dot_I[i][j] = vel_I[i];
        x_dot_D[i][j] = vel_D[i];     
        }   
    }
        break;

    // case '1':
    //     //FL x                  //FL y                  //FL z
    //     x_dot_P[0][0] = 100;    x_dot_P[0][1] = 100;    x_dot_P[0][1] = 100;
    //     x_dot_I[0][0] = 0;
    //     x_dot_D[0][0] = 10;
    //     //FR
    //     x_dot_P[1][0] = 100;
    //     x_dot_I[1][0] = 0;
    //     x_dot_D[1][0] = 10;
    //     //RL
    //     x_dot_P[2][0] = 100;
    //     x_dot_I[2][0] = 0;
    //     x_dot_D[2][0] = 10;
    //     //RR
    //     x_dot_P[3][0] = 100;
    //     x_dot_I[3][0] = 0;
    //     x_dot_D[3][0] = 10;
    //     break;
    }

};


void controller::velPID(StateModel_* state_model)
{
    for (int i = 0; i < 3; i++)
    {
    state_model->error_vel[i] = state_model->vel_ref[i] - state_model->vel[i];
    state_model->vel_P_term[i][0] = x_dot_P[i][0] * state_model->error_vel[i];

    //vel_I_term[i][0] = ki * T / 2 * (posRW_err[i] + posRW_err_old[i]) + Pos_I_term[i][1];
    // state_model->vel_D_term[i][0] = 2 * x_dot_D[i][0] / (2 * tau + Ts) * (posRW_err[idx] - posRW_err_old[i]) -
    //                    (Ts - 2 * tau) / (2 * tau + T) * vel_D_term_old[i][1];
    }
};