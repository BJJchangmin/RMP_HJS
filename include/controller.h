#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "globals.h"

class controller
{
private:
    double x_dot_P[4][3];
    double y_dot_P[4][3];
    double z_dot_P[4][3];
    
    double x_dot_I[4][3];
    double y_dot_I[4][3];
    double z_dot_I[4][3];

    double x_dot_D[4][3];
    double y_dot_D[4][3];
    double z_dot_D[4][3];


public:
    controller();
    ~controller();


    void vel_gainset();
    void velPID(StateModel_* state_model);
    void A();
};

// controller::controller(){};
controller::~controller(){};

// void controller::A()
// {
//     cout <<"A" << endl;
// }



#endif