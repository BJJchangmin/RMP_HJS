#include "MotionPolicy.h"

MotionPolicy MotionPolicy::pullback(MotionPolicy x_rmp)
{
    MotionPolicy q_rmp;


    //need Jacobian 
    Matrix3d J = Matrix3d::Random();

    q_rmp.f = pinv(J.transpose() * x_rmp.G * J) * J.transpose() * x_rmp.G * x_rmp.f;
    q_rmp.G = J.transpose() * x_rmp.G * J;
   
    return q_rmp;
}

MotionPolicy MotionPolicy::pushforward(MotionPolicy q_rmp)
{

    MotionPolicy x_rmp;

    //need Jacobian 
    Matrix3d J = Matrix3d::Random();

    x_rmp.f = J * q_rmp.f;
    x_rmp.G = pinv(J) * q_rmp.G * pinv(J).transpose();

    return x_rmp;
}