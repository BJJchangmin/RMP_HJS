#include "globals.h"

class MotionPolicy
{
    private:

        Vector3d f; //policy
        Matrix3d G; //Riemannian Metric
        
    public:
    MotionPolicy pullback(MotionPolicy x_rmp);
    MotionPolicy pushforward(MotionPolicy q_rmp);

    MotionPolicy(/* args */);
    ~MotionPolicy();
};



