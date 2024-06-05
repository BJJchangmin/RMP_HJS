//#pragma once
#include "sub_func.h"
#include <controller.h>
#include "kinematics.h"


#include <state.h>
#include <globals.h>
#include <casadi/casadi.hpp>
#include <iostream>


double t;

ParamModel_ paramModel_FL_;
ParamModel_ paramModel_FR_;
ParamModel_ paramModel_RL_;
ParamModel_ paramModel_RR_;

// controller parameters
ParamTuning_ paramTuning_;

// state parameters
StateModel_ stateModel_FL_;
StateModel_ stateModel_FR_;
StateModel_ stateModel_RL_;
StateModel_ stateModel_RR_;

const int leg_FL_no = 0;
const int leg_FR_no = 3;
const int leg_RL_no = 6;
const int leg_RR_no = 9;

controller C;



void mycontroller(const mjModel* m, mjData* d) {



t = d->time;

    d->qpos[0] = 0;
    d->qpos[1] = 0;
    d->qpos[2] = 0.7;   // qpos[0,1,2] : trunk pos                                                                                                                 
                        // qpos[3,4,5.6] : trunk orientation quaternian
    
    d->qpos[3] = -0.73;
    d->qpos[4] = 0.73;
    d->qpos[5] = 0;
    d->qpos[6] = 0;
    //d->qpos[7] = 0; //FLHAA         //d->ctrl[0] FLHAA
    d->qpos[8] = 0; //FLHIP       //d->ctrl[1] FLHIP
    d->qpos[9] = 0; //FLKNEE        //d->ctrl[2] FLKNEE
    //d->qpos[10] = 0; //FRHAA        //d->ctrl[3] FRHAA
    d->qpos[11] = 0; //FRHIP        //d->ctrl[4] FRHIP
    d->qpos[12] = 0; //FRKNEE       //d->ctrl[5] FRKNEE
    //d->qpos[13] = 0; //RLHAA        //d->ctrl[6] RLHAA
    d->qpos[14] = 0; //RLHIP        //d->ctrl[7] RLHIP
    d->qpos[15] = 0; //RLKNEE       //d->ctrl[8] RLKNEE
    //d->qpos[16] = 0; //RRHAA        //d->ctrl[9] RRHAA
    d->qpos[17] = 0; //RRHIP        //d->ctrl[10] RRHIP
    d->qpos[18] = 0; //RRKNEE       //d->ctrl[11] RRKNEE
    
    d->ctrl[0] = 1;
    d->ctrl[3] = 1;
    d->ctrl[6] = 1;
    d->ctrl[9] = 1;

    sensor_measure(m, d, &stateModel_FL_, &paramTuning_, leg_FL_no);
    sensor_measure(m, d, &stateModel_FR_, &paramTuning_, leg_FR_no);
    sensor_measure(m, d, &stateModel_RL_, &paramTuning_, leg_RL_no);
    sensor_measure(m, d, &stateModel_RR_, &paramTuning_, leg_RR_no);

    // C.A();
    // C.vel_gainset();
    
    // calculate model parameters
    model_param_cal(m, d, &paramModel_FL_, &stateModel_FL_);
    model_param_cal(m, d, &paramModel_FR_, &stateModel_FR_);
    model_param_cal(m, d, &paramModel_RL_, &stateModel_RL_);
    model_param_cal(m, d, &paramModel_RR_, &stateModel_RR_);

    // calculate RW Jacobian  => Cartesian좌표계에서 RW좌표계로 변환해주는 자코비안
    jacobian3DRW(&paramModel_FL_, &stateModel_FL_);
    jacobian3DRW(&paramModel_FR_, &stateModel_FR_);
    jacobian3DRW(&paramModel_RL_, &stateModel_RL_);
    jacobian3DRW(&paramModel_RR_, &stateModel_RR_);


    fwdKinematics_cal(&paramModel_FL_, &stateModel_FL_);
    fwdKinematics_cal(&paramModel_FR_, &stateModel_FR_);
    fwdKinematics_cal(&paramModel_RL_, &stateModel_RL_);
    fwdKinematics_cal(&paramModel_RR_, &stateModel_RR_);

    
    
    state_update(&stateModel_FL_);
    state_update(&stateModel_FR_);
    state_update(&stateModel_RL_);
    state_update(&stateModel_RR_);


using namespace casadi;
    
    SX x = SX::sym("x");

 // 목적 함수 정의: f(x) = (x - 3)^2
    SX f = pow(x - 3, 2);

    // NLP 문제 정의
    Function nlp = Function("nlp", {x}, {f});

    // IPOPT 솔버 생성
    Dict opts;
    opts["ipopt.print_level"] = 0;
    // Function solver = nlpsol("solver", "ipopt", nlp, opts);

    // // 초기 값 설정
    // DM x0 = DM::ones(1) * 0;

    // // 최적화 문제 해결
    // std::map<std::string, DM> arg = {{"x0", x0}};    
    // std::map<std::string, DM> res = solver(arg);



    // Matrix3d A;
    // A = Matrix3d::Random();

    



    
    
}

// main function
int main(int argc, const char** argv)
{

    // activate software
    mj_activate("mjkey.txt");


    // load and compile model
    char error[1000] = "Could not load binary model";

    // check command-line arguments
    if( argc<2 )
        m = mj_loadXML(filename, 0, error, 1000);

    else
        if( strlen(argv[1])>4 && !strcmp(argv[1]+strlen(argv[1])-4, ".mjb") )
            m = mj_loadModel(argv[1], 0);
        else
            m = mj_loadXML(argv[1], 0, error, 1000);
    if( !m )
        mju_error_s("Load model error: %s", error);

    // make data
    d = mj_makeData(m);


    // init GLFW
    if( !glfwInit() )
        mju_error("Could not initialize GLFW");

    // create window, make OpenGL context current, request v-sync
    GLFWwindow* window = glfwCreateWindow(1244, 700, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 2000);                // space for 2000 objects
    mjr_makeContext(m, &con, mjFONTSCALE_150);   // model-specific context

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    // double arr_view[] = {89.608063, -11.588379, 5, 0.000000, 0.000000, 0.000000};
    // cam.azimuth = arr_view[0];
    // cam.elevation = arr_view[1];
    // cam.distance = arr_view[2];
    // cam.lookat[0] = arr_view[3];
    // cam.lookat[1] = arr_view[4];
    // cam.lookat[2] = arr_view[5];
    
/* 여기에 컨트롤러 장착!!, mjcb변수 선언 필요없음*/ 
    mjcb_control = mycontroller; // 무한 반복되는 함수


    














    



    // use the first while condition if you want to simulate for a period.
    while( !glfwWindowShouldClose(window))
    {
        // advance interactive simulation for 1/60 sec
        //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
        //  this loop will finish on time for the next frame to be rendered at 60 fps.
        //  Otherwise add a cpu timer and exit this loop when it is time to render.
        mjtNum simstart = d->time;
        while( d->time - simstart < 1.0/60.0 )
        {
            mj_step(m, d);
        }

       // get framebuffer viewport
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

          // update scene and render
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);
        //printf("{%f, %f, %f, %f, %f, %f};\n",cam.azimuth,cam.elevation, cam.distance,cam.lookat[0],cam.lookat[1],cam.lookat[2]);

        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window);

        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();

    }

    // free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // free MuJoCo model and data, deactivate
    mj_deleteData(d);
    mj_deleteModel(m);
    mj_deactivate();

    // terminate GLFW (crashes with Linux NVidia drivers)
    #if defined(__APPLE__) || defined(_WIN32)
        glfwTerminate();
    #endif

    return 1;
}
