#ifndef KINEMATICS_H
#define KINEMATICS_H

#include<data_mutex.h>
#include<actuator.h>


class Kinematics
{

private:
    double L = 0.25;
    int NUMOFLEGS = 4;
    Vector2d posRW; // r, th 순서의 2x1벡터
    Vector2d velRW; // r, th 순서의 2x1벡터
    double posRW_error[2][2];
    double velRW_error[2][2];

    double r_posRW[4]{
        0,
    };
    double th_posRW[4]{
        0,
    };

    double r_ref;
    double th_ref;

    Matrix2d Jacobian;
    Matrix2d JacobianTrans;

  public:
    Kinematics(double thm, double thb, int Leg_num);
    void set_DelayDATA();
    Matrix2d RW_Jacobian();
    Vector2d get_posRW() { return posRW; };
    Vector2d get_posRW_error(int idx);
    double *get_velRW(double thm_dot, double thb_dot);
    Matrix2d get_RW_Jacobian() { return Jacobian; };
    Matrix2d get_RW_Jacobian_Trans() { return JacobianTrans; };
    void exchange_mutex();
    double trajectory_generation(double t);
};

#endif // KINEMATICS_H
