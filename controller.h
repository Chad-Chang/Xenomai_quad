#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <data_mutex.h>
#include <actuator.h>
#include <filter.h>


class Controller
{
private:
  // PID //
  Vector4d RW_r_posPgain{
      0,
  }; // FL FR RL RR
  Vector4d RW_r_posIgain{
      0,
  };
  Vector4d RW_r_posDgain{
      0,
  };
  Vector4d RW_r_posD_cutoff{
      0,
  };

  Vector4d RW_th_posPgain{
      0,
  }; // FL FR RL RR
  Vector4d RW_th_posIgain{
      0,
  };
  Vector4d RW_th_posDgain{
      0,
  };
  Vector4d RW_th_posD_cutoff{
      0,
  };

  double posPgain = 0.;
  double posDgain = 0.;
  Vector2d posPID_output{
      0,
  };
  double cutoff_freq = 150;

  // Using in Function
  double Pos_P_term[2][2]; // first column is about r, second column is about theta
  double Pos_I_term[2][2];
  double Pos_D_term[2][2];
  double kp;
  double ki;
  double kd;
  double pos_trajectory;

public:
  Controller();

  // Data set//
  void setDelayData();
  void Mutex_exchange();

  // feedback control //;
  double posPID(Vector2d posRW_err, Vector2d posRW_err_old, int r0th1, int Leg_num); // idx:  r(=0), th(=1)중 어떤 state의 PD control?
  Vector2d velPID();                                                                 // Leg_num: FL-0 FR-1 RL-2 RR-3

  double get_posPgain(int Leg_num, int r0th1) {
    if (r0th1 == 0)
      return RW_r_posPgain[Leg_num];
    else
      return RW_th_posPgain[Leg_num];
  };

  double get_posIgain(int Leg_num, int r0th1) {
    if (r0th1 == 0)
      return RW_r_posIgain[Leg_num];
    else
      return RW_th_posIgain[Leg_num];
  };

  double get_posDgain(int Leg_num, int r0th1) {
    if (r0th1 == 0)
      return RW_r_posDgain[Leg_num];
    else
      return RW_th_posDgain[Leg_num];
  };
  double get_posD_cutoff(int Leg_num, int r0th1) {
    if (r0th1 == 0)
      return RW_r_posD_cutoff[Leg_num];
    else
      return RW_th_posD_cutoff[Leg_num];
  };
  filter Tool;
};

#endif // CONTROLLER_H
