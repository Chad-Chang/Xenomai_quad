#include "kinematics.h"

Kinematics::Kinematics(double th1, double th2, int Leg_num) { // th1 << thm, th2 << thb 바꿈
// biarticular 각도 재정의
  double thm = th1;
  double thb = th1 + th2;

  Jacobian(0, 0) = sin(th2 / 2);
  Jacobian(0, 1) = -sin(th2 / 2);
  Jacobian(1, 0) = cos(th2 / 2);
  Jacobian(1, 1) = cos(th2 / 2);

  Jacobian = L * Jacobian;
  JacobianTrans = Jacobian.transpose();

  posRW[0] = 2 * L * cos((thb - thm) / 2); // biarticular -> RW변환 
  posRW[1] = 0.5 * (thm + thb);

  r_posRW[Leg_num] = posRW[0];
  th_posRW[Leg_num] = posRW[1];
}

void Kinematics::set_DelayDATA() {
  for (int i = 0; i < 2; i++) //[i][0] = z^0, [i][1] = z^1 ...
  {

    // Delay data
    posRW_error[i][1] = posRW_error[i][0];
    velRW_error[i][1] = velRW_error[i][0];
  }
}

// joint to biarticular
Vector2d Kinematics::joint2bi(Vector2d Joint_input)
{
  Vector2d bi_output[2] = {0};
  bi_output[0] = Joint_input[0];// monoarticular
  bi_output[1] = Joint_input[0] + Joint_input[1]; //biarticular 
  return bi_output
}

void Kinematics::exchange_mutex() {
  if (!pthread_mutex_trylock(&data_mut)) {
    for (int i = 0; i < NUMOFLEGS; i++) {
      _M_RW_r_pos[i] = r_posRW[i];
      _M_RW_th_pos[i] = th_posRW[i];
    }
    pthread_mutex_unlock(&data_mut);
  }
}

Matrix2d Kinematics::RW_Jacobian() { return Jacobian; };

Vector2d Kinematics ::get_posRW_error(int idx) // idx = 0이면 현재 값, idx = 1이면 이전 값
{

  Vector2d RWpos_error;
  Vector2d RWpos_error_old;

  if (idx == 0) {
    RWpos_error[0] = posRW_error[0][0];
    RWpos_error[1] = posRW_error[1][0];

    return RWpos_error;
  } else {
    RWpos_error_old[0] = posRW_error[0][1];
    RWpos_error_old[1] = posRW_error[1][1];

    return RWpos_error_old;
  }
}

double *Kinematics ::get_velRW(double thm_dot, double thb_dot) {}
