#include "controller.h"

Controller::Controller()
{

}

double Controller::posPID(Vector2d posRW_err, Vector2d posRW_err_old, int idx,
                          int Leg_num) // posRW_err, posRW_err_old 이라는 주소값을 받겠다. PID 제어를 r,th둘다 해야하는데 하나의 함수로 하기위해,
                                       // idx라는 int변수에 0을 넣으면 r에대한것, 1을 넣으면 th에 대한 PID 제어기가 된다.
{
  kp = get_posPgain(Leg_num, idx);
  ki = get_posIgain(Leg_num, idx);
  kd = get_posDgain(Leg_num, idx);
  cutoff_freq = get_posD_cutoff(Leg_num, idx);
  double tau = 1 / (2 * M_PI * cutoff_freq);

  Pos_P_term[idx][0] = kp * posRW_err[idx];
  Pos_I_term[idx][0] = ki * T / 2 * (posRW_err[idx] + posRW_err_old[idx]) + Pos_I_term[idx][1];
  Pos_D_term[idx][0] = 2 * kd / (2 * tau + T) * (posRW_err[idx] - posRW_err_old[idx]) -
                       (T - 2 * tau) / (2 * tau + T) * Pos_D_term[idx][1]; // 이 함수 내에서 r_err는 주소값 but [0]와 같은 배열 위치로 원소를
                                                                           // 특정해주면 그 부분의 value가 된다.(이건 그냥 c++ 문법)

  posPID_output[idx] = Pos_P_term[idx][0] + Pos_I_term[idx][0] + Pos_D_term[idx][0];

  return posPID_output[idx];
  //
  setDelayData();
}

void Controller::setDelayData() {
  for (int i = 0; i < 2; i++) //[i][0] = z^0, [i][1] = z^1 ->  delay data 만들어 주는 function
  {
    /****************** Delay Data ******************/

    // r
    Pos_D_term[i + 1][0] = Pos_D_term[i][0];
    Pos_P_term[i + 1][0] = Pos_P_term[i][0];

    // th
    Pos_D_term[i + 1][1] = Pos_D_term[i][1];
    Pos_P_term[i + 1][1] = Pos_P_term[i][1];
  }
}

void Controller::Mutex_exchange() // Mutex에서 데이터를 받아오는 function.  Ui gain setting -> Mutex -> 여기 순서대로 이동 된 값임
{
    // RECEIVE
    if (!pthread_mutex_trylock(&data_mut)) {

        for (int i = 0; i < 4; i++) {

          RW_r_posPgain[i] = _M_RW_r_posPgain[i];
          RW_r_posDgain[i] = _M_RW_r_posDgain[i];
          RW_r_posD_cutoff[i] = _M_RW_r_posD_cutoff[i];
          RW_th_posPgain[i] = _M_RW_th_posPgain[i];
          RW_th_posDgain[i] = _M_RW_th_posDgain[i];
          RW_th_posD_cutoff[i] = _M_RW_th_posD_cutoff[i];
        }
        pthread_mutex_unlock(&data_mut);
    }

    // SEND
}
