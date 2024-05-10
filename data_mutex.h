#ifndef DATA_MUTEX_H
#define DATA_MUTEX_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <QTimer>
#include <cstdint>
#include <inttypes.h>
#include <pthread.h>
#include <qcustomplot.h>>
#include <stdio.h>
#include <unistd.h>
#include <vector>

using namespace Eigen;
using std::vector;

#define NUMOFSLAVES 2 // Number of elmo slaves
#define NUMOF_GTWI_SLAVES 2
#define NUMOF_PTWI_SLAVES 0


//// MUTEX TOOLS ////
extern pthread_mutex_t data_mut;
extern timespec data_mut_lock_timeout;
extern int timeout_ns;
extern QTimer *tim;

extern double T;

//// ELMO ////
extern double _M_sampling_time_ms;
extern int _M_overrun_cnt;
extern int _M_Ecat_WKC; //
extern int _M_Ecat_expectedWKC;

extern uint16_t _M_STATUSWORD[NUMOFSLAVES];
extern int8_t _M_MODE_OF_OPERATION_DISPLAY[NUMOFSLAVES];

extern int8_t _M_MODE_OF_OPERATION[NUMOFSLAVES];
extern uint16_t _M_CONTROLWORD[NUMOFSLAVES];

//// Motor parameter ////
extern double _M_motor_position[NUMOFSLAVES];
extern double _M_motor_torque[NUMOFSLAVES];
extern double _M_motor_velocity[NUMOFSLAVES];
extern double _M_ref_current[NUMOFSLAVES];


//// Leg parameter (RW) ////

extern double _M_ref_pos_RL;
extern double _M_ref_vel_RL;
extern Vector4d _M_RW_r_posPgain;
extern Vector4d _M_RW_r_posDgain;
extern Vector4d _M_RW_r_posD_cutoff;
extern Vector4d _M_RW_th_posPgain;
extern Vector4d _M_RW_th_posDgain;
extern Vector4d _M_RW_th_posD_cutoff;

extern Vector4d _M_RW_r_pos;
extern Vector4d _M_RW_th_pos;

extern double _M_pos_ref_test;

#endif // DATA_MUTEX_H
