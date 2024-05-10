#include<data_mutex.h>
#include<QTimer>

double _M_sampling_time_ms = 0; //sync(send) data from RT thread to GUI
int _M_overrun_cnt = 0;

uint16_t    _M_Ecat_states[NUMOFSLAVES+1+1];
int         _M_Ecat_WKC = 0;    //Actual Working Counter
int         _M_Ecat_expectedWKC = 0;    //Working Counter which is expected.

uint16_t    _M_STATUSWORD[NUMOFSLAVES];
int8_t      _M_MODE_OF_OPERATION_DISPLAY[NUMOFSLAVES];

int8_t      _M_MODE_OF_OPERATION[NUMOFSLAVES];
uint16_t    _M_CONTROLWORD[NUMOFSLAVES];

using std:: vector;

//// Parameter ////
double T = 0.001;


//// Motor parameter ////

double _M_motor_position[NUMOFSLAVES];
double _M_motor_torque[NUMOFSLAVES];
double _M_motor_velocity[NUMOFSLAVES];
double _M_ref_current[NUMOFSLAVES];


//// Leg parameter (RW) ////

double _M_ref_pos_RL;
double _M_ref_vel_RL;
double _M_pos_ref_test;

Vector4d _M_RW_r_posPgain;
Vector4d _M_RW_r_posDgain;
Vector4d _M_RW_r_posD_cutoff;
Vector4d _M_RW_th_posPgain;
Vector4d _M_RW_th_posDgain;
Vector4d _M_RW_th_posD_cutoff;

Vector4d _M_RW_r_pos;
Vector4d _M_RW_th_pos;

//// Mutex connect ////
timespec data_mut_lock_timeout;
int timeout_ns;
QTimer *tim;
