////////////////////////////////////////////////
///Title: RT_Master_Bimanipulator_Platinum_v1 - main.cpp
///Functions: Multislave+Anybus
///Author: Copyright (C) 2022- Taehoon Kim
///Date: 2022.09.01
///Finished: 2022.09.02
////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////* INDEX *//////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////*

/// 1. INCLUDE
/// 2. MUTEX VARIABLE
/// 3. SETUP FOR RT THREAD
/// 4. DEFINITION FOR SOEM
/// 5. FUNCTION & VARIABLES DECLARATION
/// 6. MAIN FUNCTION
/// 7. FUNCTION DEFINITION
///     1) clean up
///     2) realtime_thread
///         (1) Parameter setting
///         (2) EherCAT MASTER
///         (3) Realtime loop
///             - Control loop




///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////* INCLUDE */////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
///
#include "mainwindow.h"
#include "controlwindow.h"

#include <QApplication>

///* General includes */
#include <errno.h>    //Header for defining macros for reporting and retrieving error conditions using the symbol 'errno'
#include <error.h>    //
#include <fcntl.h>    //C POSIX lib header. Header for opening and locking files and processing other tasks.
#include <inttypes.h> //
#include <iostream>
#include <malloc.h> //Memory allocation
#include <math.h>
#include <pthread.h>  //Header for using Thread operation from xenomai pthread.h
#include <rtdm/ipc.h> //
#include <signal.h>   //Header for signal processing
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>    //
#include <sys/timerfd.h> //
#include <unistd.h>      //C POSIX lib header. Header for accessing to the POSIX OS API

///* Personal includes *///
//Have to write includes in CMakeLists.txt in order to fully
//include personal headers.

#include <actuator.h>
#include <controller.h>
#include <data_exchange_mutex.h>
#include <data_mutex.h>
#include <ecat_func.h>
#include <ethercat.h>
#include <filter.h>
#include <kinematics.h>
#include <qcustomplot.h>


using namespace std;


#ifndef PI
#define PI	(3.14159265359)
#define PI2	(6.28318530718)
#endif

///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////* MUTEX VARIABLE *//////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

pthread_mutex_t data_mut = PTHREAD_MUTEX_INITIALIZER;


///////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////* SETUP FOR RT THREAD *///////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

#define RT_PERIOD_MS 1 //1msec
//#define CPU_AFFINITY_NUM 0 //Up to {$ grep processor /proc/cpuinfo | wc -l} - 1, 0~3 for Mini PC
#define XDDP_PORT 0 //0~{CONFIG-XENO_OPT_PIPE_NRDEV-1} //XENO_OPT_PIPE_NRDEV: No. of pipe devices

pthread_t rt;
int sigRTthreadKill = 0;

///////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////* DEFINITION FOR SOEM *///////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

const char *IFNAME = "eno1"; //Checked from SOEM simpletest //char*->const char*: After C++11,
char IOmap[4096]; //
int usedmem; //

int expectedWKC; //Working Counter:
volatile int wkc; //Doesn't want to occupy memory location

//Gold Twitter
//Mensioned in ecat_func.h as extern
//{No. of entries to be mapped, Entry addr., Entry addr., ...}
uint16 RXPDO_ADDR_GTWI[3] = {2, 0x1600, 0x1605}; //
//uint16 TXPDO_ADDR_GTWI[5] = {4, 0x1A02, 0x1A03, 0x1A18, 0x1A1D};
//uint16 TXPDO_ADDR_GTWI[6] = {5, 0x1A02, 0x1A03, 0x1A18, 0x1A1D, 0x1A1E};
uint16 TXPDO_ADDR_GTWI[3] = {2, 0x1A02,0x1A1E};

//Platinum Twitter
//uint16 RXPDO_ADDR_PTWI[2] = {1, 0x1600};
//uint16 TXPDO_ADDR_PTWI[2] = {1, 0x1A00};

//TS
uint16 TXPDO_ADDR_TS[4] = {3, 0x1A01, 0x1A02,0x1A03};

///////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////* FUNCTION & VARIABLES DECLARATION */////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////


static void cleanup(void); //Delete, release of all handle, memory
static void *realtime_thread(void *arg); //RT thread

///////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////* Class declaration *//////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

Controller c;

// Jacobian
Matrix2d J_RL;
Matrix2d JTrans_RL;

// Controller output //
Vector2d RL_output;
Vector2d RL_control_input;

Actuator RLHIP(0, 0.546812);
Actuator RLKNEE(1, 2.59478);
Kinematics K_RL(RLHIP.getMotor_pos(), RLKNEE.getMotor_pos(), 3);

///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////* Variable declaration *////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

// double t = 0;
int t = 0;
Vector2d posRW;
Vector2d velRW;
Vector2d posRW_err;
Vector2d posRW_err_old;
Vector2d velRW_err;

///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////* MAIN FUNCTION *///////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[]) {
  mlockall(MCL_CURRENT | MCL_FUTURE); // Lock the memory page to prevent performance degrade

  pthread_attr_t rtattr, regattr; //
  sigset_t set;
  int sig;
  cpu_set_t cpus;
  int cpu_num = 0;

  sigemptyset(&set); //
  sigaddset(&set, SIGINT);
  sigaddset(&set, SIGTERM);
  sigaddset(&set, SIGHUP);
  pthread_sigmask(SIG_BLOCK, &set, NULL); // to send the system signal into the thread, not implemented in the thread yet.

  ////* THREAD SETTING *////
  struct sched_param p;
  int ret;

  ret = pthread_attr_init(&rtattr); // initialize pthread attribute
  if (ret)
    error(1, ret, "pthread_attr_init()");

  ret = pthread_attr_setinheritsched(&rtattr, PTHREAD_EXPLICIT_SCHED); // pthread scheduling inherit setting as explicit
  if (ret)
    error(1, ret, "pthread_attr_setinheritsched()");

  ret = pthread_attr_setschedpolicy(&rtattr, SCHED_FIFO); // pthread scheduling policy setting as FIFO
  if (ret)
    error(1, ret, "pthread_attr_setschedpolicy()");

  p.sched_priority = 99;
  ret = pthread_attr_setschedparam(&rtattr, &p); // setting scheduler parameter - priority 99 (Highest)
  if (ret)
    error(1, ret, "pthread_attr_setschedparam()");

  CPU_ZERO(&cpus);
  CPU_SET(cpu_num, &cpus);
  ret = pthread_attr_setaffinity_np(&rtattr, sizeof(cpus), &cpus); // give cpu affinity to be used to calculate for the RT thread
  if (ret)
    error(1, ret, "pthread_attr_setaffinity_np()");

  ret = pthread_create(&rt, &rtattr, realtime_thread, NULL); // create RT thread
  if (ret)
    error(1, ret, "pthread_create(realtime_thread)");

  pthread_attr_destroy(&rtattr); // delete pthread attribute union

  QApplication a(argc, argv);
  MainWindow w;
  Controlwindow c;

  w.show();
  c.show();

  ret = a.exec();      // Execute the mainwindow thread and return when GUI is terminated
  sigRTthreadKill = 1; // set(send) the kill signal to the RT thread

  usleep(2000); // wait time for exit the thread
  cleanup();    // clean up the thread

  return ret;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////* FUNCTION DEFINITION */////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

static void cleanup(void)
{
     pthread_cancel(rt); //thread end
     pthread_join(rt, NULL); //wait for thread is ended
}

static void *realtime_thread(void *arg)
{

#ifdef SLAVE_GTWI
     output_GTWI_t *out_twitter_GTWI[NUMOFSLAVES];    //RxPDO mapping data (output to the slaves)
     input_GTWI_t *in_twitter_GTWI[NUMOFSLAVES];      //TxPDO mapping data (input from the slaves)
#endif

///////////////////////////////////////* PARAMETER SETING */////////////////////////////////////

// EtherCAT
    bool ecatconfig_success = false;
    int chk;

// RT thread
    struct timespec trt;
    struct itimerspec timer_conf;
    struct timespec expected;

    long t1 = 0;
    long t2 = 0;
    long old_t1 = 0;
    long delta_t1 = 0;
    long t_before_sample_start = 0;
    double sampling_ms = 0;
    int tfd;

    uint32_t overrun = 0;
    uint64_t ticks;


///////////////////////////////////////* EherCAT MASTER */////////////////////////////////////

// 1. Initialize EtherCAT Master(Init)

    if(ecat_init(IFNAME)) //If there is any error, ecat_init returns '0'
        //There is no error with ecat_init
        sigRTthreadKill = 0;

    else
        //There is error
        sigRTthreadKill = 1;

// 2. EtherCAT slave number check

    #ifdef NON_SLAVE_TS
    if(ec_slavecount == NUMOFSLAVES)
    {
        sigRTthreadKill = 0;
        //for(int i=1;i<=NUMOFSLAVES-1;i++)
    #ifdef SLAVE_GTWI                                          // SLAVE pdomapping setting /////////////////////////////////////
            for(int i=1;i<=NUMOF_GTWI_SLAVES;i++)
            {
                //PO: Pre-Operation, SO: Safe-Operation
                //Link slave's specific setups to PO -> SO
                ec_slave[i].PO2SOconfig = ecat_PDO_Config_GTWI; //Doesn't this function need argument?
                                                           //=> PO2SOconfig is also function.
            }
    #endif
    #ifdef SLAVE_PTWI
            for(int i=NUMOF_GTWI_SLAVES+1;i<=NUMOF_GTWI_SLAVES+NUMOF_PTWI_SLAVES;i++)
            {
                //PO: Pre-Operation, SO: Safe-Operation
                //Link slave's specific setups to PO -> SO
                ec_slave[i].PO2SOconfig = ecat_PDO_Config_PTWI; //Doesn't this function need argument?
                                                           //=> PO2SOconfig is also function.
            }
    #endif

    }
    else
        sigRTthreadKill = 1;
    #endif


// 3. PDO mapping

    ec_config_overlap_map(&IOmap); //Map all PDOs from slaves to IOmap with Outputs/Inputs in sequential order.
    //    ec_config_map(&IOmap);

// 4. Setting Distributed Clock

    int8 dc = ec_configdc(); //Returns ecx_configdc(ecx_contextt *context)=>returns boolean if slaves are found with DC

// 5. Change all slaves pre-OP to SafeOP.

    //ec_statecheck(slave number(0:all slaves), Requested state, Timeout value in microsec)
    ec_statecheck(0, EC_STATE_SAFE_OP, 4*EC_TIMEOUTSTATE); //EC_TIMEOUTSTATE=2,000,000 us

// 6. Calculate expected WKC

    expectedWKC = (ec_group[0].outputsWKC *2) + ec_group[0].inputsWKC; //Calculate WKCs

// 7. Change MASTER state to operational

    ec_slave[0].state = EC_STATE_OPERATIONAL;

// 8. Send one valid process data(Execute PDO communication once)

    ec_send_overlap_processdata(); //Send processdata to slave.
    //    ec_send_processdata();
    wkc = ec_receive_processdata(EC_TIMEOUTRET); //Receive processdata from slaves.


// 9. Request OP state for all slaves

    ec_writestate(0); // Write slave state.

    // 10. PDO data Receive/Transmit

    uint32 obytes = ec_slave[1].Obytes; // Obytes: Output bytes
    uint32 ibytes = ec_slave[1].Ibytes; // Ibytes: Input bytes

    for (int i = 0; i < NUMOFSLAVES; i++) // 0 does not mean master in here
    {
        if (i < NUMOF_GTWI_SLAVES) {
#ifdef SLAVE_GTWI
                // The reason why ec_slave[]has i+1 not i, because ec_slave[0] represents master
                out_twitter_GTWI[i] = (output_GTWI_t *)ec_slave[i + 1].outputs;
                in_twitter_GTWI[i] = (input_GTWI_t *)ec_slave[i + 1].inputs;
#endif
        } else {
#ifdef SLAVE_PTWI
                out_twitter_PTWI[i] = (output_PTWI_t *)ec_slave[i + 1].outputs;
                in_twitter_PTWI[i] = (input_PTWI_t *)ec_slave[i + 1].inputs;
#endif
        }
    }
#ifdef SLAVE_TS
    in_twitter_ts[0] = (input_TS_t *)ec_slave[1].inputs;
#endif

    // 11. Real time

    clock_gettime(CLOCK_MONOTONIC, &expected); //get the system's current time

    tfd = timerfd_create(CLOCK_MONOTONIC, 0); //create timer descriptor
    if(tfd == -1) error(1, errno, "timerfd_create()");

    timer_conf.it_value = expected; // from now
    timer_conf.it_interval.tv_sec = 0;
    timer_conf.it_interval.tv_nsec = RT_PERIOD_MS*1000000; //interval with RT_PERIOD_MS

    int err = timerfd_settime(tfd, TFD_TIMER_ABSTIME, &timer_conf, NULL); //set the timer descriptor
    if(err) error(1, errno, "timerfd_setting()");

    usleep(1000);

///////////////////////////////////////* REALTIME LOOP */////////////////////////////////////

    while(!sigRTthreadKill)
    {
    // 11-1
        clock_gettime(CLOCK_REALTIME, &trt); //get the system time
        t_before_sample_start = trt.tv_nsec;

        err = read(tfd, &ticks, sizeof(ticks)); //read timer and return when the defined interval is reached (periodic)
        clock_gettime(CLOCK_REALTIME, &trt); //get the system time
        if(err<0) error(1, errno, "read()"); //

        old_t1 = t1;
        t1 = trt.tv_nsec;
        if(old_t1>trt.tv_nsec)
        {
            delta_t1 = (t1+1000000000) - old_t1;
        }
        else
        {
            delta_t1 = t1 - old_t1;
        }

        sampling_ms = (double)delta_t1*0.000001; //calculating time interval

        double jitter = sampling_ms - RT_PERIOD_MS; //calculating jitter

        if(ticks>1) overrun += ticks - 1; //calculating total overrun

    // 11-2. Sending processdata (calculated one tick before) => In order to ensure punctuality

        ec_send_overlap_processdata(); //PDO sending
        // ec_send_processdata();

        wkc = ec_receive_processdata(EC_TIMEOUTRET); //returns WKC

        if(expectedWKC>wkc)
        //In case of checked wkc less than WKC that have to be right
        //This means the etherCAT frame cannot be successfully read or
        //wrote on at least one slaves.
        //Up to user
        {
            sigRTthreadKill = 1; //Kill the entire of the RT thread
        }





    // 11-4. Control loop
//////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////* Control loop start *///////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////

        t++;
        /****************** Mutex Exchange ******************/

        K_RL.exchange_mutex();
        RLHIP.exchange_mutex();
        RLKNEE.exchange_mutex();

        // Delay DATA set

        K_RL.set_DelayDATA();

        // Data receive from ELMO
        RLHIP.DATA_Receive(in_twitter_GTWI);
        RLKNEE.DATA_Receive(in_twitter_GTWI);

        /****************** Kinematics ******************/

        J_RL = K_RL.get_RW_Jacobian();
        JTrans_RL = K_RL.get_RW_Jacobian_Trans();

        /****************** Trajectory ******************/

        // c.trajectory_generation(0.001 * t);

        /****************** State ******************/
        posRW = K_RL.get_posRW(); // function의 마지막 input이 0이면 현재 값, 1이면 이전 값
        posRW_err = K_RL.get_posRW_error(0);
        posRW_err_old = K_RL.get_posRW_error(1);

        /****************** Conrtoller ******************/
        RL_output[0] = c.posPID(posRW_err, posRW_err_old, 0, 0); // R방향 force
        RL_output[1] = c.posPID(posRW_err, posRW_err_old, 1, 0); // thteta 방향 force

        /****************** Put the torque in Motor ******************/
        RL_control_input = JTrans_RL * RL_output; // biarticular ipnut torque vector

        /****************** Data send to ELMO ******************/
        RLHIP.DATA_Send(out_twitter_GTWI);
        RLKNEE.DATA_Send(out_twitter_GTWI);

        /****************** Mutex exchange ******************/
        RLHIP.exchange_mutex();
        RLKNEE.exchange_mutex();

        //
        //
        //
        //
        //
        //
        //
        //

        // 11-6. Sync data with GUI thread

        // mutex_trylock : 
        if(!pthread_mutex_trylock(&data_mut))
        {
            _M_sampling_time_ms = sampling_ms; //when the thread get the mutex, write data into shared global variables
            _M_overrun_cnt = overrun;

            _M_Ecat_WKC = wkc;
            _M_Ecat_expectedWKC = expectedWKC;
            _M_motor_torque[0] = RL_control_input[1] - RL_control_input[0]; // theta 1
            _M_motor_torque[1] = RL_control_input[0]; // theta 2

            pthread_mutex_unlock(&data_mut);
        }
        t2 = trt.tv_nsec;

    }
    pthread_exit(NULL);
    return NULL;

    //////////////////////////////////////////////////////////////////////////////////////////////////

}
