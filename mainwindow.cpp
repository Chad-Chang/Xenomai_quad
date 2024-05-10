#include "mainwindow.h"
#include "./ui_mainwindow.h"

#include <pthread.h>
#include "data_mutex.h"

#include <QTimer>
#include <QThread>


/*           UI variable         */
int8_t MODE_OF_OPERATION[NUMOFSLAVES];
double ref_pos[NUMOFSLAVES];
double ref_vel[NUMOFSLAVES];
double ref_current[NUMOFSLAVES];
double P_gain[NUMOFSLAVES];
double I_gain[NUMOFSLAVES];
double D_gain[NUMOFSLAVES];
double Target_torque[NUMOFSLAVES];
double CONTROLWORD[NUMOFSLAVES];

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
  ui->setupUi(this);

  data_mut_lock_timeout.tv_nsec = timeout_ns;
  data_mut_lock_timeout.tv_sec = 0;

  tim = new QTimer(this);
  connect(tim, SIGNAL(timeout()), this, SLOT(updateWindow()));
  tim->start(50);
}

void MainWindow::updateWindow()
{

    double motor_position[NUMOFSLAVES];
    double motor_torque[NUMOFSLAVES];
    double motor_velocity[NUMOFSLAVES];


    int ret = pthread_mutex_timedlock(&data_mut,&data_mut_lock_timeout);
    if (!ret) {
      //// Receive at GUI ////
      sampling_time_ms = _M_sampling_time_ms;
      overrun_cnt = _M_overrun_cnt;
      WKC = _M_Ecat_WKC;
      expectedWKC = _M_Ecat_expectedWKC;

      for (int i = 0; i < NUMOFSLAVES; i++) {

        //Motor State
        motor_position[i] = _M_motor_position[i];
        motor_torque[i] = _M_motor_torque[i];
        motor_velocity[i] = _M_motor_velocity[i];

        _M_CONTROLWORD[i] = CONTROLWORD[i];
      }

    } else {
      QString str_errCode;
      str_errCode.setNum(ret);
    }
    pthread_mutex_unlock(&data_mut);

    QString str_sampling_time_ms;
    QString str_overrun_cnt;
    QString str_WKC;
    QString str_expectedWKC;

    QString str_status_report;

    QString str_statusword;
    QString str_modeofoperation_disp;

    str_sampling_time_ms.clear();
    str_overrun_cnt.clear();
    str_WKC.clear();
    str_expectedWKC.clear();

    str_sampling_time_ms.setNum(sampling_time_ms,'f',4);
    str_sampling_time_ms.prepend("Real-time task: ");
    str_sampling_time_ms.append("\tms sampling with ");

    str_overrun_cnt.setNum(overrun_cnt);
    str_overrun_cnt.append("\ttimes overrun.\t\t");

    str_expectedWKC.setNum(expectedWKC);
    str_expectedWKC.prepend("EtherCAT data frame: Expected workcounter= ");
    str_expectedWKC.append(", ");
    str_WKC.setNum(WKC);
    str_WKC.prepend("Workcounter= ");

    str_status_report.clear();
    str_status_report.append(str_sampling_time_ms).append(str_overrun_cnt).append(str_expectedWKC).append(str_WKC);

    ui->statusbar->showMessage(str_status_report);
}

MainWindow::~MainWindow() { delete ui; }

void MainWindow::on_Controlword128_clicked() {
    for (int i = 0; i < NUMOFSLAVES; i++) {
      _M_MODE_OF_OPERATION[i] = 4;
      CONTROLWORD[i] = 128;
    }
}
void MainWindow::on_Controlword6_clicked() {
    for (int i = 0; i < NUMOFSLAVES; i++) {
        CONTROLWORD[i] = 6;
    }
}
void MainWindow::on_Controlword7_clicked() {
    for (int i = 0; i < NUMOFSLAVES; i++) {
        CONTROLWORD[i] = 7;
    }
}
void MainWindow::on_Controlword14_clicked() {
    for (int i = 0; i < NUMOFSLAVES; i++) {
        CONTROLWORD[i] = 14;
    }
}
void MainWindow::on_Controlword15_clicked() {
    for (int i = 0; i < NUMOFSLAVES; i++) {
        CONTROLWORD[i] = 15;
    }
}
