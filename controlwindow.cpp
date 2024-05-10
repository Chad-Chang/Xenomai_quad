#include "controlwindow.h"
#include "ui_controlwindow.h"
#include <data_mutex.h>

#include <QThread>
#include <QTimer>
#include <pthread.h>

QSharedPointer<QCPAxisTickerTime> timeTicker(new QCPAxisTickerTime);

Controlwindow::Controlwindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::Controlwindow) {
  ui->setupUi(this);

  // Plot setting
  createPlot(ui->RL_pos_plot);
  createPlot(ui->RLHIP_pos_plot);
  createPlot(ui->RLKNEE_pos_plot);
}

void Controlwindow::createPlot(QCustomPlot *plot) {
  plot->addGraph();
  plot->graph(0)->setPen(QPen(QColor(237, 237, 237)));
  plot->addGraph();
  plot->graph(1)->setPen(QPen(QColor(255, 246, 18)));
  plot->xAxis->setTicker(timeTicker);
  plot->axisRect()->setupFullAxesBox();
  connect(plot->xAxis, SIGNAL(rangeChanged(QCPRange)), plot->xAxis2, SLOT(setRange(QCPRange)));
  connect(plot->yAxis, SIGNAL(rangeChanged(QCPRange)), plot->yAxis2, SLOT(setRange(QCPRange)));
  plot->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignLeft | Qt::AlignTop);

  plot->xAxis->setBasePen(QPen(Qt::white, 1));
  plot->yAxis->setBasePen(QPen(Qt::white, 1));
  plot->xAxis->setTickPen(QPen(Qt::white, 1));
  plot->yAxis->setTickPen(QPen(Qt::white, 1));
  plot->xAxis->setSubTickPen(QPen(Qt::white, 1));
  plot->yAxis->setSubTickPen(QPen(Qt::white, 1));
  plot->xAxis->setTickLabelColor(Qt::white);
  plot->yAxis->setTickLabelColor(Qt::white);
  plot->xAxis->grid()->setPen(QPen(QColor(140, 140, 140), 1, Qt::DotLine));
  plot->yAxis->grid()->setPen(QPen(QColor(140, 140, 140), 1, Qt::DotLine));
  plot->xAxis->grid()->setSubGridPen(QPen(QColor(80, 80, 80), 1, Qt::DotLine));
  plot->yAxis->grid()->setSubGridPen(QPen(QColor(80, 80, 80), 1, Qt::DotLine));
  plot->xAxis->grid()->setSubGridVisible(true);
  plot->yAxis->grid()->setSubGridVisible(true);
  plot->xAxis->grid()->setZeroLinePen(Qt::NoPen);
  plot->yAxis->grid()->setZeroLinePen(Qt::NoPen);
  plot->setBackground(QColor(25, 35, 45));
  plot->axisRect()->setBackground(QColor(25, 35, 45));
}

Controlwindow::~Controlwindow()
{
    delete ui;
}

void Controlwindow::Mutexexchange()
{

    int ret = pthread_mutex_timedlock(&data_mut,&data_mut_lock_timeout);
    if (!ret) {
      for (int i = 0; i < 4; i++) {
        _M_RW_r_posPgain[i] = RW_r_posPgain[i];
        _M_RW_r_posDgain[i] = RW_r_posDgain[i];
        _M_RW_r_posD_cutoff[i] = RW_r_posD_cutoff[i];
        _M_RW_th_posPgain[i] = RW_th_posPgain[i];
        _M_RW_th_posDgain[i] = RW_th_posDgain[i];
        _M_RW_th_posD_cutoff[i] = RW_th_posD_cutoff[i];

        RW_r_pos[i] = _M_RW_r_pos[i];
        RW_th_pos[i] = _M_RW_th_pos[i];
      }
      // Mutex exchange

    } else {
      QString str_errCode;
      str_errCode.setNum(ret);
    }
    pthread_mutex_unlock(&data_mut);
}

//// Flag ////

void Controlwindow::on_Set_clicked() {

    /****************** Gain ******************/

    /**** r direction ****/
    RW_r_posPgain[0] = ui->FL_r_posPgain->value();
    RW_r_posDgain[0] = ui->FL_r_posDgain->value();
    RW_r_posPgain[1] = ui->FR_r_posPgain->value();
    RW_r_posDgain[1] = ui->FR_r_posDgain->value();
    RW_r_posPgain[2] = ui->RL_r_posPgain->value();
    RW_r_posDgain[2] = ui->RL_r_posDgain->value();
    RW_r_posPgain[3] = ui->RR_r_posPgain->value();
    RW_r_posDgain[3] = ui->RR_r_posDgain->value();

    /**** th direction ****/
    RW_th_posPgain[0] = ui->FL_th_posPgain->value();
    RW_th_posDgain[0] = ui->FL_th_posDgain->value();
    RW_th_posPgain[1] = ui->FR_th_posPgain->value();
    RW_th_posDgain[1] = ui->FR_th_posDgain->value();
    RW_th_posPgain[2] = ui->RL_th_posPgain->value();
    RW_th_posDgain[2] = ui->RL_th_posDgain->value();
    RW_th_posPgain[3] = ui->RR_th_posPgain->value();
    RW_th_posDgain[3] = ui->RR_th_posDgain->value();

    /****************** Cut off ******************/

    /**** r direction ****/
    RW_r_posD_cutoff[0] = ui->FL_r_posD_cutoff->value();
    RW_r_posD_cutoff[1] = ui->FR_r_posD_cutoff->value();
    RW_r_posD_cutoff[2] = ui->RL_r_posD_cutoff->value();
    RW_r_posD_cutoff[3] = ui->RR_r_posD_cutoff->value();

    /**** th direction ****/
    RW_th_posD_cutoff[0] = ui->FL_th_posD_cutoff->value();
    RW_th_posD_cutoff[1] = ui->FR_th_posD_cutoff->value();
    RW_th_posD_cutoff[2] = ui->RL_th_posD_cutoff->value();
    RW_th_posD_cutoff[3] = ui->RR_th_posD_cutoff->value();
}
