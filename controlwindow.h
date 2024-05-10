#ifndef CONTROLWINDOW_H
#define CONTROLWINDOW_H

#include <QMainWindow>
#include <data_mutex.h>

namespace Ui {
class Controlwindow;
}

class Controlwindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit Controlwindow(QWidget *parent = nullptr);
    void Mutexexchange();
    ~Controlwindow();
    void createPlot(QCustomPlot *plot);

  private slots:



    void on_Set_clicked();

  private:
    Ui::Controlwindow *ui;

    // Variable //
    Vector4d RW_r_posPgain{
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
    };
    Vector4d RW_th_posDgain{
        0,
    };
    Vector4d RW_th_posD_cutoff{
        0,
    };

    double RW_r_pos[4]{
        0,
    };
    double RW_th_pos[4]{
        0,
    };
    double Motor_pos[NUMOFSLAVES]{
        0,
    };

    // flag //
    int PosCtrlON = 0;
    int VelCtrlON = 0;
    int CurCtrlON = 0;
};

#endif // CONTROLWINDOW_H
