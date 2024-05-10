#ifndef NEWWINDOW_H
#define NEWWINDOW_H

#include<data_mutex.h>

#include <QMainWindow>
#include <QThread>
#include <QPixmap>
#include <algorithm>
#include <qcustomplot.h>
#include <stdio.h>

namespace Ui {
class NewWindow;
}

class NewWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit NewWindow(QWidget *parent = nullptr);
    ~NewWindow();

private slots:
    void updateWindow();
    void on_pushButton_clicked();


private:
    Ui::NewWindow *ui;
    double ref_pos[NUMOFSLAVES];
    double ref_vel[NUMOFSLAVES];
    double ref_current[NUMOFSLAVES];
};

#endif // NEWWINDOW_H
