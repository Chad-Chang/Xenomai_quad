#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QThread>
#include <QPixmap>
#include <algorithm>
#include <qcustomplot.h>
#include <stdio.h>
#include <data_mutex.h>

using namespace std;

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
    double sampling_time_ms = 0.0;
    int cnt_ID = 0;
    int overrun_cnt = 0;
    int WKC = 0;
    int expectedWKC = 0;


private slots:
  void updateWindow();
  void on_Controlword128_clicked();
  void on_Controlword6_clicked();
  void on_Controlword7_clicked();
  void on_Controlword14_clicked();
  void on_Controlword15_clicked();
};
#endif // MAINWINDOW_H
