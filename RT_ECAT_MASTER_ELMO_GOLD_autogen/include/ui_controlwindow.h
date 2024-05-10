/********************************************************************************
** Form generated from reading UI file 'controlwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.15.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_CONTROLWINDOW_H
#define UI_CONTROLWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Controlwindow
{
public:
    QWidget *centralwidget;
    QTabWidget *Control;
    QWidget *EtherCAT;
    QWidget *tab_2;
    QWidget *tab;
    QPushButton *PosON_pushButton;
    QPushButton *VelON_pushButton;
    QPushButton *CurON_pushButton;
    QDoubleSpinBox *RL_posPgain;
    QLabel *label;
    QLabel *label_2;
    QLabel *label_3;
    QDoubleSpinBox *RL_posDgain;
    QLabel *label_4;
    QPushButton *Set;
    QStatusBar *statusbar;
    QMenuBar *menubar;

    void setupUi(QMainWindow *Controlwindow)
    {
        if (Controlwindow->objectName().isEmpty())
            Controlwindow->setObjectName(QString::fromUtf8("Controlwindow"));
        Controlwindow->resize(1235, 600);
        centralwidget = new QWidget(Controlwindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        Control = new QTabWidget(centralwidget);
        Control->setObjectName(QString::fromUtf8("Control"));
        Control->setGeometry(QRect(400, 300, 511, 271));
        EtherCAT = new QWidget();
        EtherCAT->setObjectName(QString::fromUtf8("EtherCAT"));
        QFont font;
        font.setPointSize(11);
        EtherCAT->setFont(font);
        Control->addTab(EtherCAT, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName(QString::fromUtf8("tab_2"));
        Control->addTab(tab_2, QString());
        tab = new QWidget();
        tab->setObjectName(QString::fromUtf8("tab"));
        Control->addTab(tab, QString());
        PosON_pushButton = new QPushButton(centralwidget);
        PosON_pushButton->setObjectName(QString::fromUtf8("PosON_pushButton"));
        PosON_pushButton->setGeometry(QRect(50, 20, 80, 22));
        VelON_pushButton = new QPushButton(centralwidget);
        VelON_pushButton->setObjectName(QString::fromUtf8("VelON_pushButton"));
        VelON_pushButton->setGeometry(QRect(140, 20, 80, 22));
        CurON_pushButton = new QPushButton(centralwidget);
        CurON_pushButton->setObjectName(QString::fromUtf8("CurON_pushButton"));
        CurON_pushButton->setGeometry(QRect(230, 20, 80, 22));
        RL_posPgain = new QDoubleSpinBox(centralwidget);
        RL_posPgain->setObjectName(QString::fromUtf8("RL_posPgain"));
        RL_posPgain->setGeometry(QRect(690, 80, 62, 23));
        label = new QLabel(centralwidget);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(650, 80, 31, 20));
        label_2 = new QLabel(centralwidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(700, 50, 41, 20));
        label_3 = new QLabel(centralwidget);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(770, 50, 41, 20));
        RL_posDgain = new QDoubleSpinBox(centralwidget);
        RL_posDgain->setObjectName(QString::fromUtf8("RL_posDgain"));
        RL_posDgain->setGeometry(QRect(770, 80, 62, 23));
        label_4 = new QLabel(centralwidget);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setGeometry(QRect(650, 20, 91, 16));
        Set = new QPushButton(centralwidget);
        Set->setObjectName(QString::fromUtf8("Set"));
        Set->setGeometry(QRect(550, 80, 80, 22));
        Controlwindow->setCentralWidget(centralwidget);
        statusbar = new QStatusBar(Controlwindow);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        Controlwindow->setStatusBar(statusbar);
        menubar = new QMenuBar(Controlwindow);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 1235, 19));
        Controlwindow->setMenuBar(menubar);

        retranslateUi(Controlwindow);

        Control->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(Controlwindow);
    } // setupUi

    void retranslateUi(QMainWindow *Controlwindow)
    {
        Controlwindow->setWindowTitle(QCoreApplication::translate("Controlwindow", "MainWindow", nullptr));
#if QT_CONFIG(whatsthis)
        EtherCAT->setWhatsThis(QCoreApplication::translate("Controlwindow", "<html><head/><body><p>EtherCAT</p></body></html>", nullptr));
#endif // QT_CONFIG(whatsthis)
        Control->setTabText(Control->indexOf(EtherCAT), QCoreApplication::translate("Controlwindow", "EtherCAT", nullptr));
        Control->setTabText(Control->indexOf(tab_2), QCoreApplication::translate("Controlwindow", "Tab 2", nullptr));
        Control->setTabText(Control->indexOf(tab), QCoreApplication::translate("Controlwindow", "Page", nullptr));
        PosON_pushButton->setText(QCoreApplication::translate("Controlwindow", "PosON", nullptr));
        VelON_pushButton->setText(QCoreApplication::translate("Controlwindow", "VelON", nullptr));
        CurON_pushButton->setText(QCoreApplication::translate("Controlwindow", "CurON", nullptr));
        label->setText(QCoreApplication::translate("Controlwindow", "RL", nullptr));
        label_2->setText(QCoreApplication::translate("Controlwindow", "P_gain", nullptr));
        label_3->setText(QCoreApplication::translate("Controlwindow", "D_gain", nullptr));
        label_4->setText(QCoreApplication::translate("Controlwindow", "Position PD", nullptr));
        Set->setText(QCoreApplication::translate("Controlwindow", "Set", nullptr));
    } // retranslateUi

};

namespace Ui {
    class Controlwindow: public Ui_Controlwindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_CONTROLWINDOW_H
