/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.15.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QTabWidget *tabWidget;
    QWidget *initialize;
    QCheckBox *checkBox;
    QCheckBox *checkBox_2;
    QCheckBox *checkBox_3;
    QCheckBox *checkBox_4;
    QLabel *label;
    QCheckBox *checkBox_5;
    QCheckBox *checkBox_6;
    QLabel *label_3;
    QLabel *label_4;
    QTabWidget *tabWidget_2;
    QWidget *tab;
    QPushButton *Controlword7;
    QPushButton *Controlword6;
    QPushButton *Controlword15;
    QPushButton *Controlword14;
    QPushButton *Controlword128;
    QWidget *tab_2;
    QWidget *Mode_Select;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(1900, 1000);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        tabWidget = new QTabWidget(centralwidget);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        tabWidget->setEnabled(true);
        tabWidget->setGeometry(QRect(-290, -100, 1871, 961));
        tabWidget->setUsesScrollButtons(true);
        initialize = new QWidget();
        initialize->setObjectName(QString::fromUtf8("initialize"));
        checkBox = new QCheckBox(initialize);
        checkBox->setObjectName(QString::fromUtf8("checkBox"));
        checkBox->setGeometry(QRect(40, 100, 151, 20));
        QFont font;
        font.setPointSize(14);
        checkBox->setFont(font);
        checkBox_2 = new QCheckBox(initialize);
        checkBox_2->setObjectName(QString::fromUtf8("checkBox_2"));
        checkBox_2->setGeometry(QRect(40, 130, 141, 20));
        checkBox_2->setFont(font);
        checkBox_3 = new QCheckBox(initialize);
        checkBox_3->setObjectName(QString::fromUtf8("checkBox_3"));
        checkBox_3->setGeometry(QRect(40, 160, 151, 20));
        checkBox_3->setFont(font);
        checkBox_4 = new QCheckBox(initialize);
        checkBox_4->setObjectName(QString::fromUtf8("checkBox_4"));
        checkBox_4->setGeometry(QRect(40, 190, 161, 20));
        checkBox_4->setFont(font);
        label = new QLabel(initialize);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(80, 60, 91, 21));
        QFont font1;
        font1.setPointSize(18);
        label->setFont(font1);
        checkBox_5 = new QCheckBox(initialize);
        checkBox_5->setObjectName(QString::fromUtf8("checkBox_5"));
        checkBox_5->setGeometry(QRect(40, 220, 161, 20));
        checkBox_5->setFont(font);
        checkBox_6 = new QCheckBox(initialize);
        checkBox_6->setObjectName(QString::fromUtf8("checkBox_6"));
        checkBox_6->setGeometry(QRect(30, 360, 161, 20));
        checkBox_6->setFont(font);
        label_3 = new QLabel(initialize);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(60, 320, 91, 21));
        label_3->setFont(font1);
        label_4 = new QLabel(initialize);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setGeometry(QRect(750, 50, 221, 31));
        label_4->setMinimumSize(QSize(0, 21));
        tabWidget_2 = new QTabWidget(initialize);
        tabWidget_2->setObjectName(QString::fromUtf8("tabWidget_2"));
        tabWidget_2->setGeometry(QRect(340, 80, 1311, 531));
        tab = new QWidget();
        tab->setObjectName(QString::fromUtf8("tab"));
        Controlword7 = new QPushButton(tab);
        Controlword7->setObjectName(QString::fromUtf8("Controlword7"));
        Controlword7->setGeometry(QRect(10, 80, 291, 22));
        Controlword6 = new QPushButton(tab);
        Controlword6->setObjectName(QString::fromUtf8("Controlword6"));
        Controlword6->setGeometry(QRect(10, 50, 291, 22));
        Controlword15 = new QPushButton(tab);
        Controlword15->setObjectName(QString::fromUtf8("Controlword15"));
        Controlword15->setGeometry(QRect(10, 140, 291, 22));
        Controlword14 = new QPushButton(tab);
        Controlword14->setObjectName(QString::fromUtf8("Controlword14"));
        Controlword14->setGeometry(QRect(10, 110, 291, 22));
        Controlword128 = new QPushButton(tab);
        Controlword128->setObjectName(QString::fromUtf8("Controlword128"));
        Controlword128->setGeometry(QRect(10, 20, 291, 21));
        tabWidget_2->addTab(tab, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName(QString::fromUtf8("tab_2"));
        tabWidget_2->addTab(tab_2, QString());
        tabWidget->addTab(initialize, QString());
        Mode_Select = new QWidget();
        Mode_Select->setObjectName(QString::fromUtf8("Mode_Select"));
        tabWidget->addTab(Mode_Select, QString());
        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 1900, 19));
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        MainWindow->setStatusBar(statusbar);

        retranslateUi(MainWindow);

        tabWidget->setCurrentIndex(0);
        tabWidget_2->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "MainWindow", nullptr));
        checkBox->setText(QCoreApplication::translate("MainWindow", "Elmo Enabled", nullptr));
        checkBox_2->setText(QCoreApplication::translate("MainWindow", "Motor angle", nullptr));
        checkBox_3->setText(QCoreApplication::translate("MainWindow", "Spring angle", nullptr));
        checkBox_4->setText(QCoreApplication::translate("MainWindow", "Torque sensor", nullptr));
        label->setText(QCoreApplication::translate("MainWindow", "CHECK", nullptr));
        checkBox_5->setText(QCoreApplication::translate("MainWindow", "HOMING", nullptr));
        checkBox_6->setText(QCoreApplication::translate("MainWindow", "Torque", nullptr));
        label_3->setText(QCoreApplication::translate("MainWindow", "SAFETY", nullptr));
        label_4->setText(QCoreApplication::translate("MainWindow", "<html><head/><body><p><span style=\" font-size:16pt; font-weight:700;\">Mode of operation</span></p></body></html>", nullptr));
        Controlword7->setText(QCoreApplication::translate("MainWindow", "3. Switch On (0b0000'0111, 0x07, 7)", nullptr));
        Controlword6->setText(QCoreApplication::translate("MainWindow", "2. Shutdown (0b0000'0110, 0x06, 6)", nullptr));
        Controlword15->setText(QCoreApplication::translate("MainWindow", "5. Enable (15)", nullptr));
        Controlword14->setText(QCoreApplication::translate("MainWindow", "4. Shutdown (0b0000'1110, 0x0E, 14)", nullptr));
        Controlword128->setText(QCoreApplication::translate("MainWindow", "1. Disable Voltage (0b1000'0000, 0x80, 128))", nullptr));
        tabWidget_2->setTabText(tabWidget_2->indexOf(tab), QCoreApplication::translate("MainWindow", "Init", nullptr));
        tabWidget_2->setTabText(tabWidget_2->indexOf(tab_2), QCoreApplication::translate("MainWindow", "Tab 2", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(initialize), QCoreApplication::translate("MainWindow", "INITIALIZE", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(Mode_Select), QCoreApplication::translate("MainWindow", "MODE SELECT", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
