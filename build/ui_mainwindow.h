/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.15.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QFrame>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QFrame *frame;
    QPushButton *BTN_RESTART;
    QLabel *LB_INPUT_STATE;
    QFrame *BTN_RESTART_SHADOW;
    QFrame *BTN_EXIT_SHADOW;
    QPushButton *BTN_MINIMIZE;
    QFrame *line;
    QFrame *FRAME_DEVICE;
    QPushButton *BTN_EXIT;
    QFrame *BTN_MINIMIZE_SHADOW;
    QFrame *line_2;
    QLabel *LB_STS;
    QLabel *MESSAGE_LB;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(1920, 1100);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        frame = new QFrame(centralwidget);
        frame->setObjectName(QString::fromUtf8("frame"));
        frame->setGeometry(QRect(380, 53, 1081, 653));
        frame->setAutoFillBackground(true);
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setFrameShadow(QFrame::Raised);
        BTN_RESTART = new QPushButton(centralwidget);
        BTN_RESTART->setObjectName(QString::fromUtf8("BTN_RESTART"));
        BTN_RESTART->setGeometry(QRect(117, 976, 85, 38));
        BTN_RESTART->setAutoFillBackground(false);
        BTN_RESTART->setStyleSheet(QString::fromUtf8("QPushButton{font:12pt;\n"
"								background-color: #3bd466;\n"
"                            border-radius: 10px;\n"
"                            border: 0px solid #595959;}"));
        LB_INPUT_STATE = new QLabel(centralwidget);
        LB_INPUT_STATE->setObjectName(QString::fromUtf8("LB_INPUT_STATE"));
        LB_INPUT_STATE->setGeometry(QRect(20, 780, 67, 17));
        BTN_RESTART_SHADOW = new QFrame(centralwidget);
        BTN_RESTART_SHADOW->setObjectName(QString::fromUtf8("BTN_RESTART_SHADOW"));
        BTN_RESTART_SHADOW->setGeometry(QRect(120, 980, 85, 38));
        BTN_RESTART_SHADOW->setStyleSheet(QString::fromUtf8("QFrame { \n"
"   border: 0px solid #595959 ;\n"
"   background-color: #B4B4B4;\n"
"   border-radius:10px;\n"
"} "));
        BTN_RESTART_SHADOW->setFrameShape(QFrame::StyledPanel);
        BTN_RESTART_SHADOW->setFrameShadow(QFrame::Raised);
        BTN_EXIT_SHADOW = new QFrame(centralwidget);
        BTN_EXIT_SHADOW->setObjectName(QString::fromUtf8("BTN_EXIT_SHADOW"));
        BTN_EXIT_SHADOW->setGeometry(QRect(22, 980, 85, 38));
        BTN_EXIT_SHADOW->setStyleSheet(QString::fromUtf8("QFrame { \n"
"   border: 0px solid #595959 ;\n"
"   background-color: #B4B4B4;\n"
"   border-radius:10px;\n"
"} "));
        BTN_EXIT_SHADOW->setFrameShape(QFrame::StyledPanel);
        BTN_EXIT_SHADOW->setFrameShadow(QFrame::Raised);
        BTN_MINIMIZE = new QPushButton(centralwidget);
        BTN_MINIMIZE->setObjectName(QString::fromUtf8("BTN_MINIMIZE"));
        BTN_MINIMIZE->setGeometry(QRect(216, 976, 85, 38));
        BTN_MINIMIZE->setStyleSheet(QString::fromUtf8("QPushButton{font:12pt;\n"
"								background-color: #be36eb;\n"
"                            border-radius: 10px;\n"
"                            border: 0px solid #595959;}"));
        line = new QFrame(centralwidget);
        line->setObjectName(QString::fromUtf8("line"));
        line->setGeometry(QRect(10, 740, 341, 16));
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);
        FRAME_DEVICE = new QFrame(centralwidget);
        FRAME_DEVICE->setObjectName(QString::fromUtf8("FRAME_DEVICE"));
        FRAME_DEVICE->setGeometry(QRect(10, 0, 345, 720));
        FRAME_DEVICE->setAutoFillBackground(true);
        FRAME_DEVICE->setFrameShape(QFrame::StyledPanel);
        FRAME_DEVICE->setFrameShadow(QFrame::Raised);
        BTN_EXIT = new QPushButton(centralwidget);
        BTN_EXIT->setObjectName(QString::fromUtf8("BTN_EXIT"));
        BTN_EXIT->setGeometry(QRect(19, 976, 85, 38));
        BTN_EXIT->setStyleSheet(QString::fromUtf8("QPushButton{font:12pt;\n"
"								background-color: #FF9999;\n"
"                            border-radius: 10px;\n"
"                            border: 0px solid #595959;}"));
        BTN_MINIMIZE_SHADOW = new QFrame(centralwidget);
        BTN_MINIMIZE_SHADOW->setObjectName(QString::fromUtf8("BTN_MINIMIZE_SHADOW"));
        BTN_MINIMIZE_SHADOW->setGeometry(QRect(218, 980, 85, 38));
        BTN_MINIMIZE_SHADOW->setStyleSheet(QString::fromUtf8("QFrame { \n"
"   border: 0px solid #595959 ;\n"
"   background-color: #B4B4B4;\n"
"   border-radius:10px;\n"
"} "));
        BTN_MINIMIZE_SHADOW->setFrameShape(QFrame::StyledPanel);
        BTN_MINIMIZE_SHADOW->setFrameShadow(QFrame::Raised);
        line_2 = new QFrame(centralwidget);
        line_2->setObjectName(QString::fromUtf8("line_2"));
        line_2->setGeometry(QRect(10, 920, 341, 20));
        line_2->setFrameShape(QFrame::HLine);
        line_2->setFrameShadow(QFrame::Sunken);
        LB_STS = new QLabel(centralwidget);
        LB_STS->setObjectName(QString::fromUtf8("LB_STS"));
        LB_STS->setGeometry(QRect(390, 870, 171, 41));
        QFont font;
        font.setPointSize(16);
        LB_STS->setFont(font);
        LB_STS->setStyleSheet(QString::fromUtf8("position:center;"));
        LB_STS->setAlignment(Qt::AlignCenter);
        MESSAGE_LB = new QLabel(centralwidget);
        MESSAGE_LB->setObjectName(QString::fromUtf8("MESSAGE_LB"));
        MESSAGE_LB->setGeometry(QRect(570, 870, 601, 41));
        MESSAGE_LB->setFont(font);
        MainWindow->setCentralWidget(centralwidget);
        BTN_RESTART_SHADOW->raise();
        BTN_MINIMIZE_SHADOW->raise();
        frame->raise();
        BTN_RESTART->raise();
        LB_INPUT_STATE->raise();
        BTN_EXIT_SHADOW->raise();
        BTN_MINIMIZE->raise();
        line->raise();
        FRAME_DEVICE->raise();
        BTN_EXIT->raise();
        line_2->raise();
        LB_STS->raise();
        MESSAGE_LB->raise();
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 1920, 22));
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        MainWindow->setStatusBar(statusbar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "MainWindow", nullptr));
        BTN_RESTART->setText(QCoreApplication::translate("MainWindow", "RESTART", nullptr));
        LB_INPUT_STATE->setText(QCoreApplication::translate("MainWindow", "INPUT", nullptr));
        BTN_MINIMIZE->setText(QCoreApplication::translate("MainWindow", "MINIMIZE", nullptr));
        BTN_EXIT->setText(QCoreApplication::translate("MainWindow", "CLOSE", nullptr));
        LB_STS->setText(QCoreApplication::translate("MainWindow", "*Import Notice:", nullptr));
        MESSAGE_LB->setText(QCoreApplication::translate("MainWindow", "Please don't turn of the camera during the robot is operating!", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
