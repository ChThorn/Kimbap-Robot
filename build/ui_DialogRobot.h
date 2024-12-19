/********************************************************************************
** Form generated from reading UI file 'DialogRobot.ui'
**
** Created by: Qt User Interface Compiler version 5.15.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_DIALOGROBOT_H
#define UI_DIALOGROBOT_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QDialog>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>

QT_BEGIN_NAMESPACE

class Ui_DialogRobot
{
public:
    QGroupBox *GB_ROBOT;
    QPushButton *BTN_ROBOT_TASK_START;
    QFrame *line;
    QFrame *line_2;
    QFrame *line_3;
    QFrame *line_4;
    QPushButton *BTN_ROBOT_PAUSE;
    QPushButton *BTN_ROBOT_RESUME_PAUSED;
    QPushButton *BTN_ROBOT_RESUME_OUTCOL;
    QPushButton *BTN_ROBOT_HALT;
    QLabel *label;
    QLineEdit *LE_JNT_REF_1;
    QLineEdit *LE_JNT_REF_2;
    QLineEdit *LE_JNT_REF_3;
    QLineEdit *LE_JNT_REF_4;
    QLineEdit *LE_JNT_REF_5;
    QLineEdit *LE_JNT_REF_6;
    QLabel *label_2;
    QLabel *label_3;
    QLabel *label_4;
    QLabel *label_5;
    QLabel *label_6;
    QLabel *label_7;
    QLineEdit *LE_JNT_ENC_2;
    QLineEdit *LE_JNT_ENC_5;
    QLineEdit *LE_JNT_ENC_1;
    QLineEdit *LE_JNT_ENC_3;
    QLineEdit *LE_JNT_ENC_4;
    QLineEdit *LE_JNT_ENC_6;
    QLineEdit *LE_TCP_REF_Y;
    QLineEdit *LE_TCP_REF_RY;
    QLineEdit *LE_TCP_REF_X;
    QLineEdit *LE_TCP_REF_Z;
    QLineEdit *LE_TCP_REF_RX;
    QLineEdit *LE_TCP_REF_RZ;
    QLabel *label_8;
    QLabel *label_9;
    QLabel *label_10;
    QLabel *label_11;
    QLabel *label_12;
    QLabel *label_13;
    QLabel *label_14;
    QLabel *label_15;
    QLabel *label_16;
    QLabel *label_17;
    QLabel *label_18;
    QLabel *label_19;
    QLabel *label_20;
    QLabel *label_21;
    QLineEdit *LE_ROBOT_STATE_MOVING;
    QLineEdit *LE_ROBOT_STATE_IDLE;
    QLineEdit *LE_PG_MODE_REAL;
    QLineEdit *LE_PG_MODE_SIMULATION;
    QLineEdit *LE_ROBOT_STATUS_PAUSED;
    QLineEdit *LE_ROBOT_STATUS_SELF_COLLISION;
    QLineEdit *LE_ROBOT_STATUS_EXT_COLLISION;
    QLineEdit *LE_ROBOT_STATUS_TEACHING;
    QLineEdit *LE_ROBOT_STATUS_EMS;
    QLineEdit *LE_ROBOT_STATUS_SOS;
    QLineEdit *LE_ROBOT_INIT_BASE_ANGLE;
    QLineEdit *LE_ROBOT_INIT_POSITION;
    QPushButton *BTN_ROBOT_REAL;
    QPushButton *BTN_ROBOT_SIMUL;
    QLabel *label_25;
    QLabel *label_26;
    QFrame *frame;
    QRadioButton *RB_ROBOT_SPEED_NORMAL;
    QRadioButton *RB_ROBOT_SPEED_SLOW;
    QLabel *label_27;
    QLabel *label_28;
    QLineEdit *LE_ROBOT_INIT_STAT_1;
    QLineEdit *LE_ROBOT_INIT_STAT_2;
    QLineEdit *LE_ROBOT_INIT_STAT_3;
    QLineEdit *LE_ROBOT_INIT_STAT_4;
    QLineEdit *LE_ROBOT_CONNECTION_CMD;
    QLineEdit *LE_ROBOT_PRG_WORKING_STATUS;
    QLineEdit *LE_ROBOT_PRG_CONNECT;
    QLineEdit *LE_ROBOT_CONNECTION_DATA;
    QPushButton *BTN_ROBOT_TASK_STOP;
    QLabel *label_29;
    QLabel *label_30;
    QLabel *label_31;
    QLabel *label_32;
    QLabel *label_33;
    QLabel *label_34;
    QLabel *label_35;
    QLabel *label_36;
    QLabel *label_37;
    QFrame *BTN_ROBOT_TASK_STOP_SHADOW;
    QFrame *BTN_ROBOT_TASK_START_SHADOW;

    void setupUi(QDialog *DialogRobot)
    {
        if (DialogRobot->objectName().isEmpty())
            DialogRobot->setObjectName(QString::fromUtf8("DialogRobot"));
        DialogRobot->resize(345, 720);
        GB_ROBOT = new QGroupBox(DialogRobot);
        GB_ROBOT->setObjectName(QString::fromUtf8("GB_ROBOT"));
        GB_ROBOT->setGeometry(QRect(0, 0, 345, 720));
        QFont font;
        font.setFamily(QString::fromUtf8("\355\225\250\354\264\210\353\241\254\353\217\213\354\233\200"));
        font.setPointSize(20);
        font.setBold(false);
        font.setItalic(false);
        font.setWeight(50);
        GB_ROBOT->setFont(font);
        GB_ROBOT->setAutoFillBackground(false);
        GB_ROBOT->setStyleSheet(QString::fromUtf8("QGroupBox  {\n"
"    margin-top: 10px;\n"
"	font: 20pt \"\355\225\250\354\264\210\353\241\254\353\217\213\354\233\200\";\n"
"}\n"
"\n"
"QGroupBox::title  {\n"
"    subcontrol-origin: margin;\n"
"    subcontrol-position: top center;\n"
"    padding: 10px 8000px 10px 8000px;\n"
"    background-color: #595959;\n"
"    color: rgb(255, 255, 255);\n"
"}\n"
"QLabel {\n"
"	font: 10pt \"\355\225\250\354\264\210\353\241\254\353\217\213\354\233\200\";\n"
"}\n"
"QComboBox{\n"
"	font: 10pt \"\355\225\250\354\264\210\353\241\254\353\217\213\354\233\200\";\n"
"	background-color: #ffffff\n"
"}\n"
"QPushButton{\n"
"	font: 10pt \"\355\225\250\354\264\210\353\241\254\353\217\213\354\233\200\";\n"
"	background-color: #ffffff\n"
"}\n"
"QLineEdit{\n"
"	font: 10pt \"\355\225\250\354\264\210\353\241\254\353\217\213\354\233\200\";\n"
"	background-color: #ffffff\n"
"}\n"
"\n"
"\n"
"\n"
""));
        GB_ROBOT->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        BTN_ROBOT_TASK_START = new QPushButton(GB_ROBOT);
        BTN_ROBOT_TASK_START->setObjectName(QString::fromUtf8("BTN_ROBOT_TASK_START"));
        BTN_ROBOT_TASK_START->setGeometry(QRect(15, 178, 150, 25));
        QFont font1;
        font1.setFamily(QString::fromUtf8("\355\225\250\354\264\210\353\241\254\353\217\213\354\233\200"));
        font1.setPointSize(10);
        font1.setBold(true);
        font1.setItalic(false);
        font1.setWeight(75);
        BTN_ROBOT_TASK_START->setFont(font1);
        BTN_ROBOT_TASK_START->setStyleSheet(QString::fromUtf8("QPushButton{color:white;background-color:#ad44cd;border-radius:50px;font:bold;font-size:10pt;};"));
        line = new QFrame(GB_ROBOT);
        line->setObjectName(QString::fromUtf8("line"));
        line->setGeometry(QRect(0, 210, 341, 20));
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);
        line_2 = new QFrame(GB_ROBOT);
        line_2->setObjectName(QString::fromUtf8("line_2"));
        line_2->setGeometry(QRect(0, 120, 341, 20));
        line_2->setFrameShape(QFrame::HLine);
        line_2->setFrameShadow(QFrame::Sunken);
        line_3 = new QFrame(GB_ROBOT);
        line_3->setObjectName(QString::fromUtf8("line_3"));
        line_3->setGeometry(QRect(0, 280, 341, 20));
        line_3->setFrameShape(QFrame::HLine);
        line_3->setFrameShadow(QFrame::Sunken);
        line_4 = new QFrame(GB_ROBOT);
        line_4->setObjectName(QString::fromUtf8("line_4"));
        line_4->setGeometry(QRect(0, 535, 341, 20));
        line_4->setFrameShape(QFrame::HLine);
        line_4->setFrameShadow(QFrame::Sunken);
        BTN_ROBOT_PAUSE = new QPushButton(GB_ROBOT);
        BTN_ROBOT_PAUSE->setObjectName(QString::fromUtf8("BTN_ROBOT_PAUSE"));
        BTN_ROBOT_PAUSE->setGeometry(QRect(240, 409, 93, 25));
        BTN_ROBOT_PAUSE->setFont(font1);
        BTN_ROBOT_PAUSE->setAutoFillBackground(false);
        BTN_ROBOT_PAUSE->setStyleSheet(QString::fromUtf8("QPushButton {\n"
"	font: 10pt \"\355\225\250\354\264\210\353\241\254\353\217\213\354\233\200\";\n"
"	font:bold;\n"
"}"));
        BTN_ROBOT_PAUSE->setFlat(false);
        BTN_ROBOT_RESUME_PAUSED = new QPushButton(GB_ROBOT);
        BTN_ROBOT_RESUME_PAUSED->setObjectName(QString::fromUtf8("BTN_ROBOT_RESUME_PAUSED"));
        BTN_ROBOT_RESUME_PAUSED->setGeometry(QRect(240, 439, 93, 25));
        BTN_ROBOT_RESUME_PAUSED->setFont(font1);
        BTN_ROBOT_RESUME_PAUSED->setStyleSheet(QString::fromUtf8("QPushButton {\n"
"	font: 10pt \"\355\225\250\354\264\210\353\241\254\353\217\213\354\233\200\";\n"
"	font:bold;\n"
"}"));
        BTN_ROBOT_RESUME_OUTCOL = new QPushButton(GB_ROBOT);
        BTN_ROBOT_RESUME_OUTCOL->setObjectName(QString::fromUtf8("BTN_ROBOT_RESUME_OUTCOL"));
        BTN_ROBOT_RESUME_OUTCOL->setGeometry(QRect(240, 469, 93, 25));
        BTN_ROBOT_RESUME_OUTCOL->setFont(font1);
        BTN_ROBOT_RESUME_OUTCOL->setStyleSheet(QString::fromUtf8("QPushButton {\n"
"	font: 10pt \"\355\225\250\354\264\210\353\241\254\353\217\213\354\233\200\";\n"
"	font:bold;\n"
"}"));
        BTN_ROBOT_HALT = new QPushButton(GB_ROBOT);
        BTN_ROBOT_HALT->setObjectName(QString::fromUtf8("BTN_ROBOT_HALT"));
        BTN_ROBOT_HALT->setGeometry(QRect(240, 500, 93, 25));
        BTN_ROBOT_HALT->setFont(font1);
        BTN_ROBOT_HALT->setStyleSheet(QString::fromUtf8("QPushButton {\n"
"	font: 10pt \"\355\225\250\354\264\210\353\241\254\353\217\213\354\233\200\";\n"
"	font:bold;\n"
"}"));
        label = new QLabel(GB_ROBOT);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(149, 560, 67, 17));
        QFont font2;
        font2.setFamily(QString::fromUtf8("\355\225\250\354\264\210\353\241\254\353\217\213\354\233\200"));
        font2.setPointSize(10);
        font2.setBold(false);
        font2.setItalic(false);
        font2.setWeight(50);
        label->setFont(font2);
        LE_JNT_REF_1 = new QLineEdit(GB_ROBOT);
        LE_JNT_REF_1->setObjectName(QString::fromUtf8("LE_JNT_REF_1"));
        LE_JNT_REF_1->setGeometry(QRect(39, 578, 60, 21));
        LE_JNT_REF_1->setFont(font2);
        LE_JNT_REF_1->setAlignment(Qt::AlignCenter);
        LE_JNT_REF_2 = new QLineEdit(GB_ROBOT);
        LE_JNT_REF_2->setObjectName(QString::fromUtf8("LE_JNT_REF_2"));
        LE_JNT_REF_2->setGeometry(QRect(39, 599, 60, 21));
        LE_JNT_REF_2->setFont(font2);
        LE_JNT_REF_2->setAlignment(Qt::AlignCenter);
        LE_JNT_REF_3 = new QLineEdit(GB_ROBOT);
        LE_JNT_REF_3->setObjectName(QString::fromUtf8("LE_JNT_REF_3"));
        LE_JNT_REF_3->setGeometry(QRect(39, 620, 60, 21));
        LE_JNT_REF_3->setFont(font2);
        LE_JNT_REF_3->setAlignment(Qt::AlignCenter);
        LE_JNT_REF_4 = new QLineEdit(GB_ROBOT);
        LE_JNT_REF_4->setObjectName(QString::fromUtf8("LE_JNT_REF_4"));
        LE_JNT_REF_4->setGeometry(QRect(39, 641, 60, 21));
        LE_JNT_REF_4->setFont(font2);
        LE_JNT_REF_4->setAlignment(Qt::AlignCenter);
        LE_JNT_REF_5 = new QLineEdit(GB_ROBOT);
        LE_JNT_REF_5->setObjectName(QString::fromUtf8("LE_JNT_REF_5"));
        LE_JNT_REF_5->setGeometry(QRect(39, 662, 60, 21));
        LE_JNT_REF_5->setFont(font2);
        LE_JNT_REF_5->setAlignment(Qt::AlignCenter);
        LE_JNT_REF_6 = new QLineEdit(GB_ROBOT);
        LE_JNT_REF_6->setObjectName(QString::fromUtf8("LE_JNT_REF_6"));
        LE_JNT_REF_6->setGeometry(QRect(39, 683, 60, 21));
        LE_JNT_REF_6->setFont(font2);
        LE_JNT_REF_6->setAlignment(Qt::AlignCenter);
        label_2 = new QLabel(GB_ROBOT);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(20, 583, 31, 17));
        label_2->setFont(font2);
        label_3 = new QLabel(GB_ROBOT);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(20, 603, 31, 17));
        label_3->setFont(font2);
        label_4 = new QLabel(GB_ROBOT);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setGeometry(QRect(20, 623, 31, 17));
        label_4->setFont(font2);
        label_5 = new QLabel(GB_ROBOT);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setGeometry(QRect(20, 643, 31, 17));
        label_5->setFont(font2);
        label_6 = new QLabel(GB_ROBOT);
        label_6->setObjectName(QString::fromUtf8("label_6"));
        label_6->setGeometry(QRect(20, 663, 31, 17));
        label_6->setFont(font2);
        label_7 = new QLabel(GB_ROBOT);
        label_7->setObjectName(QString::fromUtf8("label_7"));
        label_7->setGeometry(QRect(20, 683, 31, 17));
        label_7->setFont(font2);
        LE_JNT_ENC_2 = new QLineEdit(GB_ROBOT);
        LE_JNT_ENC_2->setObjectName(QString::fromUtf8("LE_JNT_ENC_2"));
        LE_JNT_ENC_2->setGeometry(QRect(150, 599, 60, 21));
        LE_JNT_ENC_2->setFont(font2);
        LE_JNT_ENC_2->setAlignment(Qt::AlignCenter);
        LE_JNT_ENC_5 = new QLineEdit(GB_ROBOT);
        LE_JNT_ENC_5->setObjectName(QString::fromUtf8("LE_JNT_ENC_5"));
        LE_JNT_ENC_5->setGeometry(QRect(150, 662, 60, 21));
        LE_JNT_ENC_5->setFont(font2);
        LE_JNT_ENC_5->setAlignment(Qt::AlignCenter);
        LE_JNT_ENC_1 = new QLineEdit(GB_ROBOT);
        LE_JNT_ENC_1->setObjectName(QString::fromUtf8("LE_JNT_ENC_1"));
        LE_JNT_ENC_1->setGeometry(QRect(150, 578, 60, 21));
        LE_JNT_ENC_1->setFont(font2);
        LE_JNT_ENC_1->setAlignment(Qt::AlignCenter);
        LE_JNT_ENC_3 = new QLineEdit(GB_ROBOT);
        LE_JNT_ENC_3->setObjectName(QString::fromUtf8("LE_JNT_ENC_3"));
        LE_JNT_ENC_3->setGeometry(QRect(150, 620, 60, 21));
        LE_JNT_ENC_3->setFont(font2);
        LE_JNT_ENC_3->setAlignment(Qt::AlignCenter);
        LE_JNT_ENC_4 = new QLineEdit(GB_ROBOT);
        LE_JNT_ENC_4->setObjectName(QString::fromUtf8("LE_JNT_ENC_4"));
        LE_JNT_ENC_4->setGeometry(QRect(150, 641, 60, 21));
        LE_JNT_ENC_4->setFont(font2);
        LE_JNT_ENC_4->setAlignment(Qt::AlignCenter);
        LE_JNT_ENC_6 = new QLineEdit(GB_ROBOT);
        LE_JNT_ENC_6->setObjectName(QString::fromUtf8("LE_JNT_ENC_6"));
        LE_JNT_ENC_6->setGeometry(QRect(150, 683, 60, 21));
        LE_JNT_ENC_6->setFont(font2);
        LE_JNT_ENC_6->setAlignment(Qt::AlignCenter);
        LE_TCP_REF_Y = new QLineEdit(GB_ROBOT);
        LE_TCP_REF_Y->setObjectName(QString::fromUtf8("LE_TCP_REF_Y"));
        LE_TCP_REF_Y->setGeometry(QRect(260, 599, 60, 21));
        LE_TCP_REF_Y->setFont(font2);
        LE_TCP_REF_Y->setAlignment(Qt::AlignCenter);
        LE_TCP_REF_RY = new QLineEdit(GB_ROBOT);
        LE_TCP_REF_RY->setObjectName(QString::fromUtf8("LE_TCP_REF_RY"));
        LE_TCP_REF_RY->setGeometry(QRect(260, 662, 60, 21));
        LE_TCP_REF_RY->setFont(font2);
        LE_TCP_REF_RY->setAlignment(Qt::AlignCenter);
        LE_TCP_REF_X = new QLineEdit(GB_ROBOT);
        LE_TCP_REF_X->setObjectName(QString::fromUtf8("LE_TCP_REF_X"));
        LE_TCP_REF_X->setGeometry(QRect(260, 578, 60, 21));
        LE_TCP_REF_X->setFont(font2);
        LE_TCP_REF_X->setAlignment(Qt::AlignCenter);
        LE_TCP_REF_Z = new QLineEdit(GB_ROBOT);
        LE_TCP_REF_Z->setObjectName(QString::fromUtf8("LE_TCP_REF_Z"));
        LE_TCP_REF_Z->setGeometry(QRect(260, 620, 60, 21));
        LE_TCP_REF_Z->setFont(font2);
        LE_TCP_REF_Z->setAlignment(Qt::AlignCenter);
        LE_TCP_REF_RX = new QLineEdit(GB_ROBOT);
        LE_TCP_REF_RX->setObjectName(QString::fromUtf8("LE_TCP_REF_RX"));
        LE_TCP_REF_RX->setGeometry(QRect(260, 641, 60, 21));
        LE_TCP_REF_RX->setFont(font2);
        LE_TCP_REF_RX->setAlignment(Qt::AlignCenter);
        LE_TCP_REF_RZ = new QLineEdit(GB_ROBOT);
        LE_TCP_REF_RZ->setObjectName(QString::fromUtf8("LE_TCP_REF_RZ"));
        LE_TCP_REF_RZ->setGeometry(QRect(260, 683, 60, 21));
        LE_TCP_REF_RZ->setFont(font2);
        LE_TCP_REF_RZ->setAlignment(Qt::AlignCenter);
        label_8 = new QLabel(GB_ROBOT);
        label_8->setObjectName(QString::fromUtf8("label_8"));
        label_8->setGeometry(QRect(128, 623, 31, 17));
        label_8->setFont(font2);
        label_9 = new QLabel(GB_ROBOT);
        label_9->setObjectName(QString::fromUtf8("label_9"));
        label_9->setGeometry(QRect(128, 683, 31, 17));
        label_9->setFont(font2);
        label_10 = new QLabel(GB_ROBOT);
        label_10->setObjectName(QString::fromUtf8("label_10"));
        label_10->setGeometry(QRect(128, 583, 31, 17));
        label_10->setFont(font2);
        label_11 = new QLabel(GB_ROBOT);
        label_11->setObjectName(QString::fromUtf8("label_11"));
        label_11->setGeometry(QRect(128, 603, 31, 17));
        label_11->setFont(font2);
        label_12 = new QLabel(GB_ROBOT);
        label_12->setObjectName(QString::fromUtf8("label_12"));
        label_12->setGeometry(QRect(128, 643, 31, 17));
        label_12->setFont(font2);
        label_13 = new QLabel(GB_ROBOT);
        label_13->setObjectName(QString::fromUtf8("label_13"));
        label_13->setGeometry(QRect(128, 663, 31, 17));
        label_13->setFont(font2);
        label_14 = new QLabel(GB_ROBOT);
        label_14->setObjectName(QString::fromUtf8("label_14"));
        label_14->setGeometry(QRect(237, 623, 31, 17));
        label_14->setFont(font2);
        label_15 = new QLabel(GB_ROBOT);
        label_15->setObjectName(QString::fromUtf8("label_15"));
        label_15->setGeometry(QRect(237, 683, 31, 17));
        label_15->setFont(font2);
        label_16 = new QLabel(GB_ROBOT);
        label_16->setObjectName(QString::fromUtf8("label_16"));
        label_16->setGeometry(QRect(237, 583, 31, 17));
        label_16->setFont(font2);
        label_17 = new QLabel(GB_ROBOT);
        label_17->setObjectName(QString::fromUtf8("label_17"));
        label_17->setGeometry(QRect(237, 603, 31, 17));
        label_17->setFont(font2);
        label_18 = new QLabel(GB_ROBOT);
        label_18->setObjectName(QString::fromUtf8("label_18"));
        label_18->setGeometry(QRect(237, 643, 31, 17));
        label_18->setFont(font2);
        label_19 = new QLabel(GB_ROBOT);
        label_19->setObjectName(QString::fromUtf8("label_19"));
        label_19->setGeometry(QRect(237, 663, 31, 17));
        label_19->setFont(font2);
        label_20 = new QLabel(GB_ROBOT);
        label_20->setObjectName(QString::fromUtf8("label_20"));
        label_20->setGeometry(QRect(39, 560, 67, 17));
        label_20->setFont(font2);
        label_21 = new QLabel(GB_ROBOT);
        label_21->setObjectName(QString::fromUtf8("label_21"));
        label_21->setGeometry(QRect(261, 560, 71, 17));
        label_21->setFont(font2);
        LE_ROBOT_STATE_MOVING = new QLineEdit(GB_ROBOT);
        LE_ROBOT_STATE_MOVING->setObjectName(QString::fromUtf8("LE_ROBOT_STATE_MOVING"));
        LE_ROBOT_STATE_MOVING->setGeometry(QRect(15, 504, 91, 21));
        LE_ROBOT_STATE_MOVING->setFont(font2);
        LE_ROBOT_STATE_MOVING->setAlignment(Qt::AlignCenter);
        LE_ROBOT_STATE_IDLE = new QLineEdit(GB_ROBOT);
        LE_ROBOT_STATE_IDLE->setObjectName(QString::fromUtf8("LE_ROBOT_STATE_IDLE"));
        LE_ROBOT_STATE_IDLE->setGeometry(QRect(15, 483, 91, 21));
        LE_ROBOT_STATE_IDLE->setFont(font2);
        LE_ROBOT_STATE_IDLE->setAlignment(Qt::AlignCenter);
        LE_PG_MODE_REAL = new QLineEdit(GB_ROBOT);
        LE_PG_MODE_REAL->setObjectName(QString::fromUtf8("LE_PG_MODE_REAL"));
        LE_PG_MODE_REAL->setGeometry(QRect(15, 412, 61, 21));
        LE_PG_MODE_REAL->setFont(font2);
        LE_PG_MODE_REAL->setAlignment(Qt::AlignCenter);
        LE_PG_MODE_SIMULATION = new QLineEdit(GB_ROBOT);
        LE_PG_MODE_SIMULATION->setObjectName(QString::fromUtf8("LE_PG_MODE_SIMULATION"));
        LE_PG_MODE_SIMULATION->setGeometry(QRect(15, 433, 61, 21));
        LE_PG_MODE_SIMULATION->setFont(font2);
        LE_PG_MODE_SIMULATION->setAlignment(Qt::AlignCenter);
        LE_ROBOT_STATUS_PAUSED = new QLineEdit(GB_ROBOT);
        LE_ROBOT_STATUS_PAUSED->setObjectName(QString::fromUtf8("LE_ROBOT_STATUS_PAUSED"));
        LE_ROBOT_STATUS_PAUSED->setGeometry(QRect(170, 467, 61, 19));
        LE_ROBOT_STATUS_PAUSED->setFont(font2);
        LE_ROBOT_STATUS_PAUSED->setAlignment(Qt::AlignCenter);
        LE_ROBOT_STATUS_SELF_COLLISION = new QLineEdit(GB_ROBOT);
        LE_ROBOT_STATUS_SELF_COLLISION->setObjectName(QString::fromUtf8("LE_ROBOT_STATUS_SELF_COLLISION"));
        LE_ROBOT_STATUS_SELF_COLLISION->setGeometry(QRect(170, 448, 61, 19));
        LE_ROBOT_STATUS_SELF_COLLISION->setFont(font2);
        LE_ROBOT_STATUS_SELF_COLLISION->setAlignment(Qt::AlignCenter);
        LE_ROBOT_STATUS_EXT_COLLISION = new QLineEdit(GB_ROBOT);
        LE_ROBOT_STATUS_EXT_COLLISION->setObjectName(QString::fromUtf8("LE_ROBOT_STATUS_EXT_COLLISION"));
        LE_ROBOT_STATUS_EXT_COLLISION->setGeometry(QRect(170, 429, 61, 19));
        LE_ROBOT_STATUS_EXT_COLLISION->setFont(font2);
        LE_ROBOT_STATUS_EXT_COLLISION->setAlignment(Qt::AlignCenter);
        LE_ROBOT_STATUS_TEACHING = new QLineEdit(GB_ROBOT);
        LE_ROBOT_STATUS_TEACHING->setObjectName(QString::fromUtf8("LE_ROBOT_STATUS_TEACHING"));
        LE_ROBOT_STATUS_TEACHING->setGeometry(QRect(170, 410, 61, 19));
        LE_ROBOT_STATUS_TEACHING->setFont(font2);
        LE_ROBOT_STATUS_TEACHING->setAlignment(Qt::AlignCenter);
        LE_ROBOT_STATUS_EMS = new QLineEdit(GB_ROBOT);
        LE_ROBOT_STATUS_EMS->setObjectName(QString::fromUtf8("LE_ROBOT_STATUS_EMS"));
        LE_ROBOT_STATUS_EMS->setGeometry(QRect(170, 486, 61, 19));
        LE_ROBOT_STATUS_EMS->setFont(font2);
        LE_ROBOT_STATUS_EMS->setAlignment(Qt::AlignCenter);
        LE_ROBOT_STATUS_SOS = new QLineEdit(GB_ROBOT);
        LE_ROBOT_STATUS_SOS->setObjectName(QString::fromUtf8("LE_ROBOT_STATUS_SOS"));
        LE_ROBOT_STATUS_SOS->setGeometry(QRect(170, 505, 61, 19));
        LE_ROBOT_STATUS_SOS->setFont(font2);
        LE_ROBOT_STATUS_SOS->setAlignment(Qt::AlignCenter);
        LE_ROBOT_INIT_BASE_ANGLE = new QLineEdit(GB_ROBOT);
        LE_ROBOT_INIT_BASE_ANGLE->setObjectName(QString::fromUtf8("LE_ROBOT_INIT_BASE_ANGLE"));
        LE_ROBOT_INIT_BASE_ANGLE->setGeometry(QRect(130, 240, 63, 21));
        LE_ROBOT_INIT_BASE_ANGLE->setFont(font2);
        LE_ROBOT_INIT_BASE_ANGLE->setAlignment(Qt::AlignCenter);
        LE_ROBOT_INIT_POSITION = new QLineEdit(GB_ROBOT);
        LE_ROBOT_INIT_POSITION->setObjectName(QString::fromUtf8("LE_ROBOT_INIT_POSITION"));
        LE_ROBOT_INIT_POSITION->setGeometry(QRect(130, 260, 63, 21));
        LE_ROBOT_INIT_POSITION->setFont(font2);
        LE_ROBOT_INIT_POSITION->setAlignment(Qt::AlignCenter);
        BTN_ROBOT_REAL = new QPushButton(GB_ROBOT);
        BTN_ROBOT_REAL->setObjectName(QString::fromUtf8("BTN_ROBOT_REAL"));
        BTN_ROBOT_REAL->setGeometry(QRect(80, 407, 60, 25));
        BTN_ROBOT_REAL->setFont(font1);
        BTN_ROBOT_REAL->setStyleSheet(QString::fromUtf8("QPushButton {\n"
"	font: 10pt \"\355\225\250\354\264\210\353\241\254\353\217\213\354\233\200\";\n"
"	font:bold;\n"
"}"));
        BTN_ROBOT_SIMUL = new QPushButton(GB_ROBOT);
        BTN_ROBOT_SIMUL->setObjectName(QString::fromUtf8("BTN_ROBOT_SIMUL"));
        BTN_ROBOT_SIMUL->setGeometry(QRect(80, 432, 60, 25));
        BTN_ROBOT_SIMUL->setFont(font1);
        BTN_ROBOT_SIMUL->setStyleSheet(QString::fromUtf8("QPushButton {\n"
"	font: 10pt \"\355\225\250\354\264\210\353\241\254\353\217\213\354\233\200\";\n"
"	font:bold;\n"
"}"));
        label_25 = new QLabel(GB_ROBOT);
        label_25->setObjectName(QString::fromUtf8("label_25"));
        label_25->setGeometry(QRect(15, 463, 121, 17));
        label_25->setFont(font2);
        label_26 = new QLabel(GB_ROBOT);
        label_26->setObjectName(QString::fromUtf8("label_26"));
        label_26->setGeometry(QRect(15, 388, 121, 17));
        label_26->setFont(font2);
        frame = new QFrame(GB_ROBOT);
        frame->setObjectName(QString::fromUtf8("frame"));
        frame->setGeometry(QRect(120, 337, 201, 41));
        frame->setLayoutDirection(Qt::LeftToRight);
        frame->setAutoFillBackground(false);
        frame->setFrameShape(QFrame::WinPanel);
        frame->setFrameShadow(QFrame::Raised);
        RB_ROBOT_SPEED_NORMAL = new QRadioButton(frame);
        RB_ROBOT_SPEED_NORMAL->setObjectName(QString::fromUtf8("RB_ROBOT_SPEED_NORMAL"));
        RB_ROBOT_SPEED_NORMAL->setGeometry(QRect(10, 10, 81, 23));
        QFont font3;
        font3.setPointSize(11);
        font3.setBold(true);
        font3.setWeight(75);
        RB_ROBOT_SPEED_NORMAL->setFont(font3);
        RB_ROBOT_SPEED_NORMAL->setLayoutDirection(Qt::LeftToRight);
        RB_ROBOT_SPEED_NORMAL->setChecked(true);
        RB_ROBOT_SPEED_SLOW = new QRadioButton(frame);
        RB_ROBOT_SPEED_SLOW->setObjectName(QString::fromUtf8("RB_ROBOT_SPEED_SLOW"));
        RB_ROBOT_SPEED_SLOW->setGeometry(QRect(130, 10, 61, 23));
        RB_ROBOT_SPEED_SLOW->setFont(font3);
        label_27 = new QLabel(GB_ROBOT);
        label_27->setObjectName(QString::fromUtf8("label_27"));
        label_27->setGeometry(QRect(15, 350, 91, 17));
        label_27->setFont(font2);
        label_28 = new QLabel(GB_ROBOT);
        label_28->setObjectName(QString::fromUtf8("label_28"));
        label_28->setGeometry(QRect(15, 310, 101, 17));
        label_28->setFont(font2);
        LE_ROBOT_INIT_STAT_1 = new QLineEdit(GB_ROBOT);
        LE_ROBOT_INIT_STAT_1->setObjectName(QString::fromUtf8("LE_ROBOT_INIT_STAT_1"));
        LE_ROBOT_INIT_STAT_1->setGeometry(QRect(140, 308, 35, 21));
        LE_ROBOT_INIT_STAT_1->setFont(font2);
        LE_ROBOT_INIT_STAT_1->setAlignment(Qt::AlignCenter);
        LE_ROBOT_INIT_STAT_2 = new QLineEdit(GB_ROBOT);
        LE_ROBOT_INIT_STAT_2->setObjectName(QString::fromUtf8("LE_ROBOT_INIT_STAT_2"));
        LE_ROBOT_INIT_STAT_2->setGeometry(QRect(180, 308, 35, 21));
        LE_ROBOT_INIT_STAT_2->setFont(font2);
        LE_ROBOT_INIT_STAT_2->setAlignment(Qt::AlignCenter);
        LE_ROBOT_INIT_STAT_3 = new QLineEdit(GB_ROBOT);
        LE_ROBOT_INIT_STAT_3->setObjectName(QString::fromUtf8("LE_ROBOT_INIT_STAT_3"));
        LE_ROBOT_INIT_STAT_3->setGeometry(QRect(220, 308, 35, 21));
        LE_ROBOT_INIT_STAT_3->setFont(font2);
        LE_ROBOT_INIT_STAT_3->setAlignment(Qt::AlignCenter);
        LE_ROBOT_INIT_STAT_4 = new QLineEdit(GB_ROBOT);
        LE_ROBOT_INIT_STAT_4->setObjectName(QString::fromUtf8("LE_ROBOT_INIT_STAT_4"));
        LE_ROBOT_INIT_STAT_4->setGeometry(QRect(260, 308, 35, 21));
        LE_ROBOT_INIT_STAT_4->setFont(font2);
        LE_ROBOT_INIT_STAT_4->setAlignment(Qt::AlignCenter);
        LE_ROBOT_CONNECTION_CMD = new QLineEdit(GB_ROBOT);
        LE_ROBOT_CONNECTION_CMD->setObjectName(QString::fromUtf8("LE_ROBOT_CONNECTION_CMD"));
        LE_ROBOT_CONNECTION_CMD->setGeometry(QRect(190, 60, 61, 21));
        LE_ROBOT_CONNECTION_CMD->setFont(font2);
        LE_ROBOT_CONNECTION_CMD->setAlignment(Qt::AlignCenter);
        LE_ROBOT_PRG_WORKING_STATUS = new QLineEdit(GB_ROBOT);
        LE_ROBOT_PRG_WORKING_STATUS->setObjectName(QString::fromUtf8("LE_ROBOT_PRG_WORKING_STATUS"));
        LE_ROBOT_PRG_WORKING_STATUS->setGeometry(QRect(190, 80, 121, 21));
        LE_ROBOT_PRG_WORKING_STATUS->setFont(font2);
        LE_ROBOT_PRG_WORKING_STATUS->setAlignment(Qt::AlignCenter);
        LE_ROBOT_PRG_CONNECT = new QLineEdit(GB_ROBOT);
        LE_ROBOT_PRG_CONNECT->setObjectName(QString::fromUtf8("LE_ROBOT_PRG_CONNECT"));
        LE_ROBOT_PRG_CONNECT->setGeometry(QRect(190, 100, 121, 21));
        LE_ROBOT_PRG_CONNECT->setFont(font2);
        LE_ROBOT_PRG_CONNECT->setAlignment(Qt::AlignCenter);
        LE_ROBOT_CONNECTION_DATA = new QLineEdit(GB_ROBOT);
        LE_ROBOT_CONNECTION_DATA->setObjectName(QString::fromUtf8("LE_ROBOT_CONNECTION_DATA"));
        LE_ROBOT_CONNECTION_DATA->setGeometry(QRect(250, 60, 61, 21));
        LE_ROBOT_CONNECTION_DATA->setFont(font2);
        LE_ROBOT_CONNECTION_DATA->setAlignment(Qt::AlignCenter);
        BTN_ROBOT_TASK_STOP = new QPushButton(GB_ROBOT);
        BTN_ROBOT_TASK_STOP->setObjectName(QString::fromUtf8("BTN_ROBOT_TASK_STOP"));
        BTN_ROBOT_TASK_STOP->setGeometry(QRect(220, 170, 85, 41));
        QFont font4;
        font4.setFamily(QString::fromUtf8("\355\225\250\354\264\210\353\241\254\353\217\213\354\233\200"));
        font4.setPointSize(12);
        font4.setBold(true);
        font4.setItalic(false);
        font4.setWeight(75);
        BTN_ROBOT_TASK_STOP->setFont(font4);
        BTN_ROBOT_TASK_STOP->setStyleSheet(QString::fromUtf8("QPushButton{color:white;background-color:#d10d23;border-radius:15px;font:bold;font-size:12pt};"));
        label_29 = new QLabel(GB_ROBOT);
        label_29->setObjectName(QString::fromUtf8("label_29"));
        label_29->setGeometry(QRect(205, 243, 131, 17));
        label_29->setFont(font2);
        label_30 = new QLabel(GB_ROBOT);
        label_30->setObjectName(QString::fromUtf8("label_30"));
        label_30->setGeometry(QRect(212, 263, 111, 17));
        label_30->setFont(font2);
        label_31 = new QLabel(GB_ROBOT);
        label_31->setObjectName(QString::fromUtf8("label_31"));
        label_31->setGeometry(QRect(15, 263, 121, 17));
        label_31->setFont(font2);
        label_32 = new QLabel(GB_ROBOT);
        label_32->setObjectName(QString::fromUtf8("label_32"));
        label_32->setGeometry(QRect(15, 243, 121, 17));
        label_32->setFont(font2);
        label_33 = new QLabel(GB_ROBOT);
        label_33->setObjectName(QString::fromUtf8("label_33"));
        label_33->setGeometry(QRect(15, 60, 151, 17));
        label_33->setFont(font2);
        label_34 = new QLabel(GB_ROBOT);
        label_34->setObjectName(QString::fromUtf8("label_34"));
        label_34->setGeometry(QRect(15, 80, 151, 17));
        label_34->setFont(font2);
        label_35 = new QLabel(GB_ROBOT);
        label_35->setObjectName(QString::fromUtf8("label_35"));
        label_35->setGeometry(QRect(15, 100, 151, 17));
        label_35->setFont(font2);
        label_36 = new QLabel(GB_ROBOT);
        label_36->setObjectName(QString::fromUtf8("label_36"));
        label_36->setGeometry(QRect(170, 388, 81, 17));
        label_36->setFont(font2);
        label_37 = new QLabel(GB_ROBOT);
        label_37->setObjectName(QString::fromUtf8("label_37"));
        label_37->setGeometry(QRect(100, 140, 131, 20));
        QFont font5;
        font5.setFamily(QString::fromUtf8("\355\225\250\354\264\210\353\241\254\353\217\213\354\233\200"));
        font5.setPointSize(12);
        font5.setBold(false);
        font5.setItalic(false);
        font5.setWeight(50);
        label_37->setFont(font5);
        label_37->setAutoFillBackground(false);
        label_37->setStyleSheet(QString::fromUtf8("QLabel {\n"
"	font: 12pt \"\355\225\250\354\264\210\353\241\254\353\217\213\354\233\200\";\n"
"	background-color:lightgreen;\n"
"}"));
        BTN_ROBOT_TASK_STOP_SHADOW = new QFrame(GB_ROBOT);
        BTN_ROBOT_TASK_STOP_SHADOW->setObjectName(QString::fromUtf8("BTN_ROBOT_TASK_STOP_SHADOW"));
        BTN_ROBOT_TASK_STOP_SHADOW->setGeometry(QRect(224, 174, 85, 41));
        BTN_ROBOT_TASK_STOP_SHADOW->setStyleSheet(QString::fromUtf8("QFrame { \n"
"   border: 0px solid #595959 ;\n"
"   background-color: #B4B4B4;\n"
"   border-radius:15px;\n"
"} "));
        BTN_ROBOT_TASK_STOP_SHADOW->setFrameShape(QFrame::StyledPanel);
        BTN_ROBOT_TASK_STOP_SHADOW->setFrameShadow(QFrame::Raised);
        BTN_ROBOT_TASK_START_SHADOW = new QFrame(GB_ROBOT);
        BTN_ROBOT_TASK_START_SHADOW->setObjectName(QString::fromUtf8("BTN_ROBOT_TASK_START_SHADOW"));
        BTN_ROBOT_TASK_START_SHADOW->setGeometry(QRect(18, 183, 150, 23));
        BTN_ROBOT_TASK_START_SHADOW->setStyleSheet(QString::fromUtf8("QFrame { \n"
"   border: 0px solid #595959 ;\n"
"   background-color: #B4B4B4;\n"
"   border-radius:40px;\n"
"} "));
        BTN_ROBOT_TASK_START_SHADOW->setFrameShape(QFrame::StyledPanel);
        BTN_ROBOT_TASK_START_SHADOW->setFrameShadow(QFrame::Raised);
        BTN_ROBOT_TASK_START_SHADOW->raise();
        BTN_ROBOT_TASK_STOP_SHADOW->raise();
        BTN_ROBOT_TASK_START->raise();
        line->raise();
        line_2->raise();
        line_3->raise();
        line_4->raise();
        BTN_ROBOT_PAUSE->raise();
        BTN_ROBOT_RESUME_PAUSED->raise();
        BTN_ROBOT_RESUME_OUTCOL->raise();
        BTN_ROBOT_HALT->raise();
        label->raise();
        LE_JNT_REF_1->raise();
        LE_JNT_REF_2->raise();
        LE_JNT_REF_3->raise();
        LE_JNT_REF_4->raise();
        LE_JNT_REF_5->raise();
        LE_JNT_REF_6->raise();
        label_2->raise();
        label_3->raise();
        label_4->raise();
        label_5->raise();
        label_6->raise();
        label_7->raise();
        LE_JNT_ENC_2->raise();
        LE_JNT_ENC_5->raise();
        LE_JNT_ENC_1->raise();
        LE_JNT_ENC_3->raise();
        LE_JNT_ENC_4->raise();
        LE_JNT_ENC_6->raise();
        LE_TCP_REF_Y->raise();
        LE_TCP_REF_RY->raise();
        LE_TCP_REF_X->raise();
        LE_TCP_REF_Z->raise();
        LE_TCP_REF_RX->raise();
        LE_TCP_REF_RZ->raise();
        label_8->raise();
        label_9->raise();
        label_10->raise();
        label_11->raise();
        label_12->raise();
        label_13->raise();
        label_14->raise();
        label_15->raise();
        label_16->raise();
        label_17->raise();
        label_18->raise();
        label_19->raise();
        label_20->raise();
        label_21->raise();
        LE_ROBOT_STATE_MOVING->raise();
        LE_ROBOT_STATE_IDLE->raise();
        LE_PG_MODE_REAL->raise();
        LE_PG_MODE_SIMULATION->raise();
        LE_ROBOT_STATUS_PAUSED->raise();
        LE_ROBOT_STATUS_SELF_COLLISION->raise();
        LE_ROBOT_STATUS_EXT_COLLISION->raise();
        LE_ROBOT_STATUS_TEACHING->raise();
        LE_ROBOT_STATUS_EMS->raise();
        LE_ROBOT_STATUS_SOS->raise();
        LE_ROBOT_INIT_BASE_ANGLE->raise();
        LE_ROBOT_INIT_POSITION->raise();
        BTN_ROBOT_REAL->raise();
        BTN_ROBOT_SIMUL->raise();
        label_25->raise();
        label_26->raise();
        frame->raise();
        label_27->raise();
        label_28->raise();
        LE_ROBOT_INIT_STAT_1->raise();
        LE_ROBOT_INIT_STAT_2->raise();
        LE_ROBOT_INIT_STAT_3->raise();
        LE_ROBOT_INIT_STAT_4->raise();
        LE_ROBOT_CONNECTION_CMD->raise();
        LE_ROBOT_PRG_WORKING_STATUS->raise();
        LE_ROBOT_PRG_CONNECT->raise();
        LE_ROBOT_CONNECTION_DATA->raise();
        BTN_ROBOT_TASK_STOP->raise();
        label_29->raise();
        label_30->raise();
        label_31->raise();
        label_32->raise();
        label_33->raise();
        label_34->raise();
        label_35->raise();
        label_36->raise();
        label_37->raise();

        retranslateUi(DialogRobot);

        QMetaObject::connectSlotsByName(DialogRobot);
    } // setupUi

    void retranslateUi(QDialog *DialogRobot)
    {
        DialogRobot->setWindowTitle(QCoreApplication::translate("DialogRobot", "Dialog", nullptr));
        GB_ROBOT->setTitle(QCoreApplication::translate("DialogRobot", "\353\241\234\353\264\207", nullptr));
        BTN_ROBOT_TASK_START->setText(QCoreApplication::translate("DialogRobot", "Click to turn ON", nullptr));
        BTN_ROBOT_PAUSE->setText(QCoreApplication::translate("DialogRobot", "Pause", nullptr));
        BTN_ROBOT_RESUME_PAUSED->setText(QCoreApplication::translate("DialogRobot", "Resume", nullptr));
        BTN_ROBOT_RESUME_OUTCOL->setText(QCoreApplication::translate("DialogRobot", "Resume Col.", nullptr));
        BTN_ROBOT_HALT->setText(QCoreApplication::translate("DialogRobot", "HALT", nullptr));
        label->setText(QCoreApplication::translate("DialogRobot", "Joint Enc.", nullptr));
        label_2->setText(QCoreApplication::translate("DialogRobot", "J0", nullptr));
        label_3->setText(QCoreApplication::translate("DialogRobot", "J1", nullptr));
        label_4->setText(QCoreApplication::translate("DialogRobot", "J2", nullptr));
        label_5->setText(QCoreApplication::translate("DialogRobot", "J3", nullptr));
        label_6->setText(QCoreApplication::translate("DialogRobot", "J4", nullptr));
        label_7->setText(QCoreApplication::translate("DialogRobot", "J5", nullptr));
        label_8->setText(QCoreApplication::translate("DialogRobot", "J2", nullptr));
        label_9->setText(QCoreApplication::translate("DialogRobot", "J5", nullptr));
        label_10->setText(QCoreApplication::translate("DialogRobot", "J0", nullptr));
        label_11->setText(QCoreApplication::translate("DialogRobot", "J1", nullptr));
        label_12->setText(QCoreApplication::translate("DialogRobot", "J3", nullptr));
        label_13->setText(QCoreApplication::translate("DialogRobot", "J4", nullptr));
        label_14->setText(QCoreApplication::translate("DialogRobot", "Z", nullptr));
        label_15->setText(QCoreApplication::translate("DialogRobot", "RZ", nullptr));
        label_16->setText(QCoreApplication::translate("DialogRobot", "X", nullptr));
        label_17->setText(QCoreApplication::translate("DialogRobot", "Y", nullptr));
        label_18->setText(QCoreApplication::translate("DialogRobot", "RX", nullptr));
        label_19->setText(QCoreApplication::translate("DialogRobot", "RY", nullptr));
        label_20->setText(QCoreApplication::translate("DialogRobot", "Joint Ref.", nullptr));
        label_21->setText(QCoreApplication::translate("DialogRobot", "Joint TCP", nullptr));
        LE_ROBOT_STATE_MOVING->setText(QCoreApplication::translate("DialogRobot", "In Operation", nullptr));
        LE_ROBOT_STATE_IDLE->setText(QCoreApplication::translate("DialogRobot", "On Standby", nullptr));
        LE_PG_MODE_REAL->setText(QCoreApplication::translate("DialogRobot", "Real", nullptr));
        LE_PG_MODE_SIMULATION->setText(QCoreApplication::translate("DialogRobot", "Sim", nullptr));
        LE_ROBOT_STATUS_PAUSED->setText(QCoreApplication::translate("DialogRobot", "Pause", nullptr));
        LE_ROBOT_STATUS_SELF_COLLISION->setText(QCoreApplication::translate("DialogRobot", "Self-Col.", nullptr));
        LE_ROBOT_STATUS_EXT_COLLISION->setText(QCoreApplication::translate("DialogRobot", "Collision", nullptr));
        LE_ROBOT_STATUS_TEACHING->setText(QCoreApplication::translate("DialogRobot", "Teach", nullptr));
        LE_ROBOT_STATUS_EMS->setText(QCoreApplication::translate("DialogRobot", "EMS", nullptr));
        LE_ROBOT_STATUS_SOS->setText(QCoreApplication::translate("DialogRobot", "SOS", nullptr));
        BTN_ROBOT_REAL->setText(QCoreApplication::translate("DialogRobot", "Real", nullptr));
        BTN_ROBOT_SIMUL->setText(QCoreApplication::translate("DialogRobot", "Sim", nullptr));
        label_25->setText(QCoreApplication::translate("DialogRobot", "Operation Status", nullptr));
        label_26->setText(QCoreApplication::translate("DialogRobot", "Operation Mode", nullptr));
        RB_ROBOT_SPEED_NORMAL->setText(QCoreApplication::translate("DialogRobot", "Normal", nullptr));
        RB_ROBOT_SPEED_SLOW->setText(QCoreApplication::translate("DialogRobot", "Slow", nullptr));
        label_27->setText(QCoreApplication::translate("DialogRobot", "Robot Speed", nullptr));
        label_28->setText(QCoreApplication::translate("DialogRobot", "Initialize Process", nullptr));
        LE_ROBOT_CONNECTION_CMD->setText(QCoreApplication::translate("DialogRobot", "CMD", nullptr));
        LE_ROBOT_CONNECTION_DATA->setText(QCoreApplication::translate("DialogRobot", "DATA", nullptr));
        BTN_ROBOT_TASK_STOP->setText(QCoreApplication::translate("DialogRobot", "STOP", nullptr));
        label_29->setText(QCoreApplication::translate("DialogRobot", "(-100 ~ -60degree)", nullptr));
        label_30->setText(QCoreApplication::translate("DialogRobot", "Foward Posture", nullptr));
        label_31->setText(QCoreApplication::translate("DialogRobot", "Position", nullptr));
        label_32->setText(QCoreApplication::translate("DialogRobot", "Initial Base Angle", nullptr));
        label_33->setText(QCoreApplication::translate("DialogRobot", "Connection Status", nullptr));
        label_34->setText(QCoreApplication::translate("DialogRobot", "Program Operation", nullptr));
        label_35->setText(QCoreApplication::translate("DialogRobot", "Communication Status", nullptr));
        label_36->setText(QCoreApplication::translate("DialogRobot", "Robot State", nullptr));
        label_37->setText(QCoreApplication::translate("DialogRobot", "Robot Operation", nullptr));
    } // retranslateUi

};

namespace Ui {
    class DialogRobot: public Ui_DialogRobot {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_DIALOGROBOT_H
