/********************************************************************************
** Form generated from reading UI file 'DialogCamera.ui'
**
** Created by: Qt User Interface Compiler version 5.15.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_DIALOGCAMERA_H
#define UI_DIALOGCAMERA_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QDialog>
#include <QtWidgets/QGraphicsView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>

QT_BEGIN_NAMESPACE

class Ui_DialogCamera
{
public:
    QPushButton *START;
    QPushButton *ORDER;
    QLabel *label;
    QLabel *valueLabel;
    QGraphicsView *graphicsView;
    QPushButton *POSBTN;
    QPushButton *NEGBTN;
    QLabel *LB_ORDER;
    QPushButton *CONTINUE;
    QLabel *LIST;
    QLabel *LIST_2;

    void setupUi(QDialog *DialogCamera)
    {
        if (DialogCamera->objectName().isEmpty())
            DialogCamera->setObjectName(QString::fromUtf8("DialogCamera"));
        DialogCamera->resize(1081, 653);
        DialogCamera->setAutoFillBackground(true);
        START = new QPushButton(DialogCamera);
        START->setObjectName(QString::fromUtf8("START"));
        START->setGeometry(QRect(48, 540, 161, 51));
        ORDER = new QPushButton(DialogCamera);
        ORDER->setObjectName(QString::fromUtf8("ORDER"));
        ORDER->setGeometry(QRect(340, 540, 151, 51));
        label = new QLabel(DialogCamera);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(810, 440, 61, 31));
        valueLabel = new QLabel(DialogCamera);
        valueLabel->setObjectName(QString::fromUtf8("valueLabel"));
        valueLabel->setGeometry(QRect(890, 160, 181, 31));
        graphicsView = new QGraphicsView(DialogCamera);
        graphicsView->setObjectName(QString::fromUtf8("graphicsView"));
        graphicsView->setGeometry(QRect(20, 0, 640, 480));
        POSBTN = new QPushButton(DialogCamera);
        POSBTN->setObjectName(QString::fromUtf8("POSBTN"));
        POSBTN->setGeometry(QRect(810, 390, 61, 25));
        NEGBTN = new QPushButton(DialogCamera);
        NEGBTN->setObjectName(QString::fromUtf8("NEGBTN"));
        NEGBTN->setGeometry(QRect(810, 480, 61, 25));
        LB_ORDER = new QLabel(DialogCamera);
        LB_ORDER->setObjectName(QString::fromUtf8("LB_ORDER"));
        LB_ORDER->setGeometry(QRect(890, 110, 181, 31));
        CONTINUE = new QPushButton(DialogCamera);
        CONTINUE->setObjectName(QString::fromUtf8("CONTINUE"));
        CONTINUE->setGeometry(QRect(550, 550, 89, 25));
        LIST = new QLabel(DialogCamera);
        LIST->setObjectName(QString::fromUtf8("LIST"));
        LIST->setGeometry(QRect(700, 110, 171, 31));
        LIST_2 = new QLabel(DialogCamera);
        LIST_2->setObjectName(QString::fromUtf8("LIST_2"));
        LIST_2->setGeometry(QRect(700, 160, 171, 31));

        retranslateUi(DialogCamera);

        QMetaObject::connectSlotsByName(DialogCamera);
    } // setupUi

    void retranslateUi(QDialog *DialogCamera)
    {
        DialogCamera->setWindowTitle(QCoreApplication::translate("DialogCamera", "Dialog", nullptr));
        START->setText(QCoreApplication::translate("DialogCamera", "START", nullptr));
        ORDER->setText(QCoreApplication::translate("DialogCamera", "ORDER", nullptr));
        label->setText(QCoreApplication::translate("DialogCamera", "TextLabel", nullptr));
        valueLabel->setText(QCoreApplication::translate("DialogCamera", "TextLabel", nullptr));
        POSBTN->setText(QCoreApplication::translate("DialogCamera", "+", nullptr));
        NEGBTN->setText(QCoreApplication::translate("DialogCamera", "-", nullptr));
        LB_ORDER->setText(QString());
        CONTINUE->setText(QCoreApplication::translate("DialogCamera", "Continue", nullptr));
        LIST->setText(QCoreApplication::translate("DialogCamera", "Order List:", nullptr));
        LIST_2->setText(QCoreApplication::translate("DialogCamera", "Processing:", nullptr));
    } // retranslateUi

};

namespace Ui {
    class DialogCamera: public Ui_DialogCamera {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_DIALOGCAMERA_H
