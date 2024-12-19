#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "DialogCamera.h"
#include <QTimer>
#include <QDebug>
#include <QRect>

#include "DialogRobot.h"
#include "Scheduler.h"
#include "Logger.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    DialogRobot* robot;
    Scheduler* scheduler;
    QVector<DialogCamera*>      ptemp;

private:
    Ui::MainWindow *ui;
    QTimer *timer;

    QTime exit_time;
    QTime restart_time;
    QTime minimize_time;
    void changeTextColor();


private slots:
    void on_BTN_EXIT_released();
    void on_BTN_EXIT_pressed();
    void on_BTN_MINIMIZE_released();
    void on_BTN_MINIMIZE_pressed();
    void on_BTN_RESTART_pressed();
    void on_BTN_RESTART_released();

};
#endif // MAINWINDOW_H
