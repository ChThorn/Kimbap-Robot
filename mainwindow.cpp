#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QApplication>
#include <QPushButton>
#include <QFrame>
#include <QGraphicsDropShadowEffect>

int IS_UI_LOCK = false;
int OP_STATUS_OPERATING = PLATFORM_OPERATION_IDLE;

/*======Call App=====*/
Logger *plog;
QString appPath;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    /*=====Logger=====*/

    appPath = QApplication::applicationDirPath()+"/";

    plog->init();
    plog->write("Program Start");


    /*----------RobotDialog link connection--------*/
    robot = new DialogRobot(ui->FRAME_DEVICE);
    robot->setWindowFlags(Qt::Widget);
    robot->move(0, 0);

//    scheduler = new Scheduler(robot);
//    Scheduler::getInstance(robot); // Can be issue.
    Scheduler::getInstance().initializeRobot(robot);

    DialogCamera* temp = new DialogCamera(ui->frame);
    temp->setWindowFlags(Qt::Widget);
    temp->move(0,0);


    timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &MainWindow::changeTextColor);
    timer->start(500);

    showFullScreen();
}

MainWindow::~MainWindow()
{
    delete ui;
}

//void MainWindow::onTimer()


void MainWindow::on_BTN_EXIT_pressed()
{
    exit_time.restart();
    ui->BTN_EXIT->setStyleSheet("font:12pt;"
                                "background-color:red;"
                                "border-radius:10px;"
                                "border:0px solid #595959;");
}

void MainWindow::on_BTN_EXIT_released()
{
    if(exit_time.elapsed()>500)
    {
        QApplication::quit();
    }
    ui->BTN_EXIT->setStyleSheet("font:12pt;"
                                "background-color:#FF9999;"
                                "border-radius: 10px;"
                                "border: 0px solid #595959");
}

void MainWindow::on_BTN_MINIMIZE_pressed()
{
    minimize_time.restart();
    ui->BTN_MINIMIZE->setStyleSheet("font:12pt;"
                                    "background-color:#55FF55;"
                                    "border-radius: 10px;"
                                    "border: 0px solid #595959;");
}

void MainWindow::on_BTN_MINIMIZE_released()
{
    if(minimize_time.elapsed()>1000)
    {
//        plog->write("Program to minimized");
        showMinimized();
    }
    ui->BTN_MINIMIZE->setStyleSheet("font:12pt;"
                                    "background-color:#99FF99;"
                                    "border-radius:10px;"
                                    "border:0px solid #595959;");
}

void MainWindow::on_BTN_RESTART_pressed()
{
    restart_time.restart();
    ui->BTN_RESTART->setStyleSheet("font:12pt;"
                                   "background-color:#9f36eb;"
                                   "border-radius:10px;"
                                   "border:0px solid #595959;");
}

void MainWindow::on_BTN_RESTART_released()
{
    if(restart_time.elapsed()>1000)
    {
        plog->write("[ROBOT] Program Restart");
        QProcess::startDetached(QApplication::applicationDirPath());
        QApplication::exit(2000);
    }
    ui->BTN_RESTART->setStyleSheet("font:12pt;"
                                   "background-color:#3bd466;"
                                   "border-radius:10px;"
                                   "border: 0px solid #595959;");
}

void MainWindow::changeTextColor()
{
    static int colorIndex = 0;
    const QStringList colorList = {"#A0D468", "#4FC1E9", "#FFCE54", "#FC6E51", "#ED5565", "#AC92EC"};

    // Change text Color
    QString color = colorList.at(colorIndex);
    ui->LB_STS->setStyleSheet("QLabel {color: " + color + "; font-size: 22px; font:bold; }");
//    ui->LB_STS_1->setStyleSheet("QLabel {color: " + color + "; font-size: 16px; font:bold; }");

    //Create drope shadow effect
    QGraphicsDropShadowEffect *shadowEffect = new QGraphicsDropShadowEffect;
    shadowEffect->setColor(QColor(color));
    shadowEffect->setBlurRadius(70);
    shadowEffect->setOffset(7, 7);

    // Apply stylesheet and drop shadow effect to the label
//    ui->LB_STS->setStyleSheet(stylesheet);
    ui->LB_STS->setGraphicsEffect(shadowEffect);

    // Increment color index loop back to start if needed
    colorIndex = (colorIndex + 1)%colorList.size();
}
