#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <QObject>
#include <QTimer>
#include <QDebug>
#include "DialogRobot.h"
enum
{
    RO_FREE,
    RO_LAYER,
    RO_PROCESS,
    RO_PUSHPULL,
    RO_INPUT
};

class Scheduler : public QObject {
    Q_OBJECT

public:
//    static Scheduler& getInstance(DialogRobot* robot = nullptr)
//    {
//        static Scheduler instance(robot);
//        return instance;
//    }
    static Scheduler& getInstance()
    {
        static Scheduler instance;
        return instance;
    }
    void initializeRobot(DialogRobot* robot);

//    explicit Scheduler(DialogRobot* _robot);
    void startProcessing(int value);
    int getCurrentValue() const { return processValue; }
    void setLayerAndDetection(int layer, int detection);

    void handleButtonClick();
    QString keymotion;
    void clearStoredDetections();


signals:
    void processingComplete();
    void currentValueChanged(int value);

private:
    enum PushPullState {
        AS_LAYER_IDLE,
        AS_LAYER_PUSH,
        AS_LAYER_PULL,
        AS_LAYER_RETURN,
        AS_LAYER_DONE
    };

    enum YoloState{
        AS_YOLO_IDLE,
        AS_YOLO_PROCESS,
        AS_YOLO_RETURN,
        AS_YOLO_DONE
    };

    enum NoDetection{
        AS_DEPTH_IDLE,
        AS_DEPTH_CHECK,
        AS_DEPTH_RETURN,
        AS_DEPTH_DONE
    };

////    explicit Scheduler(DialogRobot* _robot);
//    ~Scheduler();
    //---------------------
    Scheduler();
    ~Scheduler();

    // Delete copy constructor and assignment operator
    Scheduler(const Scheduler&) = delete;
    Scheduler& operator=(const Scheduler&) = delete;
    //---------------------

    QTimer *timer;
    PushPullState pushpullState;
    YoloState yoloState;
    int processValue;
    bool shouldProcess;

    int counter;
    const int MAX_COUNT = 20;

    // Detection and layer info
    int currentLayer;
    int currentDetection;

    NoDetection noDetectionState;
    bool continueBTN;
    int ROBOT_OCCUPY;
    DialogRobot* robot;

    //
    bool isInitialized;

private slots:
    void onTimer();
};
#endif // SCHEDULER_H
