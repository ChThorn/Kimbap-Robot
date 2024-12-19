#include "Scheduler.h"

extern Logger *plog;

Scheduler::Scheduler()
  : QObject(nullptr)
  , robot(nullptr)
  , processValue(0)
  , shouldProcess(false)
  , currentLayer(0)
  , currentDetection(-1)
  , continueBTN(false)
  , isInitialized(false)
{
    pushpullState = AS_LAYER_IDLE;
    yoloState = AS_YOLO_IDLE;
    noDetectionState = AS_DEPTH_IDLE;
    ROBOT_OCCUPY = RO_FREE;

    counter = 0;
    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(onTimer()));
    timer->start(100);  // 100ms interval
    qDebug() << "Scheduler initialized";
}

Scheduler::~Scheduler()
{
    timer->stop();
    delete timer;
}

void Scheduler::handleButtonClick()
{
    continueBTN = true;
}

//void Scheduler::clearStoredDetections()
//{
//    LAYER_NUM_YOLO = -1;
//    CELL_YOLO = -1;
//}

void Scheduler::startProcessing(int value)
{
    if (value > 0)
    {  // Only set new value if it's positive
        processValue = value;
        qDebug() << "Scheduler starting new process with value:" << value;
    }
    shouldProcess = true;
    counter = 0;
    pushpullState = AS_LAYER_IDLE;  // Reset state
    yoloState = AS_YOLO_IDLE;
    noDetectionState = AS_DEPTH_IDLE;
    ROBOT_OCCUPY = RO_FREE;
    qDebug() << "Scheduler continuing with current value:" << processValue;
    // Log current state for debugging
    qDebug() << "Scheduler state at start - Value:" << processValue
             << "Layer:" << currentLayer
             << "Detection:" << currentDetection;
    emit currentValueChanged(processValue);
}

void Scheduler::setLayerAndDetection(int layer, int detection)
{
    if(ROBOT_OCCUPY != RO_FREE)
    {
        return;
    }
    currentLayer = layer;
    currentDetection = detection;
    qDebug() << "Set layer:" << layer << "detection:" << detection;
}

void Scheduler::initializeRobot(DialogRobot* _robot)
{
    if (!isInitialized && _robot != nullptr)
    {
        robot = _robot;
        isInitialized = true;
        qDebug() << "Robot initialized in Scheduler";
    }
}

void Scheduler::onTimer()
{
    if (!isInitialized || !robot)
    {
//        qDebug() << "Error: Robot pointer is null!";
        return;
    }

//    static int LAYER_NUM = -1;
    switch(pushpullState)
    {
        case AS_LAYER_IDLE:
        {
            if(ROBOT_OCCUPY != RO_FREE)
            {
                qDebug() << "Push-Pull IDLE: Robot occupied";
                break;
            }
//            if(robot->RobotMoving == true)
//                break;

            if (processValue >= 1 && shouldProcess && currentLayer != 0 && currentDetection == -1)
            {
                qDebug() << "Push-Pull Conditions: Value=" << processValue
                                 << "Layer=" << currentLayer
                                 << "Detection=" << currentDetection;
                qDebug() << "AS_IDLE: Conditions met, moving to PROCESS";
//                LAYER_NUM = currentLayer;
                ROBOT_OCCUPY = RO_PROCESS;
                pushpullState = AS_LAYER_PUSH;
            }
            break;
        }

        case AS_LAYER_PUSH:
        {
//            if(robot->RobotMoving == true)
//                break;

            qDebug() << "AS_LAYER_PUSH";
            if(currentLayer >=1 && currentLayer <=5)
            {
                keymotion = QString("PUSH_LAYER_%1").arg(currentLayer);
            }
            else
            {
                qDebug() << "Error (AS_LAYER_PUSH)" << currentLayer;
                ROBOT_OCCUPY = RO_FREE;
                pushpullState = AS_LAYER_IDLE;
                break;
            }
            qDebug() << keymotion;
//            robot->RobotMoving = true;
//            robot->MotionServerCommand(keymotion);
            pushpullState = AS_LAYER_PULL;
            break;
        }

        case AS_LAYER_PULL:
        {
//            if(robot->RobotMoving == true)
//                break;

            qDebug() << "AS_LAYER_PULL";
            if(currentLayer >=1 && currentLayer <= 4)
            {
                keymotion = QString("PULL_LAYER_%1").arg(currentLayer+1);
            }
            else
            {
                qDebug() << "Error (AS_LAYER_PULL) wrong this:" << currentLayer;
                ROBOT_OCCUPY = RO_FREE;
                pushpullState = AS_LAYER_IDLE;
                break;
            }
            qDebug() << keymotion;
//            robot->RobotMoving = true;
//            robot->MotionServerCommand(keymotion);
            emit currentValueChanged(processValue);
            pushpullState = AS_LAYER_RETURN;
            break;
//            counter++;
//            qDebug() << "Counting:" << counter;

//            if(counter < MAX_COUNT)
//            {
//                break;
//            }

//            qDebug() << "Counting complete!";
//            counter = 0;

//            processValue--;
//            emit currentValueChanged(processValue);
//            qDebug() << "Processing value decreased to:" << processValue;
//            pushpullState = AS_LAYER_RETURN;
//            break;
        }

        case AS_LAYER_RETURN:
        {
//            if(robot->RobotMoving == true)
//                break;

            qDebug() << "AS_LAYER_RETURN";
            pushpullState = AS_LAYER_DONE;
            break;
        }

        case AS_LAYER_DONE:
        {
//            if(robot->RobotMoving == true)
//                break;

            qDebug() << "AS_LAYER_DONE";
            shouldProcess = false;  // Stop processing temporarily
            ROBOT_OCCUPY = RO_FREE;
            pushpullState = AS_LAYER_IDLE;
//            LAYER_NUM = -1;
            emit processingComplete();
            break;
        }
    }

    // YOLO processing state machine
//    static int LAYER_NUM_YOLO = -1;
//    static int CELL_YOLO = -1;
    switch(yoloState)
    {
        case AS_YOLO_IDLE:
        {
            if(ROBOT_OCCUPY != RO_FREE)
            {
                qDebug() << "YOLO IDLE: Robot occupied";
                break;
            }

//            if(robot->RobotMoving == true)
//                break;

            if (processValue >= 1 && shouldProcess &&
                currentLayer != 0 && currentDetection != -1)
            {
                qDebug() << "YOLO Conditions: Value=" << processValue
                                 << "Layer=" << currentLayer
                                 << "Detection=" << currentDetection;
                qDebug() << "AS_IDLE_YOLO: YOLO conditions met, moving to PROCESS";
//                LAYER_NUM_YOLO = currentLayer;
//                CELL_YOLO = currentDetection;

                ROBOT_OCCUPY = RO_INPUT;
                yoloState = AS_YOLO_PROCESS;
            }
            break;
        }

        case AS_YOLO_PROCESS:
        {
//            if(robot->RobotMoving == true)
//                break;

            qDebug() << "AS_PROCESS_YOLO";
            if(currentLayer != -1 && currentDetection != -1)
            {
                keymotion = QString("LAYER_%1_CELL_%2").arg(currentLayer).arg(currentDetection);
            }
            else
            {
                qDebug() << "Error (AS_YOLO_PROCESS) wrong this: " << currentLayer;
                ROBOT_OCCUPY = RO_FREE;
                yoloState = AS_YOLO_IDLE;
                break;
            }
            qDebug()<<keymotion;
//            processValue--;
//            emit currentValueChanged(processValue);
//            robot->RobotMoving = true;
//            robot->MotionServerCommand(keymotion);

//            yoloState = AS_YOLO_RETURN;
//            break;

            counter++;
            qDebug() << "YOLO Counting:" << counter;

            if(counter < MAX_COUNT)
            {
                break;
            }

            qDebug() << "YOLO counting complete!";
            counter = 0;
            processValue--;
            emit currentValueChanged(processValue);
            qDebug() << "YOLO processing value decreased to:" << processValue;
//            robot->RobotMoving = true;
//            robot->MotionServerCommand(keymotion);
            yoloState = AS_YOLO_RETURN;
            break;
        }

        case AS_YOLO_RETURN:
        {
            if(robot->RobotMoving == true)
                break;

            qDebug() << "AS_RETURN_YOLO";
            yoloState = AS_YOLO_DONE;
            break;
        }

        case AS_YOLO_DONE:
        {
//            if(robot->RobotMoving == true)
//                break;

            qDebug() << "AS_DONE_YOLO";
            shouldProcess = false;
//            LAYER_NUM_YOLO = -1;
//            CELL_YOLO = -1;
            ROBOT_OCCUPY = RO_FREE;
            yoloState = AS_YOLO_IDLE;
            emit processingComplete();
            break;
        }
    }


//    static int CURRENTCHECKLAYER = -1;
//    static int CURRENTDETECTION = -1;
    switch(noDetectionState)
    {
        case AS_DEPTH_IDLE:
        {
            if(ROBOT_OCCUPY != RO_FREE)
                break;
//            if(robot->RobotMoving == true)
//                break;

            if (processValue >= 1 && shouldProcess && currentLayer == 0 && currentDetection == -1)
            {
                qDebug() << "AS_DEPTH_IDLE: No conditions met";
//                CURRENTCHECKLAYER = currentLayer;
//                CURRENTDETECTION = currentDetection;
                ROBOT_OCCUPY = RO_LAYER;
                noDetectionState = AS_DEPTH_CHECK;
                continueBTN = false;  // Reset button state
            }
            break;
        }

        case AS_DEPTH_CHECK:
        {
//            if(robot->RobotMoving == true)
//                break;

            qDebug() << "AS_DEPTH_CHECK: Waiting for button click";
            if (continueBTN && currentLayer == 0 && currentDetection == -1)
            {
                keymotion = "PULL_LAYER_1";
                qDebug() << keymotion;
                emit currentValueChanged(processValue);
                noDetectionState = AS_DEPTH_RETURN;
//                counter++;
//                qDebug() << "Processing after button click:" << counter;

//                if(counter < MAX_COUNT)
//                {
//                    break;
//                }

//                qDebug() << "No-detection processing complete!";
//                counter = 0;
//                processValue--;
//                emit currentValueChanged(processValue);
//                noDetectionState = AS_DEPTH_RETURN;
            }
//            robot->RobotMoving = true;
//            robot->MotionServerCommand(keymotion);
            break;
        }

        case AS_DEPTH_RETURN:
        {
//            if(robot->RobotMoving == true)
//                break;

            qDebug() << "AS_DEPTH_RETURN";
            noDetectionState = AS_DEPTH_DONE;
            break;
        }

        case AS_DEPTH_DONE:
        {
//            if(robot->RobotMoving == true)
//                break;
            qDebug() << "AS_DEPTH_DONE";
            shouldProcess = false;
//            CURRENTCHECKLAYER = -1;
//            CURRENTDETECTION = -1;
            ROBOT_OCCUPY = RO_FREE;
            noDetectionState = AS_DEPTH_IDLE;
            emit processingComplete();
            break;
        }
    }
}
