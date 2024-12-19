#ifndef DIALOGCAMERA_H
#define DIALOGCAMERA_H

#include <QDialog>
#include <QTimer>
#include "librealsense2/rs.hpp"
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QSpinBox>
#include <QQueue>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <QGraphicsTextItem>
#include "DialogDetector.h"

#define BUILD_PATHSET "/home/thornch/Desktop/Kimbap_15/build/SETTING"

struct ROIParams
{
    int x;
    int y;
    int width;
    int height;
    cv::Rect getRect() const
    {
        return cv::Rect(x, y, width, height);
    }
};

namespace Ui {
class DialogCamera;
}

class DialogCamera : public QDialog
{
    Q_OBJECT

public:
    explicit DialogCamera(QWidget *parent = nullptr);
    ~DialogCamera();

    bool isInProcessingState() const {return isProcessing;}

public slots:
    void onProcessingComplete();  // Called when Scheduler is done

private slots:
    void onTimer();
    void onCameraToggle();
    void onValueCapture();
    void onAddValue();
    void onClearValues();
    void onCurrentValueChanged(int value);

private:
    Ui::DialogCamera *ui;
    QTimer *timer;
    rs2::pipeline pipe;
    rs2::colorizer color_map;
    bool isRunning;
    bool isPaused;
    int processValue;
    QGraphicsScene *scene;
    QQueue<int> valueQueue;

    void resumeCamera();
    void pauseCamera();
    void initializeCamera();
    void stopCamera();
    QImage frameToQImage(const rs2::frame& f);
    QSpinBox *valueInput;
    bool isProcessing;  // Add this flag

    void acquireLiveFrame();
    void displayFrame(const QImage& frame, bool isPaused);
    QImage lastCapturedFrame;
    void updateValueDisplay();

    void addGridAndRectangle(const QImage& frame);
    void resetProcessingState();
    void updateOrderListDisplay();

    //-----------
    bool useCustomPoint;
    cv::Point frameCenter;
    cv::Point customPoint;
    float fullFrameDepth;
    cv::Mat depthColorMap;
    float currentDepth;
    rs2::align* align_to_color;

    float GetDepth();
    float getFullFrameDepth(const rs2::depth_frame& depth_frame);
    float getFullFrameDepth(const rs2::depth_frame& depth_frame, int x, int y);
    void resetToDefaultCenter();
    int clampValue(int value, int min, int max);
    void setCustomMeasurementPoint(int x, int y);
    void addDepthOverlay(cv::Mat& frame);
    cv::Mat convertFrameToMat(const rs2::frame& frame);
    void processingDepthData(const rs2::depth_frame& depth_frame);
    QImage matToQImage(const cv::Mat& mat);
    void processYoloDetection(const QImage& frame);

    DialogDetector detector;

    ROIParams getCurrentROI();
    int GetLayer();
    void drawDepthBasedROI(const QImage& frame);

    //----------------
    static constexpr float LAYER_1_DEPTH = 370.0f;
    static constexpr float LAYER_2_DEPTH = 500.0f;
    static constexpr float LAYER_3_DEPTH = 700.0f;
    static constexpr float LAYER_4_DEPTH = 950.0f;
    static constexpr float LAYER_5_DEPTH = 1200.0f;
    static constexpr float LAYER_6_DEPTH = 1400.0f;
    //----------------

    std::vector<int> currentDetections;  // Store detection results
    bool detectionsUpdated;              // Flag to track if detection has been done

    int getPriorityDetection(const std::vector<int>&detections);
    void updateSchedulerState();

    QPushButton* processButton;
    void updateProcessButton();
    void setupButtonStyles();

};

#endif // DIALOGCAMERA_H
