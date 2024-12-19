#include "DialogCamera.h"
#include "ui_DialogCamera.h"
#include "Scheduler.h"
#include <QMessageBox>
#include <QDebug>

DialogCamera::DialogCamera(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DialogCamera),
    isRunning(false),
    isPaused(false),
    isProcessing(false),
    fullFrameDepth(0),
    useCustomPoint(false),
    currentDepth(0)
{
    ui->setupUi(this);

    setupButtonStyles();
    // Initialize YOLO model
    std::string modelConfig = std::string(BUILD_PATHSET) + "/yolov3.cfg";
    std::string modelWeights = std::string(BUILD_PATHSET) + "/yolov3.weights";
    detector.loadYOLOModel(modelConfig, modelWeights);

    align_to_color = new rs2::align(RS2_STREAM_COLOR);
    // Initialize camera center points
    frameCenter = cv::Point(500, 370);  // Default center (640/2, 480/2)

    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(onTimer()));
    timer->start(33); // ~30 fps

    scene = new QGraphicsScene(this);
    ui->graphicsView->setScene(scene);

    // Initialize label value to 0
    ui->label->setText("0");

    connect(ui->POSBTN, &QPushButton::clicked, [this]() {
        int currentValue = ui->label->text().toInt();
        ui->label->setText(QString::number(currentValue + 1));
    });

    // Connect NEGBTN (negative button) to decrement the label value
    connect(ui->NEGBTN, &QPushButton::clicked, [this]() {
        int currentValue = ui->label->text().toInt();
        if (currentValue > 0) {
            ui->label->setText(QString::number(currentValue - 1));
        } else {
            QMessageBox::warning(this, "Invalid Action", "Value cannot be negative.");
        }
    });

    connect(ui->START, &QPushButton::clicked, this, &DialogCamera::onCameraToggle);
    connect(ui->ORDER, &QPushButton::clicked, this, &DialogCamera::onValueCapture);
    connect(&Scheduler::getInstance(), &Scheduler::processingComplete,
            this, &DialogCamera::onProcessingComplete);

    connect(&Scheduler::getInstance(), &Scheduler::currentValueChanged,
                this, &DialogCamera::onCurrentValueChanged);
//    ui->valueLabel->setText("Current: 0");
    ui->valueLabel->setText("0");

    ui->LB_ORDER->setText("None");
    ui->LB_ORDER->setWordWrap(true);

    ui->START->setText("Start Camera");
    ui->ORDER->setText("Order");

    //
//    processButton = new QPushButton("Continue", this);
//    processButton->setVisible(false);
//    processButton->setEnabled(false);

    ui->CONTINUE->setEnabled(false);
//    processButton->setFixedSize(150, 40);
    ui->CONTINUE->setStyleSheet(
        "QPushButton {"
        "    background-color: #F3F4F6;"
        "    border-radius: 20px;"
        "    min-width: 40px;"
        "    min-height: 40px;"
        "    font: bold 16px;"
        "}"
        "QPushButton:enabled {"
        "    background-color: #4CAF50;"
        "    color: #db4291;"
        "}"
        "QPushButton:disabled {"
        "    background-color: #cccccc;"
        "    color: #666666;"
        "}"
        "QPushButton:hover:enabled {"
        "    background-color: #8abfa6;"
        "}"
        "QPushButton:pressed:enabled {"
        "    background-color: #35e8bb;"
        "}"
    );
//    ui->CONTINUE->setStyleSheet("QPushButton:enabled { background-color: #4CAF50; color: white; border-radius: 5px; }"
//                               "QPushButton:disabled { background-color: #cccccc; color: #666666; border-radius: 5px; }");
//    processButton->setStyleSheet("QPushButton { background-color: #4CAF50; color: white; border-radius: 5px; }");
    // Move button to a visible position
//    processButton->move(ui->graphicsView->x() + ui->graphicsView->width() - 160,  // 10px from right edge
//                       ui->graphicsView->y() + ui->graphicsView->height() + 10);   // 10px below graphicsView
    connect(ui->CONTINUE, &QPushButton::clicked, this, [this](){
        Scheduler::getInstance().handleButtonClick();
        ui->CONTINUE->setEnabled(false);
        qDebug() << "Process continue button clicked";
    });
}

DialogCamera::~DialogCamera()
{
    stopCamera();
    delete ui;
}

// Add this method to DialogCamera:
//void DialogCamera::updateProcessButton()
//{
//    if (isProcessing && Scheduler::getInstance().getCurrentValue() > 0)
//    {
//        int layer = GetLayer();
//        int detection = getPriorityDetection(currentDetections);

//        if (layer != 0 && detection == -1)
//        {
//            processButton->setVisible(true);
//            processButton->move(ui->graphicsView->x() + 10,
//                              ui->graphicsView->y() + ui->graphicsView->height() - 40);
//        }
//    }
//}

//void DialogCamera::updateProcessButton()
//{
//    if (isProcessing && Scheduler::getInstance().getCurrentValue() > 0)
//    {
//        int layer = GetLayer();
//        int detection = getPriorityDetection(currentDetections);

//        if (layer == 0 && detection == -1)
//        {
//            // Update position to ensure it's visible
////            processButton->move(ui->graphicsView->x() + ui->graphicsView->width() - 160,
////                              ui->graphicsView->y() + ui->graphicsView->height() + 10);
////            processButton->raise();  // Bring to front
////            processButton->setVisible(true);
//            ui->CONTINUE->setEnabled(true);
//            qDebug() << "Process continue button enabled";
//        }
//        else
//        {
////            processButton->setVisible(false);
//            ui->CONTINUE->setEnabled(false);
//        }
//    }
//    else
//    {
////        processButton->setVisible(false);
//        ui->CONTINUE->setEnabled(false);
//    }
//}

void DialogCamera::updateProcessButton()
{
    if (!isProcessing || Scheduler::getInstance().getCurrentValue() <= 0)
    {
        ui->CONTINUE->setEnabled(false);
        return;
    }

    int layer = GetLayer();
    int detection = getPriorityDetection(currentDetections);

    // Only enable continue button when in no-detection state (Layer 0)
    // AND not in YOLO detection state
    if (layer == 0 && detection == -1)
    {
        ui->CONTINUE->setEnabled(true);
        qDebug() << "Process continue button enabled";
    }
    else
    {
        ui->CONTINUE->setEnabled(false);
    }
}

void DialogCamera::onCurrentValueChanged(int value)
{
//    ui->valueLabel->setText(QString("Current: %1").arg(value));
    ui->valueLabel->setText(QString("%1").arg(value));
}

void DialogCamera::onAddValue()
{
//    int value = valueInput->value();
    int value = ui->label->text().toInt();

    if (value >= 1)
    {
        valueQueue.enqueue(value);
        updateValueDisplay();
    }
    else
    {
        QMessageBox::warning(this, "Invalid Value", "Please ensure the value is positive.");
    }
}

void DialogCamera::resetProcessingState()
{
    isProcessing = false;
    valueQueue.clear();
    updateValueDisplay();
    updateOrderListDisplay();
    ui->label->setText("0");
//    ui->valueLabel->setText("Current: 0");
    ui->valueLabel->setText("0");
    ui->LB_ORDER->setText("None");
    ui->CONTINUE->setEnabled(false);
}

void DialogCamera::updateOrderListDisplay()
{
    if (valueQueue.isEmpty()) {
        ui->LB_ORDER->setText("None");
        return;
    }

    QString display = "Orders: ";
    QQueue<int> tempQueue = valueQueue;  // Create a temporary copy

    QStringList valueStrings;
    while (!tempQueue.isEmpty()) {
        int value = tempQueue.dequeue();
        valueStrings.append(QString::number(value));
    }

    display += valueStrings.join(", ");
    ui->LB_ORDER->setText(display);
}

void DialogCamera::onClearValues()
{
    valueQueue.clear();
    updateValueDisplay();
}

void DialogCamera::updateValueDisplay()
{
    QString display = "Queue: ";
    QQueue<int> tempQueue = valueQueue;  // Create a temporary copy
    while (!tempQueue.isEmpty())
    {
        display += QString::number(tempQueue.dequeue());
        if (!tempQueue.isEmpty()) display += ", ";
    }
//    ui->LB_ORDER->setText(display);
    qDebug() << display;
}

void DialogCamera::resumeCamera()
{
    if (!isRunning) return;

    try {
        isPaused = false;
        detectionsUpdated = false;
        currentDetections.clear();
        lastCapturedFrame = QImage();  // Clear the stored frame
        pipe.start();
        qDebug() << "Camera resumed";
    } catch (const rs2::error& e) {
        QMessageBox::critical(this, "Error", "Failed to resume camera: " + QString(e.what()));
    }
}

void DialogCamera::pauseCamera()
{
    if (!isRunning) return;

    try {
        isPaused = true;
        // Make sure we have a frame before stopping
        detectionsUpdated = false;
        if (lastCapturedFrame.isNull())
        {
            rs2::frameset frames = pipe.wait_for_frames();
            auto color_frame = frames.get_color_frame();
            if (color_frame)
            {
                lastCapturedFrame = frameToQImage(color_frame);
            }
        }
        pipe.stop();
        qDebug() << "Camera paused";
    } catch (const rs2::error& e) {
        QMessageBox::critical(this, "Error", "Failed to pause camera: " + QString(e.what()));
    }
}

QImage DialogCamera::frameToQImage(const rs2::frame& f)
{
    auto width = f.as<rs2::video_frame>().get_width();
    auto height = f.as<rs2::video_frame>().get_height();
    auto data = f.as<rs2::video_frame>().get_data();

    return QImage((uchar*)data, width, height, QImage::Format_RGB888);
}

// Add depth overlay implementation:
void DialogCamera::addDepthOverlay(cv::Mat& frame)
{
    if(fullFrameDepth > 0)
    {
        // Draw measurement point
        cv::Scalar pointColor = useCustomPoint ?
            cv::Scalar(255, 165, 0) :  // Orange for custom point
            cv::Scalar(255, 0, 0);     // Blue for default center

        cv::circle(frame, frameCenter, 5, pointColor, -1);
        std::string depthText = cv::format("D: %.1f mm", fullFrameDepth);
        cv::putText(frame, depthText,
                    cv::Point(frameCenter.x - 60, frameCenter.y - 20),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, pointColor, 2);
    }
}

void DialogCamera::initializeCamera()
{
    try {
        rs2::config cfg;
        cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 30);
        cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
        pipe.start(cfg);
        isRunning = true;
        isPaused = false;
        ui->START->setText("Stop Camera");
        qDebug() << "Camera initialized";
    } catch (const rs2::error& e) {
        QMessageBox::critical(this, "Error", "Failed to start camera: " + QString(e.what()));
    }
}

void DialogCamera::stopCamera()
{
    if (isRunning) {
        try {
            pipe.stop();
            isRunning = false;
            isPaused = false;
            lastCapturedFrame = QImage();  // Clear the stored frame
            scene->clear();                // Clear the graphics scene
            ui->START->setText("Start Camera");
            resetProcessingState();
            qDebug() << "Camera stopped";
        } catch (const rs2::error& e) {
            QMessageBox::critical(this, "Error", "Failed to stop camera: " + QString(e.what()));
        }
    }
}

cv::Mat DialogCamera::convertFrameToMat(const rs2::frame& frame)
{
    return cv::Mat(
                cv::Size(frame.as<rs2::video_frame>().get_width(),
                         frame.as<rs2::video_frame>().get_height()),
                         CV_8UC3,
                         (void*)frame.as<rs2::video_frame>().get_data(),
                         cv::Mat::AUTO_STEP);
}

void DialogCamera::processingDepthData(const rs2::depth_frame &depth_frame)
{
    if(!useCustomPoint)
    {
        frameCenter = cv::Point(depth_frame.get_width()/2, depth_frame.get_height()/2);
    }
    fullFrameDepth = getFullFrameDepth(depth_frame, frameCenter.x, frameCenter.y);
}

QImage DialogCamera::matToQImage(const cv::Mat &mat)
{
    return QImage(
                mat.data,
                mat.cols,
                mat.rows,
                mat.step,
                QImage::Format_RGB888);
}

void DialogCamera::acquireLiveFrame()
{
    try {
        rs2::frameset frames = pipe.wait_for_frames();
        frames = align_to_color->process(frames);  // Align depth to color frame

        auto color_frame = frames.get_color_frame();
        auto depth_frame = frames.get_depth_frame();

        if (color_frame && depth_frame)  // Changed from || to &&
        {
            // Convert color frame to Mat for processing
            cv::Mat colorMat = convertFrameToMat(color_frame);

            // Process depth data
            processingDepthData(depth_frame);

            // Add depth overlay
            addDepthOverlay(colorMat);

            // Convert to QImage and display
            QImage processedFrame = matToQImage(colorMat);

            displayFrame(processedFrame, false);
            lastCapturedFrame = processedFrame;
        }
    } catch (const rs2::error& e) {
        qDebug() << "Frame acquisition error:" << e.what();
    }
}

void DialogCamera::displayFrame(const QImage& frame, bool isPaused)
{
    scene->clear();

    // Add the frame
    QPixmap pixmap = QPixmap::fromImage(frame);
    scene->addPixmap(pixmap);

    if (isPaused)
    {
        // Add a red rectangle when paused
        QPen pen(Qt::red);
        pen.setWidth(3);
        scene->addRect(QRectF(10, 10, frame.width() - 20, frame.height() - 20), pen);
        qDebug() << "Drawing rectangle on paused frame";

    }

    ui->graphicsView->fitInView(scene->sceneRect(), Qt::KeepAspectRatio);
}

void DialogCamera::addGridAndRectangle(const QImage& frame)
{
    // Clear the scene before adding elements
    scene->clear();

    QPixmap pixmap = QPixmap::fromImage(frame);
    scene->addPixmap(pixmap);

    // Add grid overlay
    int rows = 4, cols = 4; // Define grid dimensions
    QPen gridPen(Qt::blue);
    gridPen.setStyle(Qt::DotLine);

    for (int i = 1; i < rows; ++i) {
        int y = i * frame.height() / rows;
        scene->addLine(0, y, frame.width(), y, gridPen);
    }

    for (int i = 1; i < cols; ++i) {
        int x = i * frame.width() / cols;
        scene->addLine(x, 0, x, frame.height(), gridPen);
    }

    // Add red rectangle
    QPen rectPen(Qt::red);
    rectPen.setWidth(3);
    scene->addRect(QRectF(10, 10, frame.width() - 20, frame.height() - 20), rectPen);
}

void DialogCamera::processYoloDetection(const QImage& frame)
{
    // Convert QImage to cv::Mat
    cv::Mat cvFrame = cv::Mat(
        frame.height(),
        frame.width(),
        CV_8UC3,
        const_cast<uchar*>(frame.bits()),
        frame.bytesPerLine()
    );

    // Get ROI parameters and detect objects
    ROIParams params = getCurrentROI();
    cv::Rect roiRect = params.getRect();
    std::vector<int> detections = detector.detectObjectsInGridCells(cvFrame, roiRect, 2, 5);

    // Draw grid and detections
    detector.drawGridAndLabels(cvFrame, roiRect, 2, 5, detections);

    // Convert back to QImage and display
    QImage processedFrame(
        cvFrame.data,
        cvFrame.cols,
        cvFrame.rows,
        cvFrame.step,
        QImage::Format_RGB888
    );

    displayFrame(processedFrame, true);
}

//void DialogCamera::drawDepthBasedROI(const QImage& frame)
//{
//    scene->clear();
//    QPixmap pixmap = QPixmap::fromImage(frame);
//    scene->addPixmap(pixmap);

//    int currentLayer = GetLayer();
//    float currentDepth = GetDepth();  // Get current depth value

//    if(currentLayer > 0) {
//        ROIParams params = getCurrentROI();

//        // Draw ROI rectangle
//        QPen roiPen(Qt::green);
//        roiPen.setStyle(Qt::DotLine);
//        roiPen.setWidth(2);
//        scene->addRect(params.x, params.y, params.width, params.height, roiPen);

//        // Draw internal grid
//        QPen gridPen(Qt::blue);
//        gridPen.setStyle(Qt::DotLine);

//        int cellWidth = params.width / 5;
//        int cellHeight = params.height / 2;

//        // Vertical grid lines
//        for(int i = 1; i < 5; i++) {
//            int x = params.x + i * cellWidth;
//            scene->addLine(x, params.y, x, params.y + params.height, gridPen);
//        }

//        // Horizontal grid line
//        scene->addLine(params.x, params.y + cellHeight,
//                      params.x + params.width, params.y + cellHeight,
//                      gridPen);

//        // Fixed string formatting
//        QString depthInfo = QString("Layer: %1 (Depth: %2 mm)")
//                           .arg(currentLayer)
//                           .arg(currentDepth, 0, 'f', 1);  // Format with 1 decimal place

//        QGraphicsSimpleTextItem* textItem = scene->addSimpleText(depthInfo);
//        textItem->setBrush(QBrush(Qt::green));
//        textItem->setPos(10, 10);

//        // Debug output
//        qDebug() << "Layer:" << currentLayer << "Depth:" << currentDepth;
//    }

//    ui->graphicsView->fitInView(scene->sceneRect(), Qt::KeepAspectRatio);
//}

//void DialogCamera::drawDepthBasedROI(const QImage& frame)
//{
//    scene->clear();
//    QPixmap pixmap = QPixmap::fromImage(frame);
//    scene->addPixmap(pixmap);

//    int currentLayer = GetLayer();
//    float currentDepth = GetDepth();

//    if(currentLayer > 0) {
//        // Convert QImage to cv::Mat for YOLO processing
//        cv::Mat cvFrame = cv::Mat(
//            frame.height(),
//            frame.width(),
//            CV_8UC3,
//            const_cast<uchar*>(frame.bits()),
//            frame.bytesPerLine()
//        );

//        ROIParams params = getCurrentROI();

//        // Perform YOLO detection
//        std::vector<int> detections = detector.detectObjectsInGridCells(cvFrame, params.getRect(), 2, 5);

//        // Draw grid and detections using the existing detector function
//        detector.drawGridAndLabels(cvFrame, params.getRect(), 2, 5, detections);

//        // Convert back to QImage for display
//        QImage processedFrame(
//            cvFrame.data,
//            cvFrame.cols,
//            cvFrame.rows,
//            cvFrame.step,
//            QImage::Format_RGB888
//        );

//        // Update scene with processed frame
//        scene->clear();
//        scene->addPixmap(QPixmap::fromImage(processedFrame));

//        // Add depth information text
//        QString depthInfo = QString("Layer: %1 (Depth: %2 mm)")
//                           .arg(currentLayer)
//                           .arg(currentDepth, 0, 'f', 1);

//        QGraphicsSimpleTextItem* textItem = scene->addSimpleText(depthInfo);
//        textItem->setBrush(QBrush(Qt::green));
//        textItem->setPos(10, 10);

//        // Debug output
//        qDebug() << "Layer:" << currentLayer << "Depth:" << currentDepth;
//        qDebug() << "Detections:" << detections;
//    }

//    ui->graphicsView->fitInView(scene->sceneRect(), Qt::KeepAspectRatio);
//}

int DialogCamera::getPriorityDetection(const std::vector<int>& detections)
{
    // Find the first (lowest numbered) valid detection
    for(size_t i = 0; i < detections.size(); ++i) {
        if(detections[i] > 0) {
            return detections[i];
        }
    }
    return -1;  // Return -1 if no detection found
}

void DialogCamera::drawDepthBasedROI(const QImage& frame)
{
    scene->clear();
    QPixmap pixmap = QPixmap::fromImage(frame);
    scene->addPixmap(pixmap);

    int currentLayer = GetLayer();
    float currentDepth = GetDepth();

    if(currentLayer > 0) {
        // Convert QImage to cv::Mat for YOLO processing
        cv::Mat cvFrame = cv::Mat(
            frame.height(),
            frame.width(),
            CV_8UC3,
            const_cast<uchar*>(frame.bits()),
            frame.bytesPerLine()
        );

        ROIParams params = getCurrentROI();

        // Only perform detection once when paused
        if (!detectionsUpdated) {
            currentDetections = detector.detectObjectsInGridCells(cvFrame, params.getRect(), 2, 5); // rows = 2 and columns = 5
            detectionsUpdated = true;

//            updateSchedulerState();

            // Get and log priority detection
            int priorityCell = getPriorityDetection(currentDetections);
            qDebug() << "Initial detections in paused frame:" << currentDetections;
            qDebug() << "Priority detection cell:" << priorityCell;
            // Update scheduler with the current frame's state
            if(isProcessing) {
                Scheduler::getInstance().setLayerAndDetection(
                    currentLayer, priorityCell);
            }
        }

        // Draw grid and labels using stored detections
        detector.drawGridAndLabels(cvFrame, params.getRect(), 2, 5, currentDetections); // rows = 2 and columns = 5

        // Convert back to QImage for display
        QImage processedFrame(
            cvFrame.data,
            cvFrame.cols,
            cvFrame.rows,
            cvFrame.step,
            QImage::Format_RGB888
        );

        scene->clear();
        scene->addPixmap(QPixmap::fromImage(processedFrame));

        QString depthInfo = QString("Layer: %1 (Depth: %2 mm)")
                           .arg(currentLayer)
                           .arg(currentDepth, 0, 'f', 1);

        QGraphicsSimpleTextItem* textItem = scene->addSimpleText(depthInfo);
        textItem->setBrush(QBrush(Qt::green));
        textItem->setPos(10, 10);
    }

    ui->graphicsView->fitInView(scene->sceneRect(), Qt::KeepAspectRatio);
}

void DialogCamera::onTimer()
{
    if (!isRunning) {
        return;
    }

    if(isPaused)
    {
        qDebug() << "Camera Status: PAUSED";
        if (!lastCapturedFrame.isNull()) {
//            displayFrame(lastCapturedFrame, true);  // Pass true for paused state
//            addGridAndRectangle(lastCapturedFrame);
            drawDepthBasedROI(lastCapturedFrame);
            updateProcessButton();

        }
    }
    else
    {
        qDebug() << "Camera Status: LIVE";
        acquireLiveFrame();
    }
}

void DialogCamera::onCameraToggle()
{
    if (!isRunning) {
        initializeCamera();
    } else {
        stopCamera();
    }
}

//void DialogCamera::onValueCapture()
//{
//    if (!isRunning) {
//        QMessageBox::warning(this, "Warning", "Please start the camera first.");
//        return;
//    }

//    int value = ui->label->text().toInt();
//    if (value > 0)
//    {
//        valueQueue.enqueue(value);
//        ui->label->setText("0");  // Reset input to 0
//        updateValueDisplay();
//        updateOrderListDisplay();

//        if (!isProcessing && isRunning)
//        {
//            isProcessing = true;
//            int nextValue = valueQueue.dequeue();
//            pauseCamera();
//            Scheduler::getInstance().startProcessing(nextValue);
//            qDebug() << "Processing started with value:" << nextValue;
//        }
//    }
//    else
//    {
//        QMessageBox::warning(this, "Invalid Value", "Please ensure the value is positive.");
//    }
//}

void DialogCamera::updateSchedulerState()
{
    if(isProcessing)
    {
//        Scheduler::getInstance().clearStoredDetections();

        int layer = GetLayer();
        int detection = getPriorityDetection(currentDetections);
        Scheduler::getInstance().setLayerAndDetection(layer, detection);
        qDebug() << "Updated scheduler state - layer:" << layer << "Detection:" << detection;
//        updateProcessButton();
    }
}

void DialogCamera::onValueCapture()
{
    if (!isRunning) {
        QMessageBox::warning(this, "Warning", "Please start the camera first.");
        return;
    }

    int value = ui->label->text().toInt();
    if (value > 0)
    {
        valueQueue.enqueue(value);
        ui->label->setText("0");
        updateValueDisplay();
        updateOrderListDisplay();

        if (!isProcessing && isRunning)
        {
            isProcessing = true;
            int nextValue = valueQueue.dequeue();
            pauseCamera();

            // Wait briefly for YOLO detection to complete
            QTimer::singleShot(100, this, [this, nextValue]() {
                // Get current layer and priority detection after YOLO has run
                int layer = GetLayer();
                int detection = getPriorityDetection(currentDetections);

                Scheduler::getInstance().setLayerAndDetection(layer, detection);
                qDebug() << "Starting process with updated - Layer:" << layer
                         << "Priority Detection:" << detection;
//                updateSchedulerState();

                // Update scheduler with final detection result
//                Scheduler::getInstance().setLayerAndDetection(layer, detection);
                Scheduler::getInstance().startProcessing(nextValue);
            });
        }
    }
    else
    {
        QMessageBox::warning(this, "Invalid Value", "Please ensure the value is positive.");
    }
}

void DialogCamera::onProcessingComplete()
{
    if (!isRunning)
    {
        resetProcessingState();
        return;
    }
    //---------
    resumeCamera();

    // Check if current value still needs processing (not zero)
    if (Scheduler::getInstance().getCurrentValue() > 0)
    {
        // Continue processing current value until it reaches zero
        QTimer::singleShot(3000, this, [this]() {
            if (isRunning && isProcessing) {
                pauseCamera();
                QTimer::singleShot(100, this, [this]() {
                    // Get fresh state from new frame
                    int layer = GetLayer();
                    int detection = getPriorityDetection(currentDetections);
                    // Update scheduler with new state
                    Scheduler::getInstance().setLayerAndDetection(layer, detection);
                    Scheduler::getInstance().startProcessing(0);  // Continue with current value
            });
                }
        });
    }
    else if (!valueQueue.isEmpty())
    {
        // Current value finished, move to next value in queue
        QTimer::singleShot(3000, this, [this]() {
            if (isRunning && isProcessing) {
                int nextValue = valueQueue.dequeue();
                pauseCamera();
                // Important: Allow a small delay for new frame to be processed
                QTimer::singleShot(100, this, [this, nextValue]() {
                    // Get fresh state from new frame
                    int layer = GetLayer();
                    int detection = getPriorityDetection(currentDetections);
                    // Update scheduler with new state
                    Scheduler::getInstance().setLayerAndDetection(layer, detection);
                    Scheduler::getInstance().startProcessing(nextValue);
                    updateValueDisplay();
                    updateOrderListDisplay();
                });
            }
        });
    }
    else
    {
        // All values processed
        isProcessing = false;
        updateValueDisplay();
        ui->LB_ORDER->setText("None");  // Reset to None when all processing is done
        ui->CONTINUE->setEnabled(false); // Disable continue button
        qDebug() << "All values fully processed, camera resumed";
    }
}


//=============================New Add==================

float DialogCamera::GetDepth()
{
    return fullFrameDepth;
}

// Method to set custom measurement point
void DialogCamera::setCustomMeasurementPoint(int x, int y)
{
    const int width = 640;  // Your frame width
    const int height = 480; // Your frame height

    // Ensure coordinates are within frame bounds
    x = clampValue(x, 0, width - 1);
    y = clampValue(y, 0, height - 1);

    customPoint = cv::Point(x, y);
    useCustomPoint = true;
//    qDebug() << "Set custom measurement point to:" << x << "," << y;
}

// Method to reset to default center point
void DialogCamera::resetToDefaultCenter()
{
    useCustomPoint = false;
//    qDebug() << "Reset to default center point measurement";
}

float DialogCamera::getFullFrameDepth(const rs2::depth_frame& depth_frame)
{
    const int width = depth_frame.get_width();
    const int height = depth_frame.get_height();

    if(!useCustomPoint)
    {
        frameCenter = cv::Point(width/2, height/2);
    }
    else
    {
        frameCenter = customPoint;
    }

    return getFullFrameDepth(depth_frame, frameCenter.x, frameCenter.y);
}

int DialogCamera::clampValue(int value, int min, int max)
{
    if(value < min) return min;
    if(value > max) return max;
    return value;
}

float DialogCamera::getFullFrameDepth(const rs2::depth_frame& depth_frame, int x, int y)
{
    const int width = depth_frame.get_width();
    const int height = depth_frame.get_height();

    // Ensure coordinates are within bounds
    x = clampValue(x, 0, width - 1);
    y = clampValue(y, 0, height - 1);

    // Store the measurement point
    frameCenter = cv::Point(x, y);

    // Get depth at the specified point
    float centerDepth = depth_frame.get_distance(x, y) * 1000.0f;

    // Optional: If you want some averaging around the specified point for stability
    if(centerDepth > 0)
    {
        float sum_depth = 0.0f;
        int valid_points = 0;

        // Sample a 5x5 window around the specified point
        for(int dy = -2; dy <= 2; dy++)
        {
            for(int dx = -2; dx <= 2; dx ++)
            {
                int px = x + dx;
                int py = y + dy;

                if(px >= 0 && px < width && py >= 0 && py < height)
                {
                    float d = depth_frame.get_distance(px, py) * 1000.0f;
                    if( d>0)
                    {
                        sum_depth += d;
                        valid_points++;
                    }
                }
            }
        }

        if(valid_points > 0)
        {
            centerDepth = sum_depth / valid_points;
        }
    }

    // Generate color map
    cv::Mat depth_color_map(cv::Size(width, height), CV_8UC3);
    for(int py = 0; py < height; py++)
    {
        for(int px = 0; px < width; px++)
        {
            float depth_value = depth_frame.get_distance(px, py) * 1000.0f;

            cv::Vec3b color;
            if(depth_value <= 0)
            {
                color = cv::Vec3b(0, 0, 0); // Invalide depth
            }
            else if(depth_value < 500)
            {
                color = cv::Vec3b(255, 0, 0); // Close range: blue
            }
            else if(depth_value < 1000)
            {
                color = cv::Vec3b(0, 255, 0); // Mid range: green
            }
            else
            {
                color = cv::Vec3b(0, 0, 255); // Far range: red
            }
            depth_color_map.at<cv::Vec3b>(py, px) = color;
        }
    }

    depthColorMap = depth_color_map;
    qDebug() << "Depth: "<<centerDepth<<"mm";
    return centerDepth;
}

ROIParams DialogCamera::getCurrentROI()
{
    ROIParams params;

    switch(GetLayer())
    {
        case 1:  // Layer 1 (≤80mm)
            params = {40, 30, 210, 100};
//            params = {15, 93, 599, 447};
            break;
        case 2:  // Layer 2 (≤90mm)
            params = {100, 100, 500, 200};
//            params = {62, 108, 560, 422};
            break;
        case 3:  // Layer 3 (≤100mm)
            params = {100, 40, 510, 400};
//            params = {100, 126, 525, 395};
            break;
        case 4:  // Layer 4 (≤110mm)
            params = {110, 45, 510, 400};
//            params = {125, 132, 502, 384};
            break;
        case 5:  // Layer 5 (≤120mm)
            params = {120, 50, 510, 400};
//            params = {148, 143, 481, 367};
            break;
        case 6:
            params = {0, 0, 640, 480};
            break;
        default:
            return params;
    }
    return params;
}

int DialogCamera::GetLayer()
{
    if (GetDepth() <= 0)
    {
        return 0;
    }
    else if (GetDepth() <= LAYER_1_DEPTH)
    {
        return 1;
    }
    else if (GetDepth() <= LAYER_2_DEPTH)
    {
        return 2;
    }
    else if (GetDepth() <= LAYER_3_DEPTH)
    {
        return 3;
    }
    else if (GetDepth() <= LAYER_4_DEPTH)
    {
        return 4;
    }
    else if (GetDepth() <= LAYER_5_DEPTH)
    {
        return 5;
    }
    else
    {
        return 0;
    }
}

void DialogCamera::setupButtonStyles()
{
    // Start/Stop Camera button
    ui->START->setStyleSheet(
        "QPushButton {"
        "    background-color: #3B82F6;"  // blue-500
        "    color: white;"
        "    border-radius: 8px;"
        "    padding: 8px 16px;"
        "    font: bold 20px;"
        "    border: none;"
        "}"
        "QPushButton:hover {"
        "    background-color: #2563EB;"  // blue-600
        "}"
        "QPushButton:pressed {"
        "    background-color: #1D4ED8;"  // blue-700
        "}"
    );

    // Order/Process button
    ui->ORDER->setStyleSheet(
        "QPushButton {"
        "    background-color: #22C55E;"  // green-500
        "    color: white;"
        "    border-radius: 8px;"
        "    padding: 8px 16px;"
        "    font: bold 20px;"
        "    border: none;"
        "}"
        "QPushButton:hover {"
        "    background-color: #16A34A;"  // green-600
        "}"
        "QPushButton:pressed {"
        "    background-color: #15803D;"  // green-700
        "}"
    );

    // Positive button
//    ui->POSBTN->setStyleSheet(
//        "QPushButton {"
//        "    background-color: #FED7D7;"  // rose-200
//        "    color: #1F2937;"  // gray-800
//        "    border-radius: 8px;"
//        "    padding: 4px 12px;"
//        "    font: bold 16px;"
//        "    border: none;"
//        "}"
//        "QPushButton:hover {"
//        "    background-color: #FCA5A5;"  // rose-300
//        "}"
//        "QPushButton:pressed {"
//        "    background-color: #F87171;"  // rose-400
//        "}"
//    );
    ui->POSBTN->setStyleSheet(
            "QPushButton {"
            "    background-color: #a0a7db;"
            "    border-radius: 20px;"
            "    min-width: 40px;"
            "    min-height: 40px;"
            "    font: bold 20px;"
            "}"
            "QPushButton:hover {"
            "    background-color: #d986cc;"
            "}"
            "QPushButton:pressed {"
            "    background-color: #d986cc;"
            "}"
        );

    // Negative button
//    ui->NEGBTN->setStyleSheet(
//        "QPushButton {"
//        "    background-color: #E5E7EB;"  // gray-200
//        "    color: #1F2937;"  // gray-800
//        "    border-radius: 8px;"
//        "    padding: 4px 12px;"
//        "    font: bold 16px;"
//        "    border: none;"
//        "}"
//        "QPushButton:hover {"
//        "    background-color: #D1D5DB;"  // gray-300
//        "}"
//        "QPushButton:pressed {"
//        "    background-color: #9CA3AF;"  // gray-400
//        "}"
//    );

    ui->NEGBTN->setStyleSheet(
            "QPushButton {"
            "    background-color: #a0a7db;"
            "    border-radius: 20px;"
            "    min-width: 40px;"
            "    min-height: 40px;"
            "    font: bold 20px;"
            "}"
            "QPushButton:hover {"
            "    background-color: #E5E7EB;"
            "}"
            "QPushButton:pressed {"
            "    background-color: #D1D5DB;"
            "}"
        );

    // Continue button
    ui->CONTINUE->setStyleSheet(
        "QPushButton {"
        "    background-color: #22C55E;"  // green-500
        "    color: white;"
        "    border-radius: 8px;"
        "    padding: 8px 16px;"
        "    font: bold 14px;"
        "    border: none;"
        "}"
        "QPushButton:disabled {"
        "    background-color: #D1D5DB;"  // gray-300
        "    color: #6B7280;"  // gray-500
        "}"
        "QPushButton:hover:!disabled {"
        "    background-color: #16A34A;"  // green-600
        "}"
        "QPushButton:pressed:!disabled {"
        "    background-color: #15803D;"  // green-700
        "}"
    );

    // Badge Label Style
    ui->label->setStyleSheet(
        "QLabel {"
        "    background-color: #EFF6FF;"
        "    color: #2563EB;"
        "    border-radius: 14px;"
        "    padding: 6px 18px;"
        "    font: bold 16px;"
        "    qproperty-alignment: AlignCenter;"
        "}"
    );

    // Success Label Style
    ui->LB_ORDER->setStyleSheet(
        "QLabel {"
        "    color: #059669;"
        "    font: bold 24px;"
        "    padding: 4px;"
        "}"
    );

    ui->LIST->setStyleSheet(
        "QLabel {"
        "    color: #5B21B6;"
        "    font: bold 22px;"
        "    padding: 4px 12px;"
        "    qproperty-alignment: AlignCenter;"
        "    background-color: #F5F3FF;"
        "    border-radius: 12px;"
        "}"
    );
    ui->LIST_2->setStyleSheet(
        "QLabel {"
        "    color: #5B21B6;"
        "    font: bold 22px;"
        "    padding: 4px 12px;"
        "    qproperty-alignment: AlignCenter;"
        "    background-color: #F5F3FF;"
        "    border-radius: 12px;"
        "}"
    );

    // Success Label
//    ui->valueLabel->setStyleSheet(
//        "QLabel {"
//        "    color: #059669;"
//        "    font: bold 24px;"
//        "    padding: 8px 16px;"
//        "    qproperty-alignment: AlignCenter;"
//        "    background-color: #ECFDF5;"
//        "    border: 1px solid #059669;"
//        "    border-radius: 16px;"
//        "}"
//    );

    ui->valueLabel->setStyleSheet(
        "QLabel {"
        "    color: #059669;"
        "    font: bold 24px;"
        "    padding: 4px;"
        "}"
    );
}
