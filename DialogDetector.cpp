#include "DialogDetector.h"

DialogDetector::DialogDetector()
    : confThreshold(0.5f)
    , nmsThreshold(0.4f)
    , isInitialized(false)
{}

DialogDetector::~DialogDetector(){}

void DialogDetector::loadYOLOModel(const std::string &modelConfig, const std::string &modelWeights)
{
    net = cv::dnn::readNetFromDarknet(modelConfig, modelWeights);
    net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    isInitialized = true;
}

std::vector<std::string> DialogDetector::getOutputLayerNames()
{
    return net.getUnconnectedOutLayersNames();
}

std::vector<int> DialogDetector::detectObjectsInGridCells(cv::Mat &image, const cv::Rect &roi, int rows, int cols)
{
    if(!isInitialized)
    {
        std::cerr << "YOLO model not initialized!" << std::endl;
        return std::vector<int>(rows*cols, -1);
    }

    std::vector<int>cellResults(rows*cols, -1);
    int cellWidth = roi.width/cols;
    int cellHeight = roi.height/rows;

    for(int row=0; row<rows; ++row)
    {
        for(int col=0; col<cols; ++col)
        {
            cv::Rect cell(roi.x+col*cellWidth, roi.y+row*cellHeight, cellWidth, cellHeight);
            cv::Mat cellImage = image(cell);

            if(cellImage.empty() || cellImage.rows<10 || cellImage.cols<10)
            {
                continue;
            }

            try
            {
                // Prepare image for YOLO
                cv::Mat blob = cv::dnn::blobFromImage(cellImage, 1/255.0,
                                                      cv::Size(416, 416),
                                                      cv::Scalar(0, 0, 0),
                                                      true,
                                                      false);
                net.setInput(blob);

                std::vector<cv::Mat>outs;
                net.forward(outs, getOutputLayerNames());

                // Process detections
                std::vector<int>classIds;
                std::vector<float>confidences;
                std::vector<cv::Rect>boxes;

                // Only consider high-confidence detections
                float minConfidence = 0.7f; // Confidence threshold

                for(auto& output : outs)
                {
                    for(int i=0; i<output.rows; ++i)
                    {
                        float* data = (float*)output.ptr(i);
                        float confidence = data[4];

                        if(confidence > minConfidence)
                        {
                            // Additional size checks for detected objects
                            float width = data[2]*cellImage.cols;
                            float height = data[3]*cellImage.rows;

                            // Skip if detected object is too small relative to cell size
                            if(width < cellWidth*0.2 || height<cellHeight*0.2)
                            {
                                continue;
                            }

                            int centerX = (int)(data[0]*cellImage.cols);
                            int centerY = (int)(data[1]*cellImage.rows);
                            int left = centerX - width/2;
                            int top = centerY - height/2;

                            classIds.push_back((int)data[5]);
                            confidences.push_back(confidence);
                            boxes.push_back(cv::Rect(left, top, width, height));
                        }
                    }
                }

                // Apply NMS with stricter thershold
                std::vector<int>indices;
                cv::dnn::NMSBoxes(boxes, confidences, minConfidence, 0.3f, indices);

                if(!indices.empty())
                {
                    cellResults[row*cols+col] = row*cols+col+1;
                }
            }
            catch(const cv::Exception& e)
            {
                std::cerr << "Error processing cell "<<(row*cols+col) << ":" << e.what() << std::endl;
            }
        }
    }
    return cellResults;
}


void DialogDetector::drawGridAndLabels(cv::Mat &image, const cv::Rect &roi,
                                     int rows, int cols,
                                     const std::vector<int> &cellResults)
{
    int cellWidth = roi.width/cols;
    int cellHeight = roi.height/rows;

    // Draw outer boundaries first with solid thin line
    cv::rectangle(image, roi, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);

    // Draw inner grid lines with slightly different style
    // Vertical lines
    for(int col = 1; col < cols; ++col) {
        int x = roi.x + col * cellWidth;
        cv::line(image,
                 cv::Point(x, roi.y),
                 cv::Point(x, roi.y + roi.height),
                 cv::Scalar(0, 255, 0),
                 1,
                 cv::LINE_AA);
    }

    // Horizontal lines
    for(int row = 1; row < rows; ++row) {
        int y = roi.y + row * cellHeight;
        cv::line(image,
                 cv::Point(roi.x, y),
                 cv::Point(roi.x + roi.width, y),
                 cv::Scalar(0, 255, 0),
                 1,
                 cv::LINE_AA);
    }

    // Draw cell labels
    for(int row = 0; row < rows; ++row)
    {
        for(int col = 0; col < cols; ++col)
        {
            int x = roi.x + col * cellWidth;
            int y = roi.y + row * cellHeight;
            int cellIndex = row * cols + col;

            std::string label = "-1"; // Default label for empty cells
            cv::Scalar textColor(0, 0, 255); // Red for empty cells

            if(cellResults[cellIndex] > 0)
            {
                label = std::to_string(cellResults[cellIndex]);
                textColor = cv::Scalar(0, 255, 0); // Green for detected cells
            }

            // Calculate text position for center alignment
            int fontFace = cv::FONT_HERSHEY_SIMPLEX;
            double fontScale = 0.6;  // Slightly smaller text
            int thickness = 1;  // Thinner text
            cv::Size textSize = cv::getTextSize(label, fontFace, fontScale, thickness, nullptr);

            // Ensure text is centered in cell
            cv::Point textPos(
                x + (cellWidth - textSize.width) / 2,
                y + (cellHeight + textSize.height) / 2
            );

            // Draw text with thin outline for better visibility
            cv::putText(image, label, textPos,
                       fontFace, fontScale,
                       cv::Scalar(0, 0, 0), thickness + 1,
                       cv::LINE_AA);

            cv::putText(image, label, textPos,
                       fontFace, fontScale,
                       textColor, thickness,
                       cv::LINE_AA);
        }
    }
}
