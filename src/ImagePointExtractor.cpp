#include "ImagePointExtractor.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <chrono>
#include <iostream>
#include <string>
#include <thread>

using namespace cv;
using namespace std;
using namespace std::chrono;

static bool
isImageListFile(string const &file) {
    return file.find(".xml") == string::npos and file.find(".yaml") == string::npos and
        file.find(".yml") == string::npos;
}

bool
ImagePointExtractor::readSettings(const FileNode &node) {
    string input;
    int stillDelayMs, cooldownDurationMs;
    node["Input"] >> input;
    node["FlipInputAroundHorizontalAxis"] >> m_flipHorizontal;
    node["FlipInputAroundVerticalAxis"] >> m_flipVertical;
    node["StillDelay"] >> stillDelayMs;
    node["CooldownDuration"] >> cooldownDurationMs;
    node["GridWidth"] >> m_gridSize.width;
    node["GridHeight"] >> m_gridSize.height;
    node["ImagesPerGridCell"] >> m_detectionsPerGridCell;

    m_inputType = InputType::Invalid;
    m_cameraId = -1;
    m_videoFile.clear();
    m_imageFiles.clear();
    if (input[0] >= '0' and input[0] <= '9') {
        try {
            m_cameraId = stoi(input);
            m_inputType = InputType::Camera;
        } catch (...) {
            m_cameraId = -1;
        }
    } else if (isImageListFile(input)) {
        m_imageFiles.clear();
        FileStorage fs(input, FileStorage::READ);
        if (fs.isOpened()) {
            FileNode node = fs.getFirstTopLevelNode();
            if (node.type() == FileNode::SEQ) {
                for (auto it = node.begin(); it != node.end(); ++it) {
                    m_imageFiles.push_back(static_cast<string>(*it));
                }
            }
        }
        if (not m_imageFiles.empty()) {
            m_inputType = InputType::ImageList;
        }
    } else {
        m_videoFile = input;
        m_inputType = InputType::VideoFile;
    }

    m_stillDelay = stillDelayMs * 1e-3;
    m_cooldownDuration = cooldownDurationMs * 1e-3;

    validateSettings();
    return m_settingsValid;
}

bool
ImagePointExtractor::settingsValid() const {
    return m_settingsValid;
}

static void
drawImageShapes(Mat &image, vector<vector<Point2i>> const &imageShapes) {
    Mat imageOverlay = image.clone();
    for (auto const &shape : imageShapes) {
        fillConvexPoly(imageOverlay, shape, Scalar(255, 255, 255));
    }
    addWeighted(image, 0.8, imageOverlay, 0.2, 0.0, image);
}

static void
drawImageGrid(Mat &image, std::vector<std::vector<int>> const &imageGrid, int imageCountTarget) {
    Mat imageOverlay = image.clone();
    for (size_t i = 0; i < imageGrid.size(); ++i) {
        for (size_t j = 0; j < imageGrid[i].size(); ++j) {
            int x1 = static_cast<int>(i * (image.cols * 1.0 / imageGrid.size())) + 1;
            int x2 = static_cast<int>((i + 1) * (image.cols * 1.0 / imageGrid.size())) - 1;
            int y1 = static_cast<int>(j * (image.rows * 1.0 / imageGrid[i].size())) + 1;
            int y2 = static_cast<int>((j + 1) * (image.rows * 1.0 / imageGrid[i].size())) - 1;
            Point topLeft{x1, y1};
            Point bottomRight{x2, y2};
            int imageCount = imageGrid[i][j];
            double targetFrac = static_cast<double>(imageCount) / imageCountTarget;
            Scalar color = Scalar(0, 255, 0) * targetFrac + Scalar(0, 0, 255) * (1 - targetFrac);
            rectangle(imageOverlay, topLeft, bottomRight, color, 2);
        }
    }
    addWeighted(image, 0.5, imageOverlay, 0.5, 0.0, image);
}

static void
drawMessage(Mat &image, std::string const &message, Scalar color) {
    int baseLine = 0;
    Size textSize = getTextSize(message, FONT_HERSHEY_SIMPLEX, 1, 2, &baseLine);
    Point textOrigin(image.cols - 2 * textSize.width - 10, image.rows - 2 * baseLine - 10);
    putText(image, message, textOrigin, FONT_HERSHEY_SIMPLEX, 1, color, 2);
}

bool
ImagePointExtractor::findImagePoints(vector<vector<Point2f>> &imagePoints,
                                     Size &imageSize,
                                     const ReferenceObject &referenceObject) const {
    Data data;
    data.referenceObject = referenceObject;
    data.imageCountGrid.resize(static_cast<size_t>(m_gridSize.width));
    for (vector<int> &column : data.imageCountGrid) {
        column.resize(static_cast<size_t>(m_gridSize.height), 0);
    }

    if (m_inputType == InputType::Camera) {
        data.inputCapture.open(m_cameraId);
    } else if (m_inputType == InputType::VideoFile) {
        data.inputCapture.open(m_videoFile);
    }

    data.videoThreadRunning = data.inputCapture.isOpened();
    std::thread videoThread{&ImagePointExtractor::runVideoThread, std::ref(*this), std::ref(data)};
    if (data.inputCapture.isOpened()) {
        unique_lock<mutex> lock(data.videoImageMutex);
        data.videoImageCondition.wait_for(lock, 5s);
    }

    while (static_cast<int>(imagePoints.size()) < m_detectionsPerGridCell * m_gridSize.area()) {
        Mat image = nextImage(data);
        if (image.empty()) {
            bool enoughUsefulFrames = imagePoints.size() >=
                static_cast<size_t>(m_detectionsPerGridCell * m_gridSize.area());
            if (!enoughUsefulFrames) {
                cerr << "Input did not contain enough useful images" << endl;
            }
            return enoughUsefulFrames;
        }

        imageSize = image.size();
        manipulateImage(image);

        vector<Point2f> currentImagePoints;
        bool found = detectFeaturePoints(currentImagePoints, image, data);
        bool pointsAdded = false;
        if (found) {
            pointsAdded = addImagePoints(imagePoints, data, currentImagePoints, imageSize);
        }

        if (data.inputCapture.isOpened()) {
            if (pointsAdded) {
                bitwise_not(image, image);
            }

            drawImageShapes(image, data.imageShapes);

            drawImageGrid(image, data.imageCountGrid, m_detectionsPerGridCell);

            if (found) {
                drawChessboardCorners(
                    image, referenceObject.boardSize(), currentImagePoints, found);
                string message;
                Scalar color;
                if (data.moving) {
                    message = "Keep still";
                    color = Scalar(0, 0, 255); // red
                } else if (data.gridCellFull) {
                    message = "Enough points here";
                    color = Scalar(0, 165, 255); // orange
                } else if (data.cooldownActive) {
                    message = "Cooldown active";
                    color = Scalar(0, 255, 255); // yellow
                } else {
                    message = "Added";
                    color = Scalar(0, 255, 0); // green
                }
                drawMessage(image, message, color);
            }
        } else if (found) {
            drawChessboardCorners(image, referenceObject.boardSize(), currentImagePoints, found);
        }

        imshow("Image View", image);
        waitKey(data.inputCapture.isOpened() ? (pointsAdded ? 200 : 1)
                                             : static_cast<int>(m_cooldownDuration / 1e+3));
    }

    destroyWindow("Image View");
    data.videoThreadRunning = false;
    videoThread.join();

    if (data.inputCapture.isOpened()) {
        data.inputCapture.release();
    }

    return true;
}

void
ImagePointExtractor::showUndistoredInput(const CameraCalibrationResult &result) {
    Data data;

    if (m_inputType == InputType::Camera) {
        data.inputCapture.open(m_cameraId);
    } else if (m_inputType == InputType::VideoFile) {
        data.inputCapture.open(m_videoFile);
    }
    if (!data.inputCapture.isOpened()) {
        return;
    }

    data.videoThreadRunning = true;
    std::thread videoThread{&ImagePointExtractor::runVideoThread, std::ref(*this), std::ref(data)};
    {
        unique_lock<mutex> lock(data.videoImageMutex);
        data.videoImageCondition.wait_for(lock, 5s);
    }

    bool undistorted = true;
    for (Mat view = nextImage(data); !view.empty(); view = nextImage(data)) {
        manipulateImage(view);

        if (undistorted) {
            Mat temp = view.clone();
            undistort(temp, view, result.cameraMatrix, result.distortionCoefficients);
        }

        drawMessage(view, undistorted ? "undistorted" : "distorted", Scalar(0, 255, 0));

        imshow("Image View", view);
        char key = static_cast<char>(waitKey(1));
        if (key == 'u') {
            undistorted = true;
        } else if (key == 'd') {
            undistorted = false;
        } else if (key == 27) {
            break;
        }
    }

    destroyWindow("Image View");
    data.videoThreadRunning = false;
    videoThread.join();

    if (data.inputCapture.isOpened()) {
        data.inputCapture.release();
    }
}

bool
ImagePointExtractor::addImagePoints(std::vector<std::vector<Point2f>> &imagePoints,
                                    ImagePointExtractor::Data &data,
                                    const std::vector<Point2f> &currentImagePoints,
                                    const Size &imageSize) const {

    duration<double> timeSinceEpoch = high_resolution_clock::now().time_since_epoch();
    double currentTime = timeSinceEpoch.count();

    vector<Point2i> currentShape;
    determineImageShape(currentShape, currentImagePoints, data);
    Point2i meanPosition;
    for (Point2i shapePoint : currentShape) {
        meanPosition += shapePoint;
    }
    meanPosition /= static_cast<int>(currentShape.size());
    size_t gridRowIdx = static_cast<size_t>(meanPosition.y * m_gridSize.height / imageSize.height);
    size_t gridColumnIdx = static_cast<size_t>(meanPosition.x * m_gridSize.width / imageSize.width);

    if (data.referenceShape.size() == 0) {
        data.referenceShape = currentShape;
        data.referenceTime = currentTime;
    }

    double moveDistance = std::min(imageSize.width, imageSize.height) * 0.03;

    bool movedFromReference = false;
    for (size_t i = 0; i < data.referenceShape.size(); ++i) {
        Point2i referencePoint = data.referenceShape[i];
        Point2i currentPoint = currentShape[i];
        if (norm(referencePoint - currentPoint) >= moveDistance) {
            movedFromReference = true;
            data.referenceShape = currentShape;
            data.referenceTime = currentTime;
            break;
        }
    }

    data.isVideoStream = data.inputCapture.isOpened();
    data.cooldownActive = currentTime - data.pointsAddedTime < m_cooldownDuration;
    data.moving = movedFromReference or (currentTime - data.referenceTime < m_stillDelay);
    data.gridCellFull = data.imageCountGrid[gridColumnIdx][gridRowIdx] >= m_detectionsPerGridCell;

    if (data.isVideoStream and (data.cooldownActive or data.moving or data.gridCellFull)) {
        return false;
    }

    imagePoints.push_back(currentImagePoints);
    data.imageShapes.push_back(currentShape);
    data.imageCountGrid[gridColumnIdx][gridRowIdx] += 1;
    data.pointsAddedTime = currentTime;
    return true;
}

bool
ImagePointExtractor::detectFeaturePoints(vector<Point2f> &featurePoints,
                                         Mat const &image,
                                         Data const &data) const {
    Size boardSize = data.referenceObject.boardSize();
    switch (data.referenceObject.calibrationPattern()) {
    case ReferenceObject::PatternType::Chessboard: {
        if (findChessboardCorners(image, boardSize, featurePoints)) {
            Mat imageGray;
            cvtColor(image, imageGray, COLOR_BGR2GRAY);
            cornerSubPix(imageGray,
                         featurePoints,
                         Size(11, 11),
                         Size(-1, -1),
                         TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
            return true;
        } else {
            return false;
        }
    }
    case ReferenceObject::PatternType::CirclesGrid: {
        return findCirclesGrid(image, boardSize, featurePoints);
    }
    case ReferenceObject::PatternType::AsymmetricCirclesGrid: {
        return findCirclesGrid(image, boardSize, featurePoints, CALIB_CB_ASYMMETRIC_GRID);
    }
    case ReferenceObject::PatternType::Invalid: {
        cerr << "No valid pattern type was specified" << endl;
        return false;
    }
    }
}

void
ImagePointExtractor::determineImageShape(vector<Point2i> &shape,
                                         vector<Point2f> const &featurePoints,
                                         Data const &data) const {
    int patternColumns = data.referenceObject.boardSize().width;
    int patternRows = data.referenceObject.boardSize().height;
    float squareSize = data.referenceObject.squareSize();
    ReferenceObject::PatternType patternType = data.referenceObject.calibrationPattern();

    shape.resize(4);
    shape[0] = featurePoints.at(0);
    shape[1] = featurePoints.at(static_cast<size_t>(patternColumns) - 1);
    shape[2] = featurePoints.at(featurePoints.size() - 1);
    shape[3] = featurePoints.at(featurePoints.size() - static_cast<size_t>(patternColumns));

    if (patternType == ReferenceObject::PatternType::AsymmetricCirclesGrid) {
        shape[1].x += squareSize;
        if (patternRows % 2 == 1) {
            shape[2].x += squareSize;
        } else {
            shape[3].x -= squareSize;
        }
    }
}

void
ImagePointExtractor::manipulateImage(Mat &image) const {
    if (m_flipHorizontal) {
        flip(image, image, 0);
    }

    if (m_flipVertical) {
        flip(image, image, 1);
    }
}

Mat
ImagePointExtractor::nextImage(ImagePointExtractor::Data &data) const {
    Mat result;
    if (data.inputCapture.isOpened()) {
        lock_guard<mutex> lock{data.videoImageMutex};
        data.videoImage.copyTo(result);
    } else if (data.imageFileIdx < m_imageFiles.size()) {
        result = imread(m_imageFiles[data.imageFileIdx++], IMREAD_COLOR);
    }
    return result;
}

void
ImagePointExtractor::runVideoThread(ImagePointExtractor::Data &data) const {
    while (data.videoThreadRunning and data.inputCapture.isOpened()) {
        data.inputCapture.grab();
        lock_guard<mutex> lock{data.videoImageMutex};
        data.inputCapture.retrieve(data.videoImage);
        data.videoImageCondition.notify_one();
    }
}

bool
ImagePointExtractor::validateSettings() {
    m_settingsValid = true;

    if (m_inputType == InputType::Invalid) {
        cerr << "Invalid input type: unknown" << endl;
        m_settingsValid = false;
    }

    switch (m_inputType) {
    case InputType::Camera: {
        VideoCapture capture;
        if (not capture.open(m_cameraId)) {
            cerr << "Invalid input: Couldn't open camera with id " << m_cameraId << endl;
            m_settingsValid = false;
        }
        capture.release();
        break;
    }
    case InputType::VideoFile: {
        VideoCapture capture;
        if (not capture.open(m_videoFile)) {
            cerr << "Invalid input: Couldn't open video file \"" << m_videoFile << '"' << endl;
            m_settingsValid = false;
        }
        capture.release();
        break;
    }
    case InputType::ImageList: {
        for (string const &imageFile : m_imageFiles) {
            Mat image = imread(imageFile);
            if (image.empty()) {
                cerr << "Invalid input: Couldn't open image file \"" << imageFile << '"' << endl;
                m_settingsValid = false;
            }
        }
        break;
    }
    default:
        break;
    }

    if (m_stillDelay < 0) {
        cerr << "Invalid still delay" << endl;
        m_settingsValid = false;
    }

    if (m_cooldownDuration < 0) {
        cerr << "Invalid cooldown duration" << endl;
        m_settingsValid = false;
    }

    if (m_gridSize.height <= 0 or m_gridSize.width <= 0) {
        cerr << "Invalid grid size: " << m_gridSize.width << ' ' << m_gridSize.height << endl;
        m_settingsValid = false;
    }

    if (m_detectionsPerGridCell <= 0) {
        cerr << "Invalid number of images per grid" << endl;
        m_settingsValid = false;
    }
}

void
read(const FileNode &node,
     ImagePointExtractor &imagePointExtractor,
     const ImagePointExtractor &defaultValue) {
    if (node.empty()) {
        imagePointExtractor = defaultValue;
    } else {
        imagePointExtractor.readSettings(node);
    }
}
