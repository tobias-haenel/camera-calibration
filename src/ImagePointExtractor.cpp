#include "ImagePointExtractor.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <chrono>
#include <iostream>
#include <string>
#include <thread>

#include "ThreadSafePrinter.h"
#include "Utility.h"

using namespace cv;
using namespace std;
using namespace std::chrono;

bool
ImagePointExtractor::readSettings(const FileNode &node) {
    int stillDelayMs, cooldownDurationMs;
    node["StillDelay"] >> stillDelayMs;
    node["CooldownDuration"] >> cooldownDurationMs;
    node["GridWidth"] >> m_gridSize.width;
    node["GridHeight"] >> m_gridSize.height;
    node["ImagesPerGridCell"] >> m_detectionsPerGridCell;

    m_stillDelay = stillDelayMs * 1e-3;
    m_cooldownDuration = cooldownDurationMs * 1e-3;

    validateSettings();
    return m_settingsValid;
}

bool
ImagePointExtractor::settingsValid() const {
    return m_settingsValid;
}

bool
ImagePointExtractor::findReferenceObjectImagePoints(vector<vector<Point2f>> &imagePoints,
                                                    Size &imageSize,
                                                    int &focusValue,
                                                    const ReferenceObject &referenceObject,
                                                    CameraInput &input) const {
    Data data;
    data.referenceObject = referenceObject;
    data.imageCountGrid.resize(static_cast<size_t>(m_gridSize.width));
    for (vector<int> &column : data.imageCountGrid) {
        column.resize(static_cast<size_t>(m_gridSize.height), 0);
    }

    shared_ptr<CameraImage> cameraImage = input.cameraImage();
    if (not cameraImage->waitForNewImage()) {
        printErr << "Did not receive any images" << endl;
        return false;
    }

    double imageTimeStamp = 0.0;
    Mat image, featureImage;
    bool error = false;
    while (static_cast<int>(imagePoints.size()) < m_detectionsPerGridCell * m_gridSize.area()) {
        if (not input.isImaging()) {
            printErr << "Imaging stopped before enough feature points could be detected" << endl;
            error = true;
            break;
        }
        cameraImage->currentImage(image, imageTimeStamp, focusValue);
        imageSize = image.size();
        if (imageSize.empty()) {
            printErr << "Obtained empty image" << endl;
            error = true;
            break;
        }

        determineFeatureImage(image, featureImage);
        imshow("Feature Image View", featureImage);

        vector<Point2f> currentImagePoints;
        bool found = detectFeaturePoints(currentImagePoints, featureImage, data);
        bool pointsAdded = false;
        if (found) {
            pointsAdded = addImagePoints(imagePoints, data, currentImagePoints, imageSize);
        } else {
            data.referenceTime = imageTimeStamp;
        }

        if (pointsAdded) {
            Mat imageWithCorners = image.clone();
            drawChessboardCorners(
                imageWithCorners, referenceObject.boardSize(), currentImagePoints, found);
            imwrite("image-" + to_string(imagePoints.size()) + ".png", imageWithCorners);
            imwrite("feature-" + to_string(imagePoints.size()) + ".png", featureImage);
            bitwise_not(image, image);
        }

        drawImageShapes(image, data.imageShapes);

        drawImageGrid(image, data.imageCountGrid, m_detectionsPerGridCell);

        if (found) {
            drawChessboardCorners(image, referenceObject.boardSize(), currentImagePoints, found);
        }

        if (found) {
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
            } else if (not data.addImages) {
                message = "Images aren't added";
                color = Scalar(255, 0, 0);
            } else {
                message = "Added";
                color = Scalar(0, 255, 0); // green
            }
            drawMessage(image, message, color);
        }
        imshow("Image View", image);
        char key = waitKey(pointsAdded ? 200 : 1);
        if (key == ' ') {
            data.addImages = not data.addImages;
        }
    }

    destroyWindow("Image View");
    destroyWindow("Feature Image View");

    return not error;
}

void
ImagePointExtractor::showUndistoredInput(const IntrinsicCameraParameters &result,
                                         CameraInput &input) {
    Data data;

    shared_ptr<CameraImage> cameraImage = input.cameraImage();
    if (not cameraImage->waitForNewImage()) {
        printErr << "Did not receive any images" << endl;
        return;
    }

    bool undistorted = true;
    Mat image;
    double imageTimeStamp = 0.0;
    int imageFocusValue;
    while (true) {
        cameraImage->currentImage(image, imageTimeStamp, imageFocusValue);
        Mat featureImage;
        determineFeatureImage(image, featureImage);

        if (undistorted) {
            Mat temp = image.clone();
            undistort(temp, image, result.cameraMatrix, result.distortionCoefficients);
        }

        drawMessage(image, undistorted ? "undistorted" : "distorted", Scalar(0, 255, 0));

        imshow("Image View", image);
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

    data.cooldownActive = currentTime - data.pointsAddedTime < m_cooldownDuration;
    data.moving = movedFromReference or (currentTime - data.referenceTime < m_stillDelay);
    data.gridCellFull = data.imageCountGrid[gridColumnIdx][gridRowIdx] >= m_detectionsPerGridCell;

    if (data.cooldownActive or data.moving or data.gridCellFull or not data.addImages) {
        return false;
    }

    {
        size_t index = imagePoints.size() + 1;
        FileStorage file{"image-points-" + to_string(index) +".xml", FileStorage::WRITE};
        file << "ImagePointCount" << static_cast<int>(currentImagePoints.size());
        for (size_t i = 0; i < currentImagePoints.size(); ++i) {
            file << "ImagePoint" + to_string(i) << currentImagePoints.at(i);
        }
//        size_t i = 0;
//        ThreadSafePrinter print{cout};
//        print << "Added image points: " << endl;
//        for (int y = 0; y < data.referenceObject.boardSize().height; ++y) {
//            print << "  ";
//            for (int x = 0; x < data.referenceObject.boardSize().width; ++x, ++i) {
//                Point2f p = currentImagePoints.at(i);
//                print << (x != 0 ? " " : "") << '(' << p.x << ", " << p.y << ')';
//            }
//            print << endl;
//        }
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
        // determine threshold value that partions image in 2 areas where their mean values have the
        // same distance to the threshold value
        double autoThresh = 127.5;
        for (int i = 0; i < 4; ++i) {
            Mat mask, invMask;
            threshold(image, mask, autoThresh, 255, CV_THRESH_BINARY);
            bitwise_not(mask, invMask);
            double mean1 = mean(image, mask)[0];
            double mean2 = mean(image, invMask)[0];
            autoThresh = (mean1 + mean2) / 2;
        }
        SimpleBlobDetector::Params detectionParams;
        detectionParams.minThreshold = static_cast<int>(autoThresh - 20);
        detectionParams.maxThreshold = static_cast<int>(autoThresh + 20);
        detectionParams.thresholdStep = 5;
        Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(detectionParams);
        return findCirclesGrid(image, boardSize, featurePoints, CALIB_CB_SYMMETRIC_GRID, detector);
    }
    case ReferenceObject::PatternType::Invalid: {
        printErr << "No valid pattern type was specified" << endl;
        return false;
    }
    }
}

void
ImagePointExtractor::determineImageShape(vector<Point2i> &shape,
                                         vector<Point2f> const &featurePoints,
                                         Data const &data) const {
    int patternColumns = data.referenceObject.boardSize().width;
    shape.resize(4);
    shape[0] = featurePoints.at(0);
    shape[1] = featurePoints.at(static_cast<size_t>(patternColumns) - 1);
    shape[2] = featurePoints.at(featurePoints.size() - 1);
    shape[3] = featurePoints.at(featurePoints.size() - static_cast<size_t>(patternColumns));
}

void
ImagePointExtractor::determineFeatureImage(const Mat &image, Mat &featureImage) const {
    featureImage = image.clone();

    cvtColor(featureImage, featureImage, CV_RGB2GRAY);
    bitwise_not(featureImage, featureImage);
}

void
ImagePointExtractor::validateSettings() {
    m_settingsValid = true;

    if (m_stillDelay < 0) {
        printErr << "Invalid still delay" << endl;
        m_settingsValid = false;
    }

    if (m_cooldownDuration < 0) {
        printErr << "Invalid cooldown duration" << endl;
        m_settingsValid = false;
    }

    if (m_gridSize.height <= 0 or m_gridSize.width <= 0) {
        printErr << "Invalid grid size: " << m_gridSize.width << ' ' << m_gridSize.height << endl;
        m_settingsValid = false;
    }

    if (m_detectionsPerGridCell <= 0) {
        printErr << "Invalid number of images per grid" << endl;
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
