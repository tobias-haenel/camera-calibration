#include "TransformationDetermination.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>

#include <chrono>

#include "Utility.h"

using namespace std;
using namespace std::chrono;
using namespace cv;

bool
TransformationDetermination::readSettings(const FileNode &node) {
    string poseEstimationMethod;
    node["ObjectLocating"] >> m_objectLocater;
    node["ImagePointExtraction"] >> m_imagePointExtractor;
    node["PoseEstimationMethod"] >> poseEstimationMethod;
    node["XVectorScale"] >> m_xVectorScale;
    node["YVectorScale"] >> m_yVectorScale;
    node["PatternOriginXOffset"] >> m_patternOriginOffset.x;
    node["PatternOriginYOffset"] >> m_patternOriginOffset.y;
    node["CooldownDuration"] >> m_cooldownDuration;
    node["StillDelay"] >> m_stillDelay;
    node["MinimalPositionCount"] >> m_minimalPositionCount;
    node["MoveDistance"] >> m_moveDistance;

    if (poseEstimationMethod == "Iterative") {
        m_flag = SOLVEPNP_ITERATIVE;
    } else if (poseEstimationMethod == "P3P") {
        m_flag = SOLVEPNP_P3P;
    } else if (poseEstimationMethod == "AP3P") {
        m_flag = SOLVEPNP_AP3P;
    } else if (poseEstimationMethod == "DLS") {
        m_flag = SOLVEPNP_DLS;
    } else if (poseEstimationMethod == "UPNP") {
        m_flag = SOLVEPNP_UPNP;
    }

    validateSettings();

    return m_settingsValid;
}

bool
TransformationDetermination::settingsValid() const {
    return m_settingsValid;
}

bool
TransformationDetermination::determineConversionTransformation(
    const IntrinsicCameraParameters &intrinsicCameraParameters,
    const ReferenceObject &patternObject,
    CameraInput &input,
    Mat &referenceToCameraTransformation) {

    // find image points of still pattern object
    vector<vector<Point2f>> pointSets;
    Size imageSize;
    if (not m_imagePointExtractor.findImagePoints(pointSets, imageSize, patternObject, input)) {
        printErr << "Couldn't find image points" << endl;
        return false;
    }
    vector<Point2f> &foundImagePoints = pointSets.at(0);

    // extract required image points for pose estimation
    vector<Point2f> imagePoints;
    size_t patternColumns = static_cast<size_t>(patternObject.boardSize().width);
    size_t pointCount = foundImagePoints.size();
    size_t cornerIndices[4]{0, patternColumns - 1, pointCount - 1, pointCount - patternColumns};
    vector<Point2f> cornerImagePoints;
    for (size_t cornerIdx : cornerIndices) {
        Point2f const &corner = foundImagePoints.at(cornerIdx);
        cornerImagePoints.push_back(corner);
    }
    if (m_flag == SOLVEPNP_P3P or m_flag == SOLVEPNP_AP3P) {
        imagePoints = cornerImagePoints; // some pose estimation algorithms require 4 points
    } else {
        imagePoints = foundImagePoints; // other algorithms can deal with more points
    }

    // locate necessary points/elements
    if (not m_objectLocater.startTracking()) {
        printErr << "Couldn't start the tracking" << endl;
        return false;
    }
    if (not input.startImaging()) {
        printErr << "Couldn't start imaging" << endl;
        return false;
    }
    shared_ptr<TrackingInformation> trackingInformation = m_objectLocater.trackingInformation();
    // find object positions that define the plane
    Vec3f topLeft, topRight, bottomLeft;
    shared_ptr<CameraImage> cameraImage = input.cameraImage();
    if (not cameraImage->waitForNewImage()) {
        printErr << "Did not receive any images" << endl;
        return false;
    }
    namedWindow("Image View");
    locateObjectPoint("Top left", trackingInformation, cameraImage, topLeft);
    locateObjectPoint("Top right", trackingInformation, cameraImage, topRight);
    locateObjectPoint("Bottom left", trackingInformation, cameraImage, bottomLeft);
    destroyWindow("Image View");
    input.stopImaging();
    // find reference element transformation
    Mat referenceTransformation;
    locateReferenceElement(trackingInformation, referenceTransformation);
    m_objectLocater.stopTracking();

    // calculate object points
    vector<Point3f> calculatedObjectPoints;
    Vec3f xVector = (topRight - topLeft) / m_xVectorScale;
    Vec3f yVector = (bottomLeft - topLeft) / m_yVectorScale;
    Vec3f origin = topLeft + m_patternOriginOffset.x * xVector + m_patternOriginOffset.y * yVector;
    if (not patternObject.objectPointsOnPlane(origin, xVector, yVector, calculatedObjectPoints)) {
        printErr << "Determined plane vectors length didn't match pattern spacing" << endl;
        return false;
    }

    // extract required object points for pose estimation
    vector<Point3f> objectPoints;
    vector<Point3f> cornerObjectPoints;
    for (size_t cornerIdx : cornerIndices) {
        Point3f const &corner = calculatedObjectPoints.at(cornerIdx);
        cornerObjectPoints.push_back(corner);
    }
    if (m_flag == SOLVEPNP_P3P or m_flag == SOLVEPNP_AP3P) {
        objectPoints = cornerObjectPoints; // some pose estimation algorithms require 4 points
    } else {
        objectPoints = calculatedObjectPoints; // other algorithms can deal with more points
    }

    // perform pose estimation to calculate the camera transformation
    Mat cameraTransformation;
    if (not calculateCameraTransformation(
            imagePoints, calculatedObjectPoints, intrinsicCameraParameters, cameraTransformation)) {
        printErr << "Couldn't calculate camera transformation" << endl;
        return false;
    }

    // calculate the transformation from the reference element to the camera
    referenceToCameraTransformation = referenceTransformation.inv() * cameraTransformation;

    return true;
}

void
TransformationDetermination::validateSettings() {
    m_settingsValid = true;

    if (not m_objectLocater.settingsValid()) {
        printErr << "Invalid object locating method" << endl;
        m_settingsValid = false;
    }

    if (not m_imagePointExtractor.settingsValid()) {
        printErr << "Invalid image point extraction method" << endl;
        m_settingsValid = false;
    }

    if (m_flag < 0 or m_flag >= SOLVEPNP_MAX_COUNT) {
        printErr << "Invalid pose estimation method" << endl;
        m_settingsValid = false;
    }

    if (m_xVectorScale <= 0.0f) {
        printErr << "Invalid x vector scale" << endl;
        m_settingsValid = false;
    }

    if (m_yVectorScale <= 0.0f) {
        printErr << "Invalid y vector scale" << endl;
        m_settingsValid = false;
    }

    if (m_cooldownDuration <= 0.0) {
        printErr << "Invalid cooldown duration" << endl;
        m_settingsValid = false;
    }

    if (m_stillDelay <= 0.0) {
        printErr << "Invalid still delay" << endl;
        m_settingsValid = false;
    }

    if (m_minimalPositionCount < 1) {
        printErr << "Invalid minimal position count" << endl;
        m_settingsValid = false;
    }

    if (m_moveDistance <= 0.0) {
        printErr << "Invalid move distance" << endl;
        m_settingsValid = false;
    }
}

bool
TransformationDetermination::calculateCameraTransformation(
    const std::vector<Point2f> &imagePoints,
    const std::vector<Point3f> &objectPoints,
    const IntrinsicCameraParameters &intrinsicCameraParameters,
    Mat &cameraTransformation) {

    Mat rotationVector = Mat(3, 1, CV_32F); // output rotation vector
    Mat translationVector = Mat(3, 1, CV_32F);
    if (not solvePnP(objectPoints,
                     imagePoints,
                     intrinsicCameraParameters.cameraMatrix,
                     intrinsicCameraParameters.distortionCoefficients,
                     rotationVector,
                     translationVector,
                     false,
                     m_flag)) {
        printErr << "Couldn't perform pose estimation" << endl;
        return false;
    }

    cameraTransformation = Mat::eye(4, 4, CV_32F);
    Mat rotationMatrix;
    Rodrigues(rotationVector, rotationMatrix);
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            cameraTransformation.at<float>(row, col) = rotationMatrix.at<float>(row, col);
        }
    }

    for (int i = 0; i < 3; ++i) {
        cameraTransformation.at<float>(i, 3) = translationVector.at<float>(i);
    }

    return true;
}

void
TransformationDetermination::locateObjectPoint(
    const string &pointName,
    std::shared_ptr<TrackingInformation> trackingInformation,
    std::shared_ptr<CameraImage> cameraImage,
    Vec3f &objectPoint) {

    double imageTimeStamp = 0.0;
    Mat image;
    duration<double> timeSinceEpoch = high_resolution_clock::now().time_since_epoch();
    double startTime = timeSinceEpoch.count();
    double currentTime = 0.0;
    while (currentTime - startTime < m_cooldownDuration) {
        timeSinceEpoch = high_resolution_clock::now().time_since_epoch();
        currentTime = timeSinceEpoch.count();

        cameraImage->currentImage(image, imageTimeStamp);

        int countDown = static_cast<int>(ceil(m_cooldownDuration - currentTime + startTime));
        drawMessage(image, to_string(countDown), {255, 255, 255});

        imshow("Image View", image);
        waitKey(1);
    }

    Vec3f referencePosition, currentPosition, averagePosition;
    int positions = 0;
    double referenceTime = 0.0, positionTimeStamp = 0.0;
    while (true) {
        timeSinceEpoch = high_resolution_clock::now().time_since_epoch();
        currentTime = timeSinceEpoch.count();
        cameraImage->currentImage(image, imageTimeStamp);
        double oldPositionTimeStamp = positionTimeStamp;
        trackingInformation->pointer(currentPosition, positionTimeStamp);
        if (oldPositionTimeStamp < positionTimeStamp and currentTime - positionTimeStamp < 1) {
            ++positions;
            averagePosition = (averagePosition * (positions - 1) + currentPosition) / positions;
            if (positions == 1) {
                referencePosition = currentPosition;
            }
        } else if (positions == 0 or currentTime - positionTimeStamp >= 1) {
            drawMessage(image, "No pointer position", {0, 0, 255});
            waitKey(1);
            continue;
        }

        bool movedFromReference = false;
        if (norm(referencePosition - currentPosition) >= m_moveDistance) {
            movedFromReference = true;
            referenceTime = positionTimeStamp;
            referencePosition = currentPosition;
            positions = 0;
        }

        if (movedFromReference) {
            drawMessage(image, pointName, {0, 0, 255});
        } else if (positionTimeStamp - referenceTime < m_stillDelay or
                   positions < m_minimalPositionCount) {
            drawMessage(image, pointName, {0, 255, 255});
        } else {
            drawMessage(image, "OK", {0, 255, 0});
            imshow("Image View", image);
            waitKey(200);
            objectPoint = averagePosition;
            return;
        }

        imshow("Image View", image);
        waitKey(1);
    }
}

void
TransformationDetermination::locateReferenceElement(
    std::shared_ptr<TrackingInformation> trackingInformation, Mat &referenceTransformation) {
    double timeStamp = 0.0;
    duration<double> timeSinceEpoch = high_resolution_clock::now().time_since_epoch();
    double currentTime = timeSinceEpoch.count();
    while (currentTime - timeStamp > 1) {
        timeSinceEpoch = high_resolution_clock::now().time_since_epoch();
        currentTime = timeSinceEpoch.count();
        trackingInformation->referenceElement(referenceTransformation, timeStamp);
        this_thread::sleep_for(1ms);
    }
}

void
read(const FileNode &node,
     TransformationDetermination &transformationDetermination,
     const TransformationDetermination &defaultValue) {
    if (node.empty()) {
        transformationDetermination = defaultValue;
    } else {
        transformationDetermination.readSettings(node);
    }
}
