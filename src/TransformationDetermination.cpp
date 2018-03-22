#include "TransformationDetermination.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/photo.hpp>

#include <chrono>
#include <limits>

#include "Utility.h"

#include "ThreadSafePrinter.h"

using namespace std;
using namespace std::chrono;
using namespace cv;

Mat
rot2euler(const Mat &rotationMatrix) {
    Mat euler(3, 1, CV_64F);

    double m00 = rotationMatrix.at<double>(0, 0);
    double m02 = rotationMatrix.at<double>(0, 2);
    double m10 = rotationMatrix.at<double>(1, 0);
    double m11 = rotationMatrix.at<double>(1, 1);
    double m12 = rotationMatrix.at<double>(1, 2);
    double m20 = rotationMatrix.at<double>(2, 0);
    double m22 = rotationMatrix.at<double>(2, 2);

    double x, y, z;

    // Assuming the angles are in radians.
    if (m10 > 0.998) { // singularity at north pole
        x = 0;
        y = CV_PI / 2;
        z = atan2(m02, m22);
    } else if (m10 < -0.998) { // singularity at south pole
        x = 0;
        y = -CV_PI / 2;
        z = atan2(m02, m22);
    } else {
        x = atan2(-m12, m11);
        y = asin(m10);
        z = atan2(-m20, m00);
    }

    euler.at<double>(0) = x;
    euler.at<double>(1) = y;
    euler.at<double>(2) = z;

    return euler;
}

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
    } else if (poseEstimationMethod == "EPNP") {
        m_flag = SOLVEPNP_EPNP;
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
    Mat &referenceToCameraTransform,
    Mat &cameraToReferenceTransform) {

    if (not m_objectLocater.openConnection()) {
        printErr << "Couldn't open the tracking connection" << endl;
        return false;
    }

    shared_ptr<TrackingInformation> trackingInformation = m_objectLocater.trackingInformation();
    if (not trackingInformation->waitForInitialisiation() ||
        trackingInformation->connectionState() != TrackingInformation::ConnectionState::Active) {
        printErr << "Connection wasn't accepted" << endl;
        return false;
    }

    // find image points of still pattern object
    vector<vector<Point2f>> referenceObjectPointSets;
    Size imageSize;
    int focusValue;
    if (not m_imagePointExtractor.findReferenceObjectImagePoints(
            referenceObjectPointSets, imageSize, focusValue, patternObject, input)) {
        printErr << "Couldn't find image points" << endl;
        return false;
    }

    if (referenceObjectPointSets.size() != 1) {
        printErr << "Didn't find the pattern exactly once" << endl;
        return false;
    }

    vector<Point2f> &foundReferenceObjectImagePoints = referenceObjectPointSets.at(0);

    // extract required image points for pose estimation
    vector<Point2f> imagePoints;
    size_t patternColumns = static_cast<size_t>(patternObject.boardSize().width);
    size_t pointCount = foundReferenceObjectImagePoints.size();
    size_t cornerIndices[4]{0, patternColumns - 1, pointCount - 1, pointCount - patternColumns};
    vector<Point2f> cornerImagePoints;
    for (size_t cornerIdx : cornerIndices) {
        Point2f const &corner = foundReferenceObjectImagePoints.at(cornerIdx);
        cornerImagePoints.push_back(corner);
    }
    if (m_flag == SOLVEPNP_P3P or m_flag == SOLVEPNP_AP3P) {
        imagePoints = cornerImagePoints; // some pose estimation algorithms require 4 points
    } else {
        imagePoints = foundReferenceObjectImagePoints; // other algorithms can deal with more points
    }

    // locate necessary points/elements
    if (not m_objectLocater.startTracking()) {
        printErr << "Couldn't start the tracking" << endl;
        return false;
    }
    // find object positions that define the plane
    Vec3f topLeft, topRight, bottomLeft; //, high;
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

    // find reference element transformation
    Mat referenceToAnatomyTransform;
    locateReferenceElement(trackingInformation, referenceToAnatomyTransform);

    {
        FileStorage file{"reference-to-anatomy-transform.xml", FileStorage::WRITE};
        file << "ReferenceToAnatomy" << referenceToAnatomyTransform;
    }

    // calculate object points
    vector<Point3f> calculatedObjectPoints;
    Vec3f planeXVector = (topRight - topLeft) / m_xVectorScale;
    Vec3f planeYVector = (bottomLeft - topLeft) / m_yVectorScale;
    Vec3f planeOrigin =
        topLeft + m_patternOriginOffset.x * planeXVector + m_patternOriginOffset.y * planeYVector;
    if (not patternObject.objectPointsOnPlane(
            planeOrigin, planeXVector, planeYVector, calculatedObjectPoints)) {
        printErr << "Determined plane vectors length didn't match pattern spacing" << endl;
        return false;
    }

    {
        FileStorage file{"object-points.xml", FileStorage::WRITE};
        file << "ObjectPointCount" << static_cast<int>(calculatedObjectPoints.size());
        for (size_t i = 0; i < calculatedObjectPoints.size(); ++i) {
            file << "ObjectPoint" + to_string(i) << calculatedObjectPoints.at(i);
        }
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
    Mat anatomyToCameraTransform;
    vector<Point2f> reprojectedImagePoints;
    if (not calculateCameraTransformation(imagePoints,
                                          calculatedObjectPoints,
                                          intrinsicCameraParameters,
                                          reprojectedImagePoints,
                                          anatomyToCameraTransform)) {
        printErr << "Couldn't calculate anatomy to camera transformation" << endl;
        return false;
    }

    float poseRms = 0.0;
    for (size_t i = 0; i < foundReferenceObjectImagePoints.size(); ++i) {
        Point2f poseImagePoint = foundReferenceObjectImagePoints.at(i);
        Point2f reprojectedPoseImagePoint = reprojectedImagePoints.at(i);
        Point2f v = poseImagePoint - reprojectedPoseImagePoint;
        poseRms += v.x * v.x + v.y * v.y;
    }
    poseRms /= static_cast<float>(foundReferenceObjectImagePoints.size());
    poseRms = sqrt(poseRms);
    Mat eulerAngles = rot2euler(anatomyToCameraTransform.inv());
    Mat origin = Mat::zeros(4, 1, CV_64F);
    origin.at<double>(3) = 1.0;

    Mat cameraPosition = anatomyToCameraTransform.inv() * origin;
    Mat referencePosition = referenceToAnatomyTransform * origin;

    // calculate the transformation from the reference element to the camera
    referenceToCameraTransform = anatomyToCameraTransform * referenceToAnatomyTransform;
    cameraToReferenceTransform = referenceToCameraTransform.inv();

    {
        ThreadSafePrinter printer{cout};
        printer.precision(10);

        printer << scientific //
                << "Rotation angle x (deg): " << eulerAngles.at<double>(0) * 180.0 / M_PI << endl
                << "Rotation angle y (deg): " << eulerAngles.at<double>(1) * 180.0 / M_PI << endl
                << "Rotation angle z (deg): " << eulerAngles.at<double>(2) * 180.0 / M_PI << endl
                << "Position x (mm): " << cameraPosition.at<double>(0) << endl
                << "Position y (mm): " << cameraPosition.at<double>(1) << endl
                << "Position z (mm): " << cameraPosition.at<double>(2) << endl
                << "Reprojection error (px): " << poseRms << endl;
    }

    double imageTimeStamp = 0.0;
    int imageFocusValue;
    Mat image;

    namedWindow("Reprojected Pointer Image");
    do {
        double timeStamp;
        Vec3f pointerPos;
        Mat refToAna;
        trackingInformation->pointer(pointerPos, timeStamp);
        trackingInformation->referenceElement(refToAna, timeStamp);

        Mat anaToRef = refToAna.inv();
        Mat anaToCamera = referenceToCameraTransform * anaToRef;
        Mat rMat = Mat(anaToCamera, Range(0, 3), Range(0, 3));
        Mat tVec = Mat(anaToCamera, Range(0, 3), Range(3, 4));
        Mat rVec;
        Rodrigues(rMat, rVec);

        vector<Vec3f> objectPoints = {pointerPos};
        vector<Vec2f> imagePoints;
        projectPoints(objectPoints,
                      rVec,
                      tVec,
                      intrinsicCameraParameters.cameraMatrix,
                      intrinsicCameraParameters.distortionCoefficients,
                      imagePoints);

        cameraImage->currentImage(image, imageTimeStamp, imageFocusValue);

        Rect rect(Point(), image.size());
        Point pointerImagePoint{imagePoints.at(0)};

        if (rect.contains(pointerImagePoint)) {
            drawMarker(image, pointerImagePoint, Scalar(0, 0, 255));
        }

        imshow("Reprojected Pointer Image", image);
    } while (waitKey(1) != '2');
    destroyWindow("Reprojected Pointer Image");

    input.stopImaging();

    m_objectLocater.stopTracking();
    m_objectLocater.closeConnection();

    return true;
}

static void
onMouse(int event, int x, int y, int, void *userData) {
    vector<Point2f> *imagePoints = static_cast<vector<Point2f> *>(userData);
    if (event == EVENT_LBUTTONDOWN) {
        imagePoints->push_back(Point2f(x, y));
        printOut << "Added image point (" << x << "," << y << ")" << endl;
    }
}

void
TransformationDetermination::checkReprojectionError(
    const IntrinsicCameraParameters &intrinsicCameraParameters,
    const Mat &referenceToCameraTransform,
    std::vector<Point3f> objectPoints,
    CameraInput &input) {

    if (not m_objectLocater.openConnection()) {
        printErr << "Couldn't open the tracking connection" << endl;
        return;
    }

    shared_ptr<TrackingInformation> trackingInformation = m_objectLocater.trackingInformation();
    if (not trackingInformation->waitForInitialisiation() ||
        trackingInformation->connectionState() != TrackingInformation::ConnectionState::Active) {
        printErr << "Connection wasn't accepted" << endl;
        return;
    }

    for (size_t i = 0; i < objectPoints.size(); ++i) {
        printOut << "ObjectPoint" << i << ": " << objectPoints.at(i) << endl;
    }

    shared_ptr<CameraImage> cameraImage = input.cameraImage();
    if (not cameraImage->waitForNewImage()) {
        printErr << "Did not receive any images" << endl;
        return;
    }

    Mat image;
    double imageTimeStamp;
    int imageFocusValue;
    std::string windowName{"Image points"};
    vector<Point2f> imagePoints;
    namedWindow(windowName);
    setMouseCallback(windowName, onMouse, &imagePoints);
    do {
        cameraImage->currentImage(image, imageTimeStamp, imageFocusValue);
        for (Point2f imagePoint : imagePoints) {
            drawMarker(image, imagePoint, Scalar(0, 0, 255));
        }

        imshow(windowName, image);
    } while (waitKey(1) != 'n');

    Mat denoisedImg;
    cameraImage->currentImage(image, imageTimeStamp, imageFocusValue);
    imwrite("point-image.png", image);
    cvtColor(image, image, CV_RGB2GRAY);
    fastNlMeansDenoising(image, denoisedImg, 3);

    vector<Point2f> centroids;
    vector<Mat> masks;
    for (Point2i imagePoint : imagePoints) {
        Mat mask = Mat::zeros(denoisedImg.rows + 2, denoisedImg.cols + 2, CV_8U);
        floodFill(denoisedImg,
                  mask,
                  imagePoint,
                  255,
                  nullptr,
                  5,
                  5,
                  8 | (255 << 8) | FLOODFILL_MASK_ONLY | FLOODFILL_FIXED_RANGE);
        Rect maskROI{1, 1, denoisedImg.cols, denoisedImg.rows};
        Mat croppedMask = mask(maskROI);
        masks.push_back(croppedMask);
        Mat labels, statMat, centroidMat;
        int labelCount = connectedComponentsWithStats(croppedMask, labels, statMat, centroidMat);
        if (labelCount != 2) {
            printErr << "More than 2 labels!" << endl;
            return;
        }
        Point2f centroid;
        centroid.x = centroidMat.at<double>(1, 0);
        centroid.y = centroidMat.at<double>(1, 1);
        centroids.push_back(centroid);
    }

    {
        FileStorage file{"centroid-points.xml", FileStorage::WRITE};
        file << "CentroidPointCount" << static_cast<int>(centroids.size());
        for (size_t i = 0; i < centroids.size(); ++i) {
            file << "CentroidPoint" + to_string(i) << centroids.at(i);
        }
    }

    cvtColor(image, image, CV_GRAY2BGR);
    Mat imageClone = image.clone();
    for (Mat const &mask : masks) {
        imageClone.setTo(Scalar(100, 255, 100), mask);
    }
    image = 0.2 * imageClone + 0.8 * image;
    for (Point2f centroid : centroids) {
        drawMarker(image, centroid, Scalar(255, 0, 0));
    }
    imshow(windowName, image);
    while (waitKey() != 'n') {
    }
    destroyWindow(windowName);

    if (not m_objectLocater.startTracking()) {
        printErr << "Couldn't start the tracking" << endl;
        return;
    }

    windowName = "Reprojected points";
    namedWindow(windowName);

    int count = 0;
    double trackingTimeStamp = 0.0;
    Mat refToAna;
    vector<Point2f> reprojectedImagePoints;
    do {
        double oldTrackingTimeStamp = trackingTimeStamp;
        trackingInformation->referenceElement(refToAna, trackingTimeStamp);
        cameraImage->currentImage(image, imageTimeStamp, imageFocusValue);

        if (trackingTimeStamp != 0.0) {
            Mat anaToRef = refToAna.inv();
            Mat anaToCamera = referenceToCameraTransform * anaToRef;
            Mat rMat = Mat(anaToCamera, Range(0, 3), Range(0, 3));
            Mat tVec = Mat(anaToCamera, Range(0, 3), Range(3, 4));
            Mat rVec;
            Rodrigues(rMat, rVec);

            projectPoints(objectPoints,
                          rVec,
                          tVec,
                          intrinsicCameraParameters.cameraMatrix,
                          intrinsicCameraParameters.distortionCoefficients,
                          reprojectedImagePoints);

            vector<double> minimalDistances;
            for (Point2f centroid : centroids) {
                double minimalDistance = numeric_limits<double>::max();
                for (Point2f reprojectedImagePoint : reprojectedImagePoints) {
                    double currentDistance = norm(centroid - reprojectedImagePoint);
                    if (minimalDistance > currentDistance) {
                        minimalDistance = currentDistance;
                    }
                }
                minimalDistances.push_back(minimalDistance);
            }

            if (oldTrackingTimeStamp != trackingTimeStamp) {
                ++count;
                if (count <= 20) {
                    {
                        FileStorage file{"anatomy-to-reference-transform" + to_string(count) +
                                             ".xml",
                                         FileStorage::WRITE};
                        file << "AnatomyToReference" << anaToRef;
                    }
                    ThreadSafePrinter printer{cout};
                    printer.precision(10);
                    printer << "Nr. " << count << ":" << endl;
                    for (size_t i = 0; i < minimalDistances.size(); ++i) {
                        double minimalDistance = minimalDistances.at(i);
                        printer << scientific << "  Reprojection error for point " << i
                                << " (px): " << minimalDistance << endl;
                    }
                }
            }

            Rect rect(Point(), image.size());

            for (Point2f reprojectedImagePoint : reprojectedImagePoints) {
                if (rect.contains(reprojectedImagePoint)) {
                    for (size_t i = 0; i < centroids.size(); ++i) {
                        Point2f centroid = centroids.at(i);
                        double currentDistance = norm(centroids.at(i) - reprojectedImagePoint);
                        if (fabs(currentDistance - minimalDistances.at(i)) <= 0.01) {
                            line(image, reprojectedImagePoint, centroid, Scalar(255, 255, 255));
                        }
                    }

                    drawMarker(image, reprojectedImagePoint, Scalar(0, 0, 255));
                }
            }
        }

        for (Point2f centroid : centroids) {
            drawMarker(image, centroid, Scalar(255, 0, 0));
        }

        imshow(windowName, image);
    } while (waitKey(1) != 'n' || count < 20);
    destroyWindow(windowName);

    input.stopImaging();

    m_objectLocater.stopTracking();
    m_objectLocater.closeConnection();
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
    std::vector<Point2f> &reprojectedImagePoints,
    Mat &cameraTransformation) {

    Mat rotationVector = Mat::zeros(3, 1, CV_64F); // output rotation vector
    Mat translationVector = Mat::zeros(3, 1, CV_64F);
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

    projectPoints(objectPoints,
                  rotationVector,
                  translationVector,
                  intrinsicCameraParameters.cameraMatrix,
                  intrinsicCameraParameters.distortionCoefficients,
                  reprojectedImagePoints);

    cameraTransformation = Mat::eye(4, 4, CV_64F);
    Mat rotationMatrix;
    Rodrigues(rotationVector, rotationMatrix);
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            cameraTransformation.at<double>(row, col) = rotationMatrix.at<double>(row, col);
        }
    }

    for (int i = 0; i < 3; ++i) {
        cameraTransformation.at<double>(i, 3) = translationVector.at<double>(i);
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
    int imageFocusValue;
    Mat image;
    duration<double> timeSinceEpoch = high_resolution_clock::now().time_since_epoch();
    double startTime = timeSinceEpoch.count();
    double currentTime = 0.0;
    while (currentTime - startTime < m_cooldownDuration) {
        timeSinceEpoch = high_resolution_clock::now().time_since_epoch();
        currentTime = timeSinceEpoch.count();

        cameraImage->currentImage(image, imageTimeStamp, imageFocusValue);

        int countDown = static_cast<int>(ceil(m_cooldownDuration - currentTime + startTime));
        drawMessage(image, to_string(countDown), {255, 255, 255});

        imshow("Image View", image);
        waitKey(1);
    }

    Vec3f referencePosition, currentPosition, averagePosition;
    int positions = 0;
    timeSinceEpoch = high_resolution_clock::now().time_since_epoch();
    currentTime = timeSinceEpoch.count();
    double referenceTime = currentTime;
    double positionTimeStamp = 0.0;
    while (true) {
        timeSinceEpoch = high_resolution_clock::now().time_since_epoch();
        currentTime = timeSinceEpoch.count();
        cameraImage->currentImage(image, imageTimeStamp, imageFocusValue);
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
            drawMessage(image, pointName + " (M)", {0, 0, 255});
        } else if (positionTimeStamp - referenceTime < m_stillDelay or
                   positions < m_minimalPositionCount) {
            drawMessage(image, pointName + " (D)", {0, 255, 255});
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
    std::shared_ptr<TrackingInformation> trackingInformation, Mat &referenceToAnatomyTransform) {
    double timeStamp = 0.0;
    duration<double> timeSinceEpoch = high_resolution_clock::now().time_since_epoch();
    double currentTime = timeSinceEpoch.count();
    Mat transformation;
    while (currentTime - timeStamp > 1) {
        timeSinceEpoch = high_resolution_clock::now().time_since_epoch();
        currentTime = timeSinceEpoch.count();
        trackingInformation->referenceElement(transformation, timeStamp);
        this_thread::sleep_for(1ms);
    }
    referenceToAnatomyTransform = transformation;
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
