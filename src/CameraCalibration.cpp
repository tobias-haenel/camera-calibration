#include "CameraCalibration.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <array>
#include <chrono>
#include <iostream>
#include <thread>

#include "ThreadSafePrinter.h"

using namespace std;
using namespace std::chrono;
using namespace cv;

bool
CameraCalibration::readSettings(const FileNode &node) {
    node["ImagePointExtraction"] >> m_imagePointExtractor;
    node["AspectRatio"] >> m_aspectRatio;
    node["AssumeZeroTangentialDistortion"] >> m_calibZeroTangentDist;
    node["FixPrincipalPointAtTheCenter"] >> m_calibFixPrincipalPoint;
    node["FixK1"] >> m_fixK1;
    node["FixK2"] >> m_fixK2;
    node["FixK3"] >> m_fixK3;
    node["FixK4"] >> m_fixK4;
    node["FixK5"] >> m_fixK5;
    node["FixK6"] >> m_fixK6;

    m_flag = CALIB_RATIONAL_MODEL;
    if (m_calibFixPrincipalPoint)
        m_flag |= CALIB_FIX_PRINCIPAL_POINT;
    if (m_calibZeroTangentDist)
        m_flag |= CALIB_ZERO_TANGENT_DIST;
    if (m_aspectRatio != 0.0f)
        m_flag |= CALIB_FIX_ASPECT_RATIO;
    if (m_fixK1)
        m_flag |= CALIB_FIX_K1;
    if (m_fixK2)
        m_flag |= CALIB_FIX_K2;
    if (m_fixK3)
        m_flag |= CALIB_FIX_K3;
    if (m_fixK4)
        m_flag |= CALIB_FIX_K4;
    if (m_fixK5)
        m_flag |= CALIB_FIX_K5;
    if (m_fixK6)
        m_flag |= CALIB_FIX_K6;

    validateSettings();
    return m_settingsValid;
}

bool
CameraCalibration::settingsValid() const {
    return m_settingsValid;
}

bool
CameraCalibration::calibrate(IntrinsicCameraParameters &result,
                             CameraInput &input,
                             ReferenceObject const &referenceObject) {
    vector<vector<Point2f>> imagePoints;
    vector<Point3f> objectPointsOnce;
    vector<vector<Point3f>> objectPoints;
    Size imageSize;

    if (not m_imagePointExtractor.findReferenceObjectImagePoints(
            imagePoints, imageSize, result.focusValue, referenceObject, input)) {
        return false;
    }

    referenceObject.objectPoints(objectPointsOnce);
    {
        FileStorage file{"object-points.xml", FileStorage::WRITE};
        file << "ObjectPointCount" << static_cast<int>(objectPointsOnce.size());
        for (size_t i = 0; i < objectPointsOnce.size(); ++i) {
            file << "ObjectPoint" + to_string(i) << objectPointsOnce.at(i);
        }
    }
    objectPoints.resize(imagePoints.size(), objectPointsOnce);

    return calculateResult(imagePoints, objectPoints, imageSize, result);
}

void
CameraCalibration::showUndistortedInput(const IntrinsicCameraParameters &result,
                                        CameraInput &input) {
    m_imagePointExtractor.showUndistoredInput(result, input);
}

bool
CameraCalibration::calculateResult(const std::vector<std::vector<Point2f>> &imagePoints,
                                   const std::vector<std::vector<Point3f>> &objectPoints,
                                   const Size &imageSize,
                                   IntrinsicCameraParameters &result) {
    double sensorWidth = 16;  // mm
    double sensorHeight = 12; // mm
    double focalLength = 60;  // mm

    result.cameraMatrix = Mat::eye(3, 3, CV_64F);
    if (m_flag & CALIB_FIX_ASPECT_RATIO)
        result.cameraMatrix.at<double>(0, 0) = static_cast<double>(m_aspectRatio);

    // use intrinsic guess for camera matrix
    result.cameraMatrix.at<double>(0, 0) = focalLength * imageSize.width / sensorWidth;
    result.cameraMatrix.at<double>(1, 1) = focalLength * imageSize.height / sensorHeight;
    result.cameraMatrix.at<double>(0, 2) = imageSize.width * 0.5;
    result.cameraMatrix.at<double>(1, 2) = imageSize.height * 0.5;

    result.distortionCoefficients = Mat::zeros(8, 1, CV_64F);

    result.standardDeviationIntrinsics = Mat::zeros(18, 1, CV_64F);

    result.perViewErrors = Mat::zeros(static_cast<int>(imagePoints.size()), 1, CV_64F);

    double rms = calibrateCamera(objectPoints,
                                 imagePoints,
                                 imageSize,
                                 result.cameraMatrix,
                                 result.distortionCoefficients,
                                 noArray(),
                                 noArray(),
                                 result.standardDeviationIntrinsics,
                                 noArray(),
                                 result.perViewErrors,
                                 m_flag | CALIB_USE_INTRINSIC_GUESS);

    bool ok = checkRange(result.cameraMatrix) && checkRange(result.distortionCoefficients);
    if (not ok) {
        return false;
    }

    double fovx, fovy, aspectRatio;
    Point2d principalPoint;
    double estimatedFocalLength;

    calibrationMatrixValues(result.cameraMatrix,
                            imageSize,
                            sensorWidth,
                            sensorHeight,
                            fovx,
                            fovy,
                            estimatedFocalLength,
                            principalPoint,
                            aspectRatio);

    ThreadSafePrinter printer{cout};
    printer.precision(10);
    printer << scientific << "Focal length (mm): " << estimatedFocalLength << endl
            << "Focal point x (px): " << result.cameraMatrix.at<double>(0, 2) << endl
            << "Focal point y (px): " << result.cameraMatrix.at<double>(1, 2) << endl
            << "Reprojection error(px): " << rms << endl;

    return true;
}

void
CameraCalibration::validateSettings() {
    m_settingsValid = true;

    if (not m_imagePointExtractor.settingsValid()) {
        printErr << "Invalid image point extraction method" << endl;
        m_settingsValid = false;
    }

    if (m_aspectRatio <= 0.0f) {
        printErr << "Invalid aspect ratio" << endl;
        m_settingsValid = false;
    }
}

void
read(const FileNode &node,
     CameraCalibration &cameraCalibration,
     const CameraCalibration &defaultValue) {
    if (node.empty()) {
        cameraCalibration = defaultValue;
    } else {
        cameraCalibration.readSettings(node);
    }
}
