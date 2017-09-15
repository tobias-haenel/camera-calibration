#include "CameraCalibration.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <array>
#include <chrono>
#include <iostream>
#include <thread>

using namespace std;
using namespace std::chrono;
using namespace cv;

bool
CameraCalibration::readSettings(const FileNode &node) {
    node["ReferenceObject"] >> m_referenceObject;
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
CameraCalibration::calibrate(CameraCalibrationResult &result) {
    vector<vector<Point2f>> imagePoints;
    vector<Point3f> objectPointsOnce;
    vector<vector<Point3f>> objectPoints;
    Size imageSize;

    if (not m_imagePointExtractor.findImagePoints(imagePoints, imageSize, m_referenceObject)) {
        return false;
    }

    if (not m_referenceObject.objectPoints(objectPointsOnce)) {
        return false;
    } else {
        objectPoints.resize(imagePoints.size(), objectPointsOnce);
    }

    return calculateResult(imagePoints, objectPoints, imageSize, result);
}

void
CameraCalibration::showUndistortedInput(const CameraCalibrationResult &result) {
    m_imagePointExtractor.showUndistoredInput(result);
}

bool
CameraCalibration::calculateResult(const std::vector<std::vector<Point2f>> &imagePoints,
                                   const std::vector<std::vector<Point3f>> &objectPoints,
                                   const Size &imageSize,
                                   CameraCalibrationResult &result) {
    result.cameraMatrix = Mat::eye(3, 3, CV_64F);
    if (m_flag & CALIB_FIX_ASPECT_RATIO)
        result.cameraMatrix.at<double>(0, 0) = static_cast<double>(m_aspectRatio);
    result.distortionCoefficients = Mat::zeros(8, 1, CV_64F);

    vector<Mat> rvecs, tvecs;

    double rms = calibrateCamera(objectPoints,
                                 imagePoints,
                                 imageSize,
                                 result.cameraMatrix,
                                 result.distortionCoefficients,
                                 rvecs,
                                 tvecs,
                                 m_flag);

    cout << "Re-projection error reported by calibrateCamera: " << rms << endl;

    return checkRange(result.cameraMatrix) && checkRange(result.distortionCoefficients);
}

void
CameraCalibration::validateSettings() {
    m_settingsValid = true;

    if (not m_referenceObject.settingsValid()) {
        cerr << "Invalid reference object" << endl;
        m_settingsValid = false;
    }

    if (not m_imagePointExtractor.settingsValid()) {
        cerr << "Invalid image point extraction" << endl;
        m_settingsValid = false;
    }

    if (m_aspectRatio <= 0.0f) {
        cerr << "Invalid aspect ratio" << endl;
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
