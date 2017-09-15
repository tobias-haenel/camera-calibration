#pragma once

#include <opencv2/core.hpp>

/**
 * @brief The CameraCalibrationResult struct represents the all results that are determined from a
 * camera calibration
 */
struct CameraCalibrationResult {
    /**
     * @brief Camera matrix that maps world space coordinates to image space coordinates
     */
    cv::Mat cameraMatrix;

    /**
     * @brief Distortion coefficients that can be used to map undistorted image space coordinates to
     * distorted image space coordinates
     */
    cv::Mat distortionCoefficients;
};

/**
 * @brief OpenCV function for writing a CameraCalibrationResult to a FileStorage
 * @param fs FileStorage where the CameraCalibrationResult is stored
 * @param cameraCalibrationResult Camera calibration result
 */
void
write(cv::FileStorage &fs,
      const std::string &,
      const CameraCalibrationResult &cameraCalibrationResult);

/**
 * @brief OpenCV function for reading a CameraCalibrationResult from a FileNode.
 * @param node FileNode where the CameraCalibrationResult is read from
 * @param cameraCalibrationResult CameraCalibrationResult that is changed acording to the file
 * contents
 * @param defaultValue Default value for the CameraCalibrationResult if the FileNode isn't valid
 */
void
read(const cv::FileNode &node,
     CameraCalibrationResult &cameraCalibrationResult,
     const CameraCalibrationResult &defaultValue = CameraCalibrationResult());
