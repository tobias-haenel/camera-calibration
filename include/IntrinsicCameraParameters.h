#pragma once

#include <opencv2/core.hpp>

/**
 * @brief The IntrinsicCameraParameters struct represents the all results that are determined from a
 * camera calibration
 */
struct IntrinsicCameraParameters {
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
 * @brief OpenCV function for writing a IntrinsicCameraParameters to a FileStorage
 * @param fs FileStorage where the IntrinsicCameraParameters is stored
 * @param intrinsicCameraParameters Camera calibration result
 */
void
write(cv::FileStorage &fs,
      const std::string &,
      const IntrinsicCameraParameters &intrinsicCameraParameters);

/**
 * @brief OpenCV function for reading a IntrinsicCameraParameters from a FileNode.
 * @param node FileNode where the IntrinsicCameraParameters is read from
 * @param intrinsicCameraParameters IntrinsicCameraParameters that is changed acording to the file
 * contents
 * @param defaultValue Default value for the IntrinsicCameraParameters if the FileNode isn't valid
 */
void
read(const cv::FileNode &node,
     IntrinsicCameraParameters &intrinsicCameraParameters,
     const IntrinsicCameraParameters &defaultValue = IntrinsicCameraParameters());
