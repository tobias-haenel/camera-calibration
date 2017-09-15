#pragma once

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include <condition_variable>
#include <mutex>

#include "CameraCalibrationResult.h"
#include "ImagePointExtractor.h"
#include "ReferenceObject.h"

/**
 * @brief The CameraCalibration class can perform a camera calibration based on a reference object
 * and image point extraction from various inputs.
 */
class CameraCalibration {
public:
    /**
     * @brief Reads settings from an OpenCV FileNode.
     * @param node OpenCV FileNode
     * @return boolean indicating validity of the settings
     */
    bool
    readSettings(const cv::FileNode &node);

    /**
     * @brief Checks if read settings where valid.
     * @return boolean indicating validity
     */
    bool
    settingsValid() const;

    /**
     * @brief Performs the camera calibration.
     * @param result Result of the camera calibration
     * @return boolean indicating success
     */
    bool
    calibrate(CameraCalibrationResult &result);

    /**
     * @brief Shows the input in an undistorted way based on the calibration result
     * @param result Result of the camera calibration
     */
    void
    showUndistortedInput(CameraCalibrationResult const &result);

private: // methods
    /**
     * @brief Calculates the camera calibration result based on known poin correspondences between
     * image space and reference object space.
     * @param imagePoints Image points, each point set should correspond to one image
     * @param objectPoints Reference object points, each point set should correspond to one image
     * @param imageSize Size (width and height) of all images
     * @param result Result of the camera calibration
     * @return boolean indicating success
     */
    bool
    calculateResult(std::vector<std::vector<cv::Point2f>> const &imagePoints,
                    std::vector<std::vector<cv::Point3f>> const &objectPoints,
                    const cv::Size &imageSize,
                    CameraCalibrationResult &result);

    /**
     * @brief Checks if the read settings are valid and updates settingsValid() accordingly.
     */
    void
    validateSettings();

private: // members
    /**
     * @brief Indicates if the settings of this ReferenceObject are valid
     */
    bool m_settingsValid = false;

    /**
     * @brief Reference object that is used for calibration
     */
    ReferenceObject m_referenceObject;

    /**
     * @brief Extraction method that is used to obtain the points in image space from the input
     */
    ImagePointExtractor m_imagePointExtractor;

    /**
     * @brief Aspect ratio that should be assumed for the camera matrix, 0.0 means the aspect ratio
     * will be calculated
     */
    float m_aspectRatio = -1.0f;

    /**
     * @brief Indicates if the tangetial distortion should be assumed as zero
     */
    bool m_calibZeroTangentDist = false;

    /**
     * @brief Indicates if the principal point of the camera matrix should be assumed in the centre
     * of the image plane
     */
    bool m_calibFixPrincipalPoint = false;

    /**
     * @brief Indicates if the radial distortion coefficient k1 should be assumed as zero
     */
    bool m_fixK1 = false;

    /**
     * @brief Indicates if the radial distortion coefficient k2 should be assumed as zero
     */
    bool m_fixK2 = false;

    /**
     * @brief Indicates if the radial distortion coefficient k3 should be assumed as zero
     */
    bool m_fixK3 = false;

    /**
     * @brief Indicates if the radial distortion coefficient k4 should be assumed as zero
     */
    bool m_fixK4 = false;

    /**
     * @brief Indicates if the radial distortion coefficient k5 should be assumed as zero
     */
    bool m_fixK5 = false;

    /**
     * @brief Indicates if the radial distortion coefficient k6 should be assumed as zero
     */
    bool m_fixK6 = false;

    /**
     * @brief Settings flag that is passed to the OpenCV function that calculates the result
     */
    int m_flag = false;
};

/**
 * @brief OpenCV function for reading a CameraCalibration from a FileNode.
 * @param node FileNode where the settings of the CameraCalibration are read from
 * @param cameraCalibration CameraCalibration that is changed acording to the file contents
 * @param defaultValue Default value for the CameraCalibration if the FileNode isn't valid
 */
void
read(const cv::FileNode &node,
     CameraCalibration &cameraCalibration,
     const CameraCalibration &defaultValue = CameraCalibration());
