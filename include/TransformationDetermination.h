#pragma once

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include "ImagePointExtractor.h"
#include "ObjectLocater.h"

/**
 * @brief The TransformationDetermination class can determine the static transformation from the
 * coordinate system of a reference object to the camera coordinate system based on tracking
 * information, image point extraction and intrinsic camera paremeters.
 */
class TransformationDetermination {
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
     * @brief Determines the static transformation that converts the coordinate system defined by a
     * reference object to the coordinate system of the camera based on the.
     * @param intrinsicCameraParameters Intrinsic parameters of the camera
     * @param patternObject Pattern that is used to determine image points
     * @param input CameraInput that is used to obtain images
     * @param referenceToCameraTransform Result transformation
     * @param cameraToReferenceTransform Inverted result transformation
     * @param cameraToReferenceTran
     * @return boolean indicating success
     */
    bool
    determineConversionTransformation(IntrinsicCameraParameters const &intrinsicCameraParameters,
                                      ReferenceObject const &patternObject,
                                      CameraInput &input,
                                      cv::Mat &referenceToCameraTransform,
                                      cv::Mat &cameraToReferenceTransform);

    /**
     * @brief Performs a calculation of a reprojection error from known object points. Image points
     * are determined semi-automatically.
     * @param intrinsicCameraParameters Intrinsic parameters of the camera
     * @param referenceToCameraTransform Transform from the reference element to the camera
     * @param objectPoints Object points that are evaluated
     * @param input Camera input
     */
    void
    checkReprojectionError(IntrinsicCameraParameters const &intrinsicCameraParameters,
                           cv::Mat const &referenceToCameraTransform,
                           std::vector<cv::Point3f> objectPoints,
                           CameraInput &input);

private: // methods
    /**
     * @brief Checks if the read settings are valid and updates settingsValid() accordingly.
     */
    void
    validateSettings();

    /**
     * @brief Calculates the transformation to the camera coordinate system based on known point
     * correspondences and based on the intrisic camera parameters.
     * @param imagePoints Image points
     * @param objectPoints Object points
     * @param intrinsicCameraParameters Intrinsic parameters of the camera
     * @param reprojectedImagePoints Reprojected image points
     * @param cameraTransformation Transformation from the object point coordinate system to the
     * camera coordinate system
     * @return boolean indicating success
     */
    bool
    calculateCameraTransformation(std::vector<cv::Point2f> const &imagePoints,
                                  std::vector<cv::Point3f> const &objectPoints,
                                  IntrinsicCameraParameters const &intrinsicCameraParameters,
                                  std::vector<cv::Point2f> &reprojectedImagePoints,
                                  cv::Mat &cameraTransformation);

    /**
     * @brief Locates an object point with the pointer tip.
     * @param pointName Name of the point
     * @param trackingInformation Tracking information
     * @param cameraImage Camera input
     * @param objectPoint Output object point
     */
    void
    locateObjectPoint(std::string const &pointName,
                      std::shared_ptr<TrackingInformation> trackingInformation,
                      std::shared_ptr<CameraImage> cameraImage,
                      cv::Vec3f &objectPoint);

    /**
     * @brief Locates a reference element.
     * @param trackingInformation Tracking information
     * @param referenceToAnatomyTransform Output transformation from reference element to anatomy
     */
    void
    locateReferenceElement(std::shared_ptr<TrackingInformation> trackingInformation,
                           cv::Mat &referenceToAnatomyTransform);

private: // members
    /**
     * @brief Indicates if the settings are valid
     */
    bool m_settingsValid;

    /**
     * @brief Tracking method that is used to obtain the necessary tracking information.
     */
    ObjectLocater m_objectLocater;

    /**
     * @brief Extraction method that is used to obtain the points in image space from the input
     */
    ImagePointExtractor m_imagePointExtractor;

    /**
     * @brief Settings flag that is passed to the OpenCV function that calculates part of the result
     */
    int m_flag = -1;

    /**
     * @brief Length from top left to top right divided by the width of a pattern column
     */
    float m_xVectorScale = -1;

    /**
     * @brief Length from the top left to bottom left position divided by the height of a pattern
     * row
     */
    float m_yVectorScale = -1;

    /**
     * @brief Offset of the origin from the top left position described in units of the x and y
     * vector
     */
    cv::Point2f m_patternOriginOffset;

    /**
     * @brief Time (in s) that is requiered until a new position is requested from the user
     */
    double m_cooldownDuration = -1;

    /**
     * @brief Time (in s) that is required until a still pointer is treated as not moving
     */
    double m_stillDelay = -1;

    /**
     * @brief Minimum amount of positions until an average position is computed
     */
    int m_minimalPositionCount = -1;

    /**
     * @brief Distance barrier (in mm) that is considerd as movement
     */
    double m_moveDistance = -1.0;
};

/**
 * @brief OpenCV function for reading a TransformationDetermination from a FileNode.
 * @param node FileNode where the settings of the TransformationDetermination are read from
 * @param transformationDetermination TransformationDetermination that is changed acording to the
 * file contents
 * @param defaultValue Default value for the TransformationDetermination if the FileNode isn't valid
 */
void
read(const cv::FileNode &node,
     TransformationDetermination &transformationDetermination,
     const TransformationDetermination &defaultValue = TransformationDetermination());
