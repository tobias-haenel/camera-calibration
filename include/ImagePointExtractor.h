#pragma once

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include <condition_variable>
#include <mutex>
#include <vector>

#include "IntrinsicCameraParameters.h"
#include "ReferenceObject.h"
#include "CameraInput.h"

/**
 * @brief The ImagePointExtractor class can find image points on a reference object that are need
 * for camera calibration from various inputs.
 */
class ImagePointExtractor {
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
     * @brief Finds image points on a reference object from the currently set input.
     * @param imagePoints List of image points where each set of image points corresponds to one
     * input image
     * @param imageSize Size (width and height) of all images
     * @param referenceObject Reference Object that should be detected
     * @param input CameraInput that is used to obtain images
     * @return boolean indicating success
     */
    bool
    findImagePoints(std::vector<std::vector<cv::Point2f>> &imagePoints,
                    cv::Size &imageSize,
                    ReferenceObject const &referenceObject,
                    CameraInput &input) const;

    void
    showUndistoredInput(IntrinsicCameraParameters const &result, CameraInput &input);

private: // methods
    /**
     * @brief The Data struct contains all data that is required during runtime.
     */
    struct Data {
        ReferenceObject referenceObject;
        std::vector<std::vector<int>> imageCountGrid;
        std::vector<std::vector<cv::Point2i>> imageShapes;
        std::vector<cv::Point2i> referenceShape;
        double referenceTime = 0.0;
        bool cooldownActive = false;
        bool moving = false;
        bool gridCellFull = false;
        double pointsAddedTime = 0.0;
    };

    /**
     * @brief Helper method that updates statistics about found images and that adds a set of image
     * points to the desired list of image points if possible.
     * @param imagePoints List of image points where each set of image points corresponds to one
     * input image
     * @param data Runtime data
     * @param currentImagePoints Point set that should be added
     * @param imageSize Size of the image that contains the point set that should be added
     * @return boolean indicating success
     */
    bool
    addImagePoints(std::vector<std::vector<cv::Point2f>> &imagePoints,
                   Data &data,
                   const std::vector<cv::Point2f> &currentImagePoints,
                   const cv::Size &imageSize) const;

    /**
     * @brief Detects the feature points of the reference object in an image.
     * @param featurePoints Image points corresponding to feature locations
     * @param image Image where points should be detcted
     * @param data Runtime data
     * @return boolean indicating success
     */
    bool
    detectFeaturePoints(std::vector<cv::Point2f> &featurePoints,
                        cv::Mat const &image,
                        Data const &data) const;

    /**
     * @brief Determines the inner shape of the reference object pattern from some image points
     * that correspond to the objects feature locations.
     * @param shape Set of points at the margin of the reference object (polygon)
     * @param featurePoints Set of image points that correspond to feature locations
     * @param data Runtime data
     */
    void
    determineImageShape(std::vector<cv::Point2i> &shape,
                        std::vector<cv::Point2f> const &featurePoints,
                        Data const &data) const;

    /**
     * @brief Performs preprocessing of the input images according to the settings.
     * @param image Input image
     * @param featureImage Feature image that is created from input image
     */
    void
    determineFeatureImage(cv::Mat const &image, cv::Mat &featureImage) const;

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
     * @brief Indicates if the input should be thresholded after a conversion to grayscale
     */
    bool m_applyThreshold = false;

    /**
     * @brief Value that should be used for thresholding
     */
    double m_thresholdValue = 0.0;

    /**
     * @brief Time (in s) that is required until a still reference object is treated as not moving
     */
    double m_stillDelay = -1;

    /**
     * @brief Time (in s) that is requiered until the input images are considered again after a set
     * of image points was successfully added
     */
    double m_cooldownDuration = -1;

    /**
     * @brief Size (number of columns and rows) of the grid that has to be covered with image points
     * of the reference object
     */
    cv::Size m_gridSize = cv::Size(-1, -1);

    /**
     * @brief Amount of image point sets of the reference object that is required for each grid cell
     */
    int m_detectionsPerGridCell = -1;
};

/**
 * @brief OpenCV function for reading a ImagePointExtractor from a FileNode.
 * @param node FileNode where the settings of the ImagePointExtractor are read from
 * @param imagePointExtractor ImagePointExtractor that is changed acording to the file contents
 * @param defaultValue Default value for the ImagePointExtractor if the FileNode isn't valid
 */
void
read(const cv::FileNode &node,
     ImagePointExtractor &imagePointExtractor,
     const ImagePointExtractor &defaultValue = ImagePointExtractor());
