#pragma once

#include <opencv2/core.hpp>

/**
 * @brief The ReferenceObject class describes a planar reference object that is used for camera
 * calibration.
 */
class ReferenceObject {
public:
    /**
     * @brief The PatternType enum describes possible pattern types for Reference Objects.
     */
    enum class PatternType {
        Invalid,
        Chessboard,
        CirclesGrid,
        AsymmetricCirclesGrid,
    };

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
     * @brief Gets the size of the board (number of items by width and height).
     * @return Size with corresponding item counts
     */
    cv::Size
    boardSize() const;

    /**
     * @brief Gets the type of pattern that is used on the reference object.
     * @return PatternType
     */
    PatternType
    calibrationPattern() const;

    /**
     * @brief Gets the points of the object in the object space.
     * @param objectPoints ObjectPoints that are calculated
     * @return boolean indicating success
     */
    bool
    objectPoints(std::vector<cv::Point3f> &objectPoints) const;

    /**
     * @brief Gets the distance from one pattern point on the reference object to its neighbours.
     * @note If boardSize() is PatternType::AsymetricCirclesGrid, this is half the distance to row
     * neighbours
     * @return floating point value measured in a user defined unit
     */
    float
    squareSize() const;

private: // methods
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
     * @brief Size of the board (number of items by width and height)
     */
    cv::Size m_boardSize = cv::Size(-1, -1);

    /**
     * @brief Type of pattern that is used on the reference object
     */
    PatternType m_calibrationPattern = PatternType::Invalid;

    /**
     * @brief Distance from one pattern point on the reference object to its neighbours
     * @note If boardSize() is PatternType::AsymetricCirclesGrid, this is half the distance to row
     * neighbours
     */
    float m_squareSize = 0.0;
};

/**
 * @brief OpenCV function for reading a ReferenceObject from a FileNode.
 * @param node FileNode where the settings of the ReferenceObject are read from
 * @param referenceObject ReferenceObject that is changed acording to the file contents
 * @param defaultValue Default value for the ReferenceObject if the FileNode isn't valid
 */
void
read(const cv::FileNode &node,
     ReferenceObject &referenceObject,
     const ReferenceObject &defaultValue = ReferenceObject());
