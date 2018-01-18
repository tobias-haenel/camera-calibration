#include "ReferenceObject.h"

#include <iostream>
#include <string>

#include "ThreadSafePrinter.h"

using namespace cv;
using namespace std;

bool
ReferenceObject::readSettings(const FileNode &node) {
    std::string patternToUse;
    node["BoardSizeWidth"] >> m_boardSize.width;
    node["BoardSizeHeight"] >> m_boardSize.height;
    node["CalibrationPattern"] >> patternToUse;
    node["SquareSize"] >> m_spacing;

    if (patternToUse == "Chessboard") {
        m_calibrationPattern = PatternType::Chessboard;
    } else if (patternToUse == "CirclesGrid") {
        m_calibrationPattern = PatternType::CirclesGrid;
    } else {
        m_calibrationPattern = PatternType::Invalid;
    }

    validateSettings();
    return m_settingsValid;
}

bool
ReferenceObject::settingsValid() const {
    return m_settingsValid;
}

Size
ReferenceObject::boardSize() const {
    return m_boardSize;
}

ReferenceObject::PatternType
ReferenceObject::calibrationPattern() const {
    return m_calibrationPattern;
}

void
ReferenceObject::objectPoints(std::vector<Point3f> &objectPoints) const {
//    objectPointsOnPlane({0, m_spacing * (m_boardSize.height - 1), 0},
//                        {m_spacing, 0, 0},
//                        {0, -m_spacing, 0},
//                        objectPoints);
    objectPointsOnPlane({0, 0, 0}, {m_spacing, 0, 0}, {0, m_spacing, 0}, objectPoints);
}

bool
ReferenceObject::objectPointsOnPlane(const Vec3f &origin,
                                     const Vec3f &xVector,
                                     const Vec3f &yVector,
                                     std::vector<Point3f> &objectPoints) const {
    objectPoints.clear();

    float xLength = static_cast<float>(norm(xVector));
    if (fabs(1.0f - xLength / m_spacing) >= 0.1f) {
        printErr << "Length of x vector deviated more than 10% from specified length" << endl;
        return false;
    }

    float yLength = static_cast<float>(norm(yVector));
    if (fabs(1.0f - yLength / m_spacing) >= 0.1f) {
        printErr << "Length of y vector deviated more than 10% from specified length" << endl;
        return false;
    }

    switch (m_calibrationPattern) {
    case PatternType::Invalid:
        break;
    case PatternType::Chessboard:
    case PatternType::CirclesGrid: {
        for (int i = 0; i < m_boardSize.height; ++i) {
            for (int j = 0; j < m_boardSize.width; ++j) {
                Point3f objectPoint = origin + j * xVector + i * yVector;
                objectPoints.push_back(objectPoint);
            }
        }
        break;
    }
    }

    return true;
}

float
ReferenceObject::spacing() const {
    return m_spacing;
}

void
ReferenceObject::validateSettings() {
    m_settingsValid = true;
    if (m_boardSize.width <= 0 or m_boardSize.height <= 0) {
        printErr << "Invalid board size: " << m_boardSize.width << ", " << m_boardSize.height
                 << endl;
        m_settingsValid = false;
    }
    if (m_spacing <= 0.0f) {
        printErr << "Invalid square size: " << m_spacing << endl;
        m_settingsValid = false;
    }
    if (m_calibrationPattern == PatternType::Invalid) {
        printErr << "Invalid pattern type: unknown" << endl;
        m_settingsValid = false;
    }
}

void
read(const cv::FileNode &node,
     ReferenceObject &referenceObject,
     const ReferenceObject &defaultValue) {
    if (node.empty()) {
        referenceObject = defaultValue;
    } else {
        referenceObject.readSettings(node);
    }
}
