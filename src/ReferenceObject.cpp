#include "ReferenceObject.h"

#include <iostream>
#include <string>

using namespace cv;
using namespace std;

bool
ReferenceObject::readSettings(const FileNode &node) {
    std::string patternToUse;
    node["BoardSizeWidth"] >> m_boardSize.width;
    node["BoardSizeHeight"] >> m_boardSize.height;
    node["CalibrationPattern"] >> patternToUse;
    node["SquareSize"] >> m_squareSize;

    if (patternToUse == "Chessboard") {
        m_calibrationPattern = PatternType::Chessboard;
    } else if (patternToUse == "CirclesGrid") {
        m_calibrationPattern = PatternType::CirclesGrid;
    } else if (patternToUse == "AsymmetricCirclesGrid") {
        m_calibrationPattern = PatternType::AsymmetricCirclesGrid;
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

bool
ReferenceObject::objectPoints(std::vector<Point3f> &objectPoints) const {
    objectPoints.clear();

    if (not m_settingsValid) {
        cerr << "Can't calculate object points: settings invalid" << endl;
        return false;
    }

    switch (m_calibrationPattern) {
    case PatternType::Chessboard:
    case PatternType::CirclesGrid: {
        for (int i = 0; i < m_boardSize.height; ++i)
            for (int j = 0; j < m_boardSize.width; ++j)
                objectPoints.push_back(Point3f(j * m_squareSize, i * m_squareSize, 0));
        break;
    }
    case PatternType::AsymmetricCirclesGrid: {
        for (int i = 0; i < m_boardSize.height; i++)
            for (int j = 0; j < m_boardSize.width; j++)
                objectPoints.push_back(
                    Point3f((2 * j + i % 2) * m_squareSize, i * m_squareSize, 0));
        break;
    }
    default:
        break;
    }
    return true;
}

float
ReferenceObject::squareSize() const {
    return m_squareSize;
}

void
ReferenceObject::validateSettings() {
    m_settingsValid = true;
    if (m_boardSize.width <= 0 or m_boardSize.height <= 0) {
        cerr << "Invalid board size: " << m_boardSize.width << ", " << m_boardSize.height << endl;
        m_settingsValid = false;
    }
    if (m_squareSize <= 0.0f) {
        cerr << "Invalid square size: " << m_squareSize << endl;
        m_settingsValid = false;
    }
    if (m_calibrationPattern == PatternType::Invalid) {
        cerr << "Invalid pattern type: unknown" << endl;
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
