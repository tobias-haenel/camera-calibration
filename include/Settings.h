#pragma once

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

class Settings {
public:
    Settings() : goodInput(false) {}
    enum Pattern { NOT_EXISTING, CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };
    enum InputType { INVALID, CAMERA, VIDEO_FILE, IMAGE_LIST };

    void
    write(cv::FileStorage &fs) const;

    void
    read(const cv::FileNode &node);

    void
    validate();

    cv::Mat
    nextImage();

    static bool
    readStringList(const std::string &filename, std::vector<std::string> &l);

    static bool
    isListOfImages(const std::string &filename);

public:
    cv::Size boardSize;             // The size of the board -> Number of items by width and height
    Pattern calibrationPattern;     // One of the Chessboard, circles, or asymmetric circle pattern
    float squareSize;               // The size of a square in your defined unit
    int nrFrames;                   // The number of frames to use from the input for calibration
    float aspectRatio;              // The aspect ratio
    int delay;                      // In case of a video input
    bool writePoints;               // Write detected feature points
    bool writeExtrinsics;           // Write extrinsic parameters
    bool calibZeroTangentDist;      // Assume zero tangential distortion
    bool calibFixPrincipalPoint;    // Fix the principal point at the center
    bool flipVertical;              // Flip the captured images around the horizontal axis
    std::string outputFileName;     // The name of the file where to write
    bool showUndistorsed;           // Show undistorted images after calibration
    std::string input;              // The input ->
    bool useFisheye;                // use fisheye camera model for calibration
    bool fixK1;                     // fix K1 distortion coefficient
    bool fixK2;                     // fix K2 distortion coefficient
    bool fixK3;                     // fix K3 distortion coefficient
    bool fixK4;                     // fix K4 distortion coefficient
    bool fixK5;                     // fix K5 distortion coefficient

    int cameraID;
    std::vector<std::string> imageList;
    size_t atImageList;
    cv::VideoCapture inputCapture;
    InputType inputType;
    bool goodInput;
    int flag;

private:
    std::string patternToUse;
};
