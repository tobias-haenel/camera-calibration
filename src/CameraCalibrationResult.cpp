#include "CameraCalibrationResult.h"

using namespace cv;
using namespace std;

void
write(FileStorage &fs, const string &, const CameraCalibrationResult &cameraCalibrationResult) {
    fs << "{"                                                                          //
       << "Camera_Matrix" << cameraCalibrationResult.cameraMatrix                      //
       << "Distortion_Coeffiecients" << cameraCalibrationResult.distortionCoefficients //
       << "}";
}

void
read(const FileNode &node,
     CameraCalibrationResult &cameraCalibrationResult,
     const CameraCalibrationResult &defaultValue) {
    if (node.empty()) {
        cameraCalibrationResult = defaultValue;
        return;
    }

    node["Camera_Matrix"] >> cameraCalibrationResult.cameraMatrix;
    node["Distortion_Coefficients"] >> cameraCalibrationResult.distortionCoefficients;
}
