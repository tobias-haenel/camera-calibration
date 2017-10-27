#include "IntrinsicCameraParameters.h"

using namespace cv;
using namespace std;

void
write(FileStorage &fs, const string &, const IntrinsicCameraParameters &intrinsicCameraParameters) {
    fs << "{"                                                                          //
       << "Camera_Matrix" << intrinsicCameraParameters.cameraMatrix                      //
       << "Distortion_Coeffiecients" << intrinsicCameraParameters.distortionCoefficients //
       << "}";
}

void
read(const FileNode &node,
     IntrinsicCameraParameters &intrinsicCameraParameters,
     const IntrinsicCameraParameters &defaultValue) {
    if (node.empty()) {
        intrinsicCameraParameters = defaultValue;
        return;
    }

    node["Camera_Matrix"] >> intrinsicCameraParameters.cameraMatrix;
    node["Distortion_Coefficients"] >> intrinsicCameraParameters.distortionCoefficients;
}
