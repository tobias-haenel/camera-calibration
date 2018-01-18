#include "IntrinsicCameraParameters.h"

using namespace cv;
using namespace std;

void
write(FileStorage &fs, const string &, const IntrinsicCameraParameters &intrinsicCameraParameters) {
    fs << "{"                                                                                    //
       << "CameraMatrix" << intrinsicCameraParameters.cameraMatrix                               //
       << "DistortionCoeffiecients" << intrinsicCameraParameters.distortionCoefficients          //
       << "StandardDeviationIntrinsics" << intrinsicCameraParameters.standardDeviationIntrinsics //
       << "PerViewErros" << intrinsicCameraParameters.perViewErrors                              //
       << "FocusValue" << intrinsicCameraParameters.focusValue                                   //
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

    node["CameraMatrix"] >> intrinsicCameraParameters.cameraMatrix;
    node["DistortionCoefficients"] >> intrinsicCameraParameters.distortionCoefficients;
    node["StandardDeviationIntrinsics"] >> intrinsicCameraParameters.standardDeviationIntrinsics;
    node["PerViewErrors"] >> intrinsicCameraParameters.perViewErrors;
    node["FocusValue"] >> intrinsicCameraParameters.focusValue;
}
