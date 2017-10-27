#include <iostream>

#include "CameraCalibration.h"
#include "CameraInput.h"
#include "TransformationDetermination.h"

#include "ThreadSafePrinter.h"

using namespace cv;
using namespace std;

int
main(int argc, char *argv[]) {
    // Read settings file
    CameraInput input;
    ReferenceObject referenceObject;
    CameraCalibration calib;
    TransformationDetermination transDeterm;
    bool showUndistorted;

    const string inputSettingsFile = argc > 1 ? argv[1] : "settings.xml";
    {
        FileStorage fs{inputSettingsFile, FileStorage::READ}; // Read the settings
        if (not fs.isOpened()) {
            printOut << "Could not open the configuration file: \"" << inputSettingsFile << "\""
                     << endl;
            return -1;
        }

        bool settingsOk = true;
        fs["CameraInput"] >> input;
        settingsOk &= input.settingsValid();
        fs["ReferenceObject"] >> referenceObject;
        settingsOk &= referenceObject.settingsValid();
        fs["CameraCalibration"] >> calib;
        settingsOk &= calib.settingsValid();
        fs["ShowUndistorted"] >> showUndistorted;
        fs["TransformationDetermination"] >> transDeterm;
        settingsOk &= transDeterm.settingsValid();

        if (not settingsOk) {
            printOut << "Invalid settings input detected. Application stopping." << endl;
            return -1;
        }
    }

    // perform camera calibration
    IntrinsicCameraParameters intrinsicParameters;
    if (not calib.calibrate(intrinsicParameters, input, referenceObject)) {
        printOut << "Camera calibration did not succed. Application stopping." << endl;
        return -1;
    }

    // optional: show undistorted images
    if (showUndistorted) {
        calib.showUndistortedInput(intrinsicParameters, input);
    }

    // deterime transformation from reference adapter to camera
    Mat conversionTransformation;
    if (not transDeterm.determineConversionTransformation(
            intrinsicParameters, referenceObject, input, conversionTransformation)) {
        printOut << "Couldn't determine the conversion transformation. Application stopping."
                 << endl;
        return -1;
    }

    // store intrinsic parameters, tracking transformation and algorithm statistics
    const string outputFile = argc > 2 ? argv[2] : "result.xml";
    {
        FileStorage fs{outputFile, FileStorage::WRITE}; // Write the results
        if (!fs.isOpened()) {
            printOut << "Could not open the output file: \"" << inputSettingsFile << "\"" << endl;
            return -1;
        }

        fs << "IntrinsicCameraParameters" << intrinsicParameters;
        fs << "ConversionTransformation" << conversionTransformation;
    }
    return 0;
}
