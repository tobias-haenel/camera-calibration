#include <iostream>

#include "CameraCalibration.h"

#include "ThreadSafePrinter.h"

using namespace cv;
using namespace std;

int
main(int argc, char *argv[]) {
    // Read settings file
    CameraCalibration calib;
    bool showUndistorted;
    const string inputSettingsFile = argc > 1 ? argv[1] : "settings.xml";
    {
        FileStorage fs{inputSettingsFile, FileStorage::READ}; // Read the settings
        if (!fs.isOpened()) {
            printOut << "Could not open the configuration file: \"" << inputSettingsFile << "\""
                     << endl;
            return -1;
        }

        fs["CameraCalibration"] >> calib;
        if (!calib.settingsValid()) {
            printOut << "Invalid input detected. Application stopping." << endl;
            return -1;
        }

        fs["ShowUndistorted"] >> showUndistorted;
    }

    // perform camera calibration
    CameraCalibrationResult calibResult;
    if (!calib.calibrate(calibResult)) {
        printOut << "Camera calibration did not succed. Application stopping." << endl;
        return -1;
    }

    // optional: show undistorted images
    if (showUndistorted) {
        calib.showUndistortedInput(calibResult);
    }

    // perform pose estimation
    // get image
    // get 3D world positions from NN
    // find features
    // start calculation

    // extract extrinsic parameters

    // calculate transformation from reference adapter to camera sensor

    // store intrinsic parameters, tracking transformation and algorithm statistics
    const string outputFile = argc > 2 ? argv[2] : "result.xml";
    {
        FileStorage fs{outputFile, FileStorage::WRITE}; // Write the results
        if (!fs.isOpened()) {
            printOut << "Could not open the output file: \"" << inputSettingsFile << "\"" << endl;
            return -1;
        }

        fs << "CameraCalibrationResult" << calibResult;
    }
    return 0;
}
