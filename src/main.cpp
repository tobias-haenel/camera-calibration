#include <iostream>

#include <opencv2/core.hpp>

#include "CameraCalibration.h"
#include "CameraInput.h"
#include "TransformationDetermination.h"

#include "ThreadSafePrinter.h"

using namespace cv;
using namespace std;

int
main(int argc, char *argv[]) {
    const String keys =
        "{help h usage ?      |                        | print this message}"
        "{@settings           | settings.xml           | input settings file}"
        "{@calib              | camera_calibration.xml | input/output camera calibration file}"
        "{@transform          | transformation.xml     | input/output transformation}"
        "{show-undistorted s  |                        | show the undistored camera input}"
        "{calib-input c       |                        | use the camera calibration file as input}"
        "{transform-input t   |                        | use the transformation file as input}"
        "{no-transform n      |                        | stop after camera calibration}"
        "{reprojection r      |                        | calculate reprojection errors for object points} ";
    CommandLineParser clParser{argc, argv, keys};
    if (clParser.has("help")) {
        clParser.printMessage();
        return 0;
    }

    {
        ThreadSafePrinter printer{cout};
        printer << "Running:";
        for (int i = 0; i < argc; ++i) {
            printer << " " << argv[i];
        }
        printer << endl;
    }

    String inputSettingsFile = clParser.get<String>(0);
    String calibFile = clParser.get<String>(1);
    String transformFile = clParser.get<String>(2);
    bool showUndistorted = clParser.has("show-undistorted");
    bool useCalibFileAsInput = clParser.has("calib-input");
    bool useTransformFileAsInput = clParser.has("transform-input");
    bool noTransform = clParser.has("no-transform");
    bool performReprojectionErrorCalculation = clParser.has("reprojection");
    String objectPointsFile;
    if (performReprojectionErrorCalculation) {
        objectPointsFile = clParser.get<String>("reprojection");
    }

    if (not clParser.check()) {
        clParser.printErrors();
        return 0;
    }

    // Read settings file
    CameraInput input;
    ReferenceObject referenceObject;
    CameraCalibration calib;
    TransformationDetermination transDeterm;

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
        fs["TransformationDetermination"] >> transDeterm;
        settingsOk &= transDeterm.settingsValid();

        if (not settingsOk) {
            printOut << "Invalid settings input detected. Application stopping." << endl;
            return -1;
        }
    }

    // start imaging
    if (not input.startImaging()) {
        printErr << "Couldn't start imaging" << endl;
        return -1;
    }

    // perform camera calibration
    IntrinsicCameraParameters intrinsicParameters;
    if (!useCalibFileAsInput) {
        if (not calib.calibrate(intrinsicParameters, input, referenceObject)) {
            printOut << "Camera calibration did not succeed. Application stopping." << endl;
            return -1;
        }

        FileStorage fs{calibFile, FileStorage::WRITE}; // Write the results
        if (!fs.isOpened()) {
            printOut << "Could not create the output file: \"" << calibFile << "\"" << endl;
            return -1;
        }

        fs << "IntrinsicCameraParameters" << intrinsicParameters;
    } else {
        FileStorage fs{calibFile, FileStorage::READ}; // Read the results
        if (!fs.isOpened()) {
            printOut << "Could not open the input file: \"" << calibFile << "\"" << endl;
            return -1;
        }
        fs["IntrinsicCameraParameters"] >> intrinsicParameters;
    }

    // optional: show undistorted images
    if (showUndistorted) {
        calib.showUndistortedInput(intrinsicParameters, input);
    }

    if (noTransform) {
        return 0;
    }

    // determine transformation from reference adapter to camera
    Mat referenceToCameraTransform;
    Mat cameraToReferenceTransform;
    if (!useTransformFileAsInput) {
        if (!transDeterm.determineConversionTransformation(intrinsicParameters,
                                                           referenceObject,
                                                           input,
                                                           referenceToCameraTransform,
                                                           cameraToReferenceTransform)) {
            printOut << "Couldn't determine the conversion transformation. Application stopping."
                     << endl;
            return -1;
        }
        FileStorage fs{transformFile, FileStorage::WRITE}; // Write the results
        if (!fs.isOpened()) {
            printOut << "Could not create the output file: \"" << transformFile << "\"" << endl;
            return -1;
        }
        fs << "ReferenceToCameraTransformation" << referenceToCameraTransform;
        fs << "CameraToReferenceTransformation" << cameraToReferenceTransform;
    } else {
        FileStorage fs{transformFile, FileStorage::READ};
        if (!fs.isOpened()) {
            printOut << "Could not open the input file: \"" << transformFile << "\"" << endl;
            return -1;
        }
        fs["ReferenceToCameraTransformation"] >> referenceToCameraTransform;
        fs["CameraToReferenceTransformation"] >> cameraToReferenceTransform;
    }

    if (performReprojectionErrorCalculation) {
        vector<Point3f> objectPoints;
        int objectPointCount;

        FileStorage fs{objectPointsFile, FileStorage::READ};
        if (!fs.isOpened()) {
            printOut << "Could not open the input file: \"" << objectPointsFile << "\"" << endl;
            return -1;
        }

        fs["ObjectPointCount"] >> objectPointCount;
        for (int i = 0; i < objectPointCount; ++i) {
            Point3f objectPoint;
            fs["ObjectPoint" + to_string(i)] >> objectPoint;
            objectPoints.push_back(objectPoint);
        }

        transDeterm.checkReprojectionError(
            intrinsicParameters, referenceToCameraTransform, objectPoints, input);
    }

    // stop imaging
    input.stopImaging();
    return 0;
}
