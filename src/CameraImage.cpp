#include "CameraImage.h"

#include "ThreadSafePrinter.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

using namespace cv;
using namespace igtl;
using namespace std;
using namespace std::chrono;

static Mat
getImage(ImageMessage *message) {
    int dim[3], off[3];
    message->GetSubVolume(dim, off);
    if (dim[2] != 1) {
        printErr << "Only 2D images supported" << endl;
        printErr << "Dimensions: " << dim[0] << "x" << dim[1] << "x" << dim[2] << endl;
        return Mat();
    }

    int igtlScalarType = message->GetScalarType();
    int cvScalarType = -1;
    switch (igtlScalarType) {
    case ImageMessage::TYPE_INT8:
        cvScalarType = CV_8S;
        break;
    case ImageMessage::TYPE_UINT8:
        cvScalarType = CV_8U;
        break;
    case ImageMessage::TYPE_INT16:
        cvScalarType = CV_16S;
        break;
    case ImageMessage::TYPE_UINT16:
        cvScalarType = CV_16U;
        break;
    case ImageMessage::TYPE_INT32:
        cvScalarType = CV_32S;
        break;
    case ImageMessage::TYPE_FLOAT32:
        cvScalarType = CV_32F;
        break;
    case ImageMessage::TYPE_FLOAT64:
        cvScalarType = CV_64F;
        break;
    }

    if (cvScalarType == -1) {
        printErr << "Unsupported Scalar type" << endl;
        return Mat();
    }

    int numberOfComponents = message->GetNumComponents();
    if (numberOfComponents <= 0) {
        printErr << "Invalid number of components" << endl;
        return Mat();
    }

    void *imageData = message->GetScalarPointer();

    Mat messageImage{dim[1], dim[0], CV_MAKETYPE(cvScalarType, numberOfComponents), imageData};

    double minVal, maxVal;
    minMaxLoc(messageImage, &minVal, &maxVal);
    double newScale = 255.0 / (maxVal - minVal);
    messageImage.convertTo(messageImage, CV_8U, newScale, -newScale * minVal);

    if (numberOfComponents == 1) {
        Mat grayImage;
        cvtColor(messageImage, grayImage, COLOR_GRAY2BGR);
        messageImage = grayImage;
    }

    return messageImage;
}

void
CameraImage::updateFromMessage(ImageWithFocusMessage *message) {
    if (message == nullptr) {
        return;
    }

    duration<double> timeSinceEpoch = high_resolution_clock::now().time_since_epoch();
    double currentTime = timeSinceEpoch.count();
    Mat messageImage = getImage(message);
    int focusValue = message->GetFocusValue();

    lock_guard<mutex> lock{m_mutex};
    m_image = messageImage.clone();
    m_timeStamp = currentTime;
    m_focusValue = focusValue;
    m_newPictureCondition.notify_all();
}

void
CameraImage::updateFromMessage(ImageMessage *message) {
    if (message == nullptr) {
        return;
    }

    duration<double> timeSinceEpoch = high_resolution_clock::now().time_since_epoch();
    double currentTime = timeSinceEpoch.count();
    Mat messageImage = getImage(message);

    lock_guard<mutex> lock{m_mutex};
    m_image = messageImage.clone();
    m_timeStamp = currentTime;
    m_newPictureCondition.notify_all();
}

void
CameraImage::currentImage(Mat &image, double &timeStamp, int &focusValue) {
    lock_guard<mutex> lock{m_mutex};
    image = m_image.clone();
    timeStamp = m_timeStamp;
    focusValue = m_focusValue;
}

bool
CameraImage::waitForNewImage() {
    unique_lock<mutex> lock{m_mutex};
    return m_newPictureCondition.wait_for(lock, 5s) != cv_status::timeout;
}
