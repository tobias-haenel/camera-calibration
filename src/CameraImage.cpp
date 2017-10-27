#include "CameraImage.h"

#include "ThreadSafePrinter.h"

using namespace cv;
using namespace igtl;
using namespace std;
using namespace std::chrono;

void
CameraImage::updateFromMessage(MyImageMessage *message) {
    if (message == nullptr) {
        return;
    }

    duration<double> timeSinceEpoch = high_resolution_clock::now().time_since_epoch();
    double currentTime = timeSinceEpoch.count();

    int dim[3], off[3];
    message->GetSubVolume(dim, off);
    if (dim[2] != 1) {
        printErr << "Only 2D images supported" << endl;
        printErr << "Dimensions: " << dim[0] << "x" << dim[1] << "x" << dim[2] << endl;
        return;
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
        cvScalarType = CV_16U;
        break;
    case ImageMessage::TYPE_UINT16:
        cvScalarType = CV_16U;
        break;
    case ImageMessage::TYPE_INT32:
        cvScalarType = CV_16U;
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
        return;
    }

    int numberOfComponents = message->GetNumComponents();
    if (numberOfComponents <= 0) {
        printErr << "Invalid number of components" << endl;
        return;
    }

    void *imageData = message->GetScalarPointer();

    Mat messageImage{dim[0], dim[1], CV_MAKETYPE(cvScalarType, numberOfComponents), imageData};

    if (m_flipHorizontal) {
        flip(messageImage, messageImage, 0);
    }

    if (m_flipVertical) {
        flip(messageImage, messageImage, 1);
    }

    lock_guard<mutex> lock{m_mutex};
    m_image = messageImage.clone();
    m_timeStamp = currentTime;
    m_newPictureCondition.notify_all();
}

void
CameraImage::currentImage(Mat &image, double &timeStamp) {
    lock_guard<mutex> lock{m_mutex};
    image = m_image.clone();
    timeStamp = m_timeStamp;
}

bool
CameraImage::waitForNewImage() {
    unique_lock<mutex> lock{m_mutex};
    return m_newPictureCondition.wait_for(lock, 5s) != cv_status::timeout;
}

void
CameraImage::setFlipHorizontal(bool flipHorizontal) {
    m_flipHorizontal = flipHorizontal;
}

void
CameraImage::setFlipVertical(bool flipVertical) {
    m_flipVertical = flipVertical;
}
