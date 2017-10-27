#pragma once

#include <opencv2/core.hpp>

#include <igtlImageMessage.h>

#include <condition_variable>
#include <mutex>

#include "MyImageMessage.h"

class CameraImage {
public:
    void
    updateFromMessage(igtl::MyImageMessage* message);

    void
    currentImage(cv::Mat &image, double &timeStamp);

    bool
    waitForNewImage();

    void
    setFlipHorizontal(bool flipHorizontal);

    void
    setFlipVertical(bool flipVertical);

private:
    cv::Mat m_image;
    double m_timeStamp = 0.0;
    std::mutex m_mutex;
    std::condition_variable m_newPictureCondition;
    bool m_flipHorizontal = false;
    bool m_flipVertical = false;
};
