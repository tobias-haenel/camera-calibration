#pragma once

#include <opencv2/core.hpp>

#include <igtlImageMessage.h>

#include <condition_variable>
#include <mutex>

#include "ImageWithFocusMessage.h"

class CameraImage {
public:
    void
    updateFromMessage(igtl::ImageWithFocusMessage* message);

    void
    updateFromMessage(igtl::ImageMessage* message);

    void
    currentImage(cv::Mat &image, double &timeStamp, int& focusValue);

    bool
    waitForNewImage();

    void
    setFlipHorizontal(bool flipHorizontal);

    void
    setFlipVertical(bool flipVertical);

    void
    transformImage(cv::Mat &image);

private:
    cv::Mat m_image;
    double m_timeStamp = 0.0;
    int m_focusValue = -1;
    std::mutex m_mutex;
    std::condition_variable m_newPictureCondition;
    bool m_flipHorizontal = false;
    bool m_flipVertical = false;
};
