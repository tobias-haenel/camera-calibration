#pragma once

#include <opencv2/core.hpp>

#include <igtlImageMessage.h>

#include <condition_variable>
#include <mutex>

#include "ImageWithFocusMessage.h"

/**
 * @brief The CameraImage class is a thread safe container for images. It is also responsible to
 * create an 8-bit RGB image out of the 32-bit float temperature data.
 */
class CameraImage {
public:
    /**
     * @brief Update the stored image from a custom image message with a focus value.
     * @note This function is thread safe
     * @param message Image message with focus value
     */
    void
    updateFromMessage(igtl::ImageWithFocusMessage* message);

    /**
     * @brief Update the stored image from an image message.
     * @note This function is thread safe
     * @param message Image message
     */
    void
    updateFromMessage(igtl::ImageMessage* message);

    /**
     * @brief Gets the currently stored image.
     * @note This function is thread safe
     * @param image Copy of the stored image
     * @param timeStamp Time stamp when the image was obtained
     * @param focusValue Focus value of the image, -1 if none was obtained
     */
    void
    currentImage(cv::Mat &image, double &timeStamp, int& focusValue);

    /**
     * @brief Waits at most 5 seconds until a new image was stored.
     * @note This function is thread safe
     * @return boolean, that indicates if a new image was stored
     */
    bool
    waitForNewImage();

private:
    cv::Mat m_image;
    double m_timeStamp = 0.0;
    int m_focusValue = -1;
    std::mutex m_mutex;
    std::condition_variable m_newPictureCondition;
};
