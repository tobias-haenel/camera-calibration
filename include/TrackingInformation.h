#pragma once

#include <mutex>
#include <opencv2/core.hpp>

#include <igtlTrackingDataMessage.h>

class TrackingInformation {
public:
    TrackingInformation(std::string const &pointerDeviceName,
                        std::string const &referenceElementDeviceName);

    void
    updateFromMessage(igtl::TrackingDataMessage *trackingDataMessage);

    void
    pointer(cv::Vec3f &position, double &timeStamp);

    void
    referenceElement(cv::Mat &transformation, double &timeStamp);

private:
    std::string m_pointerDeviceName;
    std::string m_referenceElementDeviceName;

    cv::Vec3f m_pointerPosition;
    double m_pointerTimeStamp = 0.0;
    std::mutex m_pointerMutex;

    cv::Mat m_referenceTransformation;
    double m_referenceTimeStamp = 0.0;
    std::mutex m_referenceMutex;
};
