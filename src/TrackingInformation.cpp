#include "TrackingInformation.h"

using namespace cv;
using namespace igtl;
using namespace std;
using namespace std::chrono;

TrackingInformation::TrackingInformation(const string &pointerDeviceName,
                                         const string &referenceElementDeviceName)
    : m_pointerDeviceName{pointerDeviceName},
      m_referenceElementDeviceName{referenceElementDeviceName} {}

void
TrackingInformation::updateFromMessage(TrackingDataMessage *trackingDataMessage) {
    if (trackingDataMessage == nullptr) {
        return;
    }

    duration<double> timeSinceEpoch = high_resolution_clock::now().time_since_epoch();
    double currentTime = timeSinceEpoch.count();

    for (int i = 0; i < trackingDataMessage->GetNumberOfTrackingDataElements(); ++i) {
        TrackingDataElement::Pointer trackingDataElement;
        trackingDataMessage->GetTrackingDataElement(i, trackingDataElement);

        std::string deviceName = trackingDataElement->GetName();
        if (deviceName == m_pointerDeviceName) {
            float position[3];
            trackingDataElement->GetPosition(position);
            {
                lock_guard<mutex> lock{m_pointerMutex};
                m_pointerPosition[0] = position[0];
                m_pointerPosition[1] = position[1];
                m_pointerPosition[2] = position[2];
                m_pointerTimeStamp = currentTime;
            }
        } else if (deviceName == m_referenceElementDeviceName) {
            Matrix4x4 transformation;
            trackingDataElement->GetMatrix(transformation);
            {
                lock_guard<mutex> lock{m_referenceMutex};
                m_referenceTransformation = Mat(4, 4, CV_64F);
                for (int row = 0; row < 4; ++row) {
                    for (int col = 0; col < 4; ++col) {
                        m_referenceTransformation.at<double>(row, col) = transformation[row][col];
                    }
                }
                m_referenceTimeStamp = currentTime;
            }
        }
    }
}

void
TrackingInformation::pointer(Vec3f& position, double &timeStamp) {
    lock_guard<mutex> lock{m_pointerMutex};
    position = m_pointerPosition;
    timeStamp = m_pointerTimeStamp;
}

void
TrackingInformation::referenceElement(Mat &transformation, double &timeStamp) {
    lock_guard<mutex> lock{m_referenceMutex};
    transformation = m_referenceTransformation;
    timeStamp = m_referenceTimeStamp;
}
