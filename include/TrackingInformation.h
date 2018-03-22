#pragma once

#include <condition_variable>
#include <mutex>
#include <opencv2/core.hpp>

#include <igtlTrackingDataMessage.h>

/**
 * @brief The TrackingInformation class is a thread safe container for tracking information.
 */
class TrackingInformation {
public:
    friend class StateUpdater;
    friend class ObjectLocater;

    /**
     * @brief The ConnectionState enum describes possible connection states of the OpenIGTLink
     * 'session'
     */
    enum class ConnectionState {
        Init,
        Active,
        Inactive
    };

    /**
     * @brief Creates a new TrackingInformation.
     * @param pointerDeviceName Device name of the pointer instrument
     * @param referenceElementDeviceName Device name of the reference element
     */
    TrackingInformation(std::string const &pointerDeviceName,
                        std::string const &referenceElementDeviceName);

    /**
     * @brief Waits until the 'session' is active or inactive (waits a maximum of 120s)
     * @note This method is thread safe
     * @return boolean, indicating if the session is active
     */
    bool
    waitForInitialisiation();

    /**
     * @brief Gets the current connection state.
     * @note This method is thread safe
     * @return ConnectionState
     */
    ConnectionState
    connectionState();

    /**
     * @brief Updates the stored information from a tracking data message.
     * @note This method is thread safe
     * @param trackingDataMessage TDATA message
     */
    void
    updateFromMessage(igtl::TrackingDataMessage *trackingDataMessage);

    /**
     * @brief Gets the currently stored position of the pointer tip.
     * @note This method is thread safe
     * @param position Position in the antomy coordinate system
     * @param timeStamp Time stamp that describes when the position was stored
     */
    void
    pointer(cv::Vec3f &position, double &timeStamp);

    /**
     * @brief Gets the currently stored transform of the reference element.
     * @note This method is thread safe
     * @param transformation Transformation from the reference element to the anatomy.
     * @param timeStamp Time stamp that describes when the transformation was stored
     */
    void
    referenceElement(cv::Mat &transformation, double &timeStamp);

protected:
    /**
     * @brief Sets the current connection state.
     * @note This method is thread safe
     * @param newState New ConnectionState
     */
    void
    setConnectionState(ConnectionState newState);

private:
    /**
     * @brief Name of the pointer device
     */
    std::string m_pointerDeviceName;

    /**
     * @brief Name of the reference device
     */
    std::string m_referenceElementDeviceName;

    /**
     * @brief Currently stored pointer position
     */
    cv::Vec3f m_pointerPosition;

    /**
     * @brief Timestap that indicates when the last pointer postion was stored
     */
    double m_pointerTimeStamp = 0.0;

    /**
     * @brief Mutex that guards the pointer position
     */
    std::mutex m_pointerMutex;

    /**
     * @brief Currently stored transformation from the reference element to the anatomy
     */
    cv::Mat m_referenceTransformation;

    /**
     * @brief Timestap that indicates when the last transformation was stored
     */
    double m_referenceTimeStamp = 0.0;

    /**
     * @brief Mutex that guards the reference element
     */
    std::mutex m_referenceMutex;

    /**
     * @brief Current connection state of the OpenIGTLink 'session'
     */
    ConnectionState m_connectionState = ConnectionState::Inactive;

    /**
     * @brief Mutex that guards the connection state
     */
    std::mutex m_connectionStateMutex;

    /**
     * @brief Condition variable for the connection state
     */
    std::condition_variable m_newConnectionStateCondition;
};
