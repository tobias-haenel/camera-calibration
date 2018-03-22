#pragma once

#include <igtlMessageHandler.h>
#include <igtlMessageHandlerMacro.h>
#include <igtlTrackingDataMessage.h>
#include <igtlStatusMessage.h>

#include <condition_variable>
#include <memory>

#include <opencv2/core.hpp>

#include "OpenIGTLinkConnection.h"
#include "TrackingInformation.h"

igtlMessageHandlerClassMacro(igtl::TrackingDataMessage, TrackingInformationUpdater, void);
igtlMessageHandlerClassMacro(igtl::StatusMessage, StateUpdater, void);

/**
 * @brief The ObjectLocater class can track various objects based on an OpenIGTLinkConnection to a
 * tracking device.
 */
class ObjectLocater {
public:

    /**
     * @brief Creates a new ObjectLocater.
     */
    ObjectLocater();

    /**
     * @brief Reads settings from an OpenCV FileNode.
     * @param node OpenCV FileNode
     * @return boolean indicating validity of the settings
     */
    bool
    readSettings(const cv::FileNode &node);

    /**
     * @brief Checks if read settings where valid.
     * @return boolean indicating validity
     */
    bool
    settingsValid() const;

    /**
     * @brief Gets the tracking information
     * @return Pointer to the tracking information
     */
    std::shared_ptr<TrackingInformation>
    trackingInformation() const;

    /**
     * @brief Opens the underlying OpenIGTLink connection.
     * @return boolean, indicating success
     */
    bool
    openConnection();

    /**
     * @brief Closes the underlying OpenIGTLink connection.
     * @return boolean, indicating success
     */
    bool
    closeConnection();

    /**
     * @brief Starts the tracking process.
     * @return boolean indicating success
     */
    bool
    startTracking();

    /**
     * @brief Stops the tracking process.
     */
    void
    stopTracking();

private: // methods
    /**
     * @brief Checks if the read settings are valid and updates settingsValid() accordingly.
     */
    void
    validateSettings();

private: // members
    /**
     * @brief Indicates if the settings are valid
     */
    bool m_settingsValid = false;

    /**
     * @brief Connection to a tracking device
     */
    std::shared_ptr<OpenIGTLinkConnection> m_trackingConnection;

    /**
     * @brief Processes TrackingDataMessages
     */
    TrackingInformationUpdater::Pointer m_trackingInformationUpdater;

    /**
     * @brief Processes StatusMessages
     */
    StateUpdater::Pointer m_stateUpdater;

    /**
     * @brief Tracking information that is contiously updated
     */
    std::shared_ptr<TrackingInformation> m_trackingInformation;

    /**
     * @brief Host name of the connection to the tracking device
     */
    std::string m_hostName;

    /**
     * @brief Port of the connection to the tracking device
     */
    int m_port = -1;

    /**
     * @brief Device name of the pointer that gives position information
     */
    std::string m_pointerDeviceName;

    /**
     * @brief Interval of updates in ms (0 indicates as low as possible)
     */
    int m_updateInterval = -1;

    /**
     * @brief Device name of the reference element that gives position and orientation information
     */
    std::string m_referenceElementDeviceName;
};

/**
 * @brief OpenCV function for reading a ObjectLocater from a FileNode.
 * @param node FileNode where the settings of the ObjectLocater are read from
 * @param objectLocater ObjectLocater that is changed acording to the file contents
 * @param defaultValue Default value for the ObjectLocater if the FileNode isn't valid
 */
void
read(const cv::FileNode &node,
     ObjectLocater &objectLocater,
     const ObjectLocater &defaultValue = ObjectLocater());
