#pragma once

#include <opencv2/core.hpp>

#include <igtlMessageHandler.h>
#include <igtlMessageHandlerMacro.h>

#include <memory>

#include "CameraImage.h"
#include "ImageWithFocusMessage.h"
#include "OpenIGTLinkConnection.h"

igtlMessageHandlerClassMacro(igtl::ImageWithFocusMessage, ImageWithFocusUpdater, CameraImage);
igtlMessageHandlerClassMacro(igtl::ImageMessage, ImageUpdater, CameraImage);

/**
 * @brief The CameraInput class stream video images based on a OpenIGTLinkConnection to an imaging
 * device.
 */
class CameraInput {
public:
    /**
     * @brief Creates a new CameraInput.
     */
    CameraInput();

    /**
     * @brief Destroys the CameraInput.
     */
    ~CameraInput();

    CameraInput(const CameraInput &) = default;
    CameraInput &operator=(const CameraInput &) = default;

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
     * @brief Gets the camera image
     * @return Pointer to the camera image
     */
    std::shared_ptr<CameraImage>
    cameraImage() const;

    /**
     * @brief Starts the imaging process.
     * @return boolean indicating success
     */
    bool
    startImaging();

    /**
     * @brief Checks if the imaging process is still working.
     * @return boolean indicating status
     */
    bool
    isImaging();

    /**
     * @brief Stops the imaging process.
     */
    void
    stopImaging();

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
     * @brief Connection to a imaging device
     */
    std::shared_ptr<OpenIGTLinkConnection> m_imagingConnection;

    /**
     * @brief Processes ImageWithFocusMessages
     */
    ImageWithFocusUpdater::Pointer m_imageWithFocusUpdater;

    /**
     * @brief Processes ImageMessages
     */
    ImageUpdater::Pointer m_imageUpdater;

    /**
     * @brief Camera image that is contiously updated
     */
    std::shared_ptr<CameraImage> m_cameraImage;

    /**
     * @brief Host name of the connection to the imaging device
     */
    std::string m_hostName;

    /**
     * @brief Port of the connection to the imaging device
     */
    int m_port = -1;
};

/**
 * @brief OpenCV function for reading a CameraInput from a FileNode.
 * @param node FileNode where the settings of the CameraInput are read from
 * @param cameraInput CameraInput that is changed acording to the file contents
 * @param defaultValue Default value for the CameraInput if the FileNode isn't valid
 */
void
read(const cv::FileNode &node,
     CameraInput &cameraInput,
     const CameraInput &defaultValue = CameraInput());
