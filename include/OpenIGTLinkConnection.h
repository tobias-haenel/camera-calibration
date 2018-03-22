#pragma once

#include <igtlMessageHandler.h>
#include <igtlClientSocket.h>

#include <set>
#include <thread>
#include <map>

/**
 * @brief The OpenIGTLinkConnection class manages a connection to an OpenIGTLink server.
 */
class OpenIGTLinkConnection {
public:
    /**
     * @brief Creates a OpenIGTLinkConnection to a given host on a given port.
     * @param host Host name
     * @param port Port
     */
    OpenIGTLinkConnection(std::string const &host, int port);

    /**
     * @brief Destroys the OpenIGTLinkConnection.
     */
    virtual ~OpenIGTLinkConnection();

    /**
     * @brief Adds a message handler that deals with messages of a certain device type.
     * @param messageHandler New message handler
     * @return
     */
    bool
    addMessageHandler(igtl::MessageHandler *messageHandler);

    /**
     * @brief Opens the connection and starts receiving and parsing of messages.
     * @return boolean, indicating success
     */
    bool
    open();

    /**
     * @brief Checks if the connection is currently open.
     * @note The connection may close itself asynchronically
     * @return boolean, indicating success
     */
    bool
    isOpen();

    /**
     * @brief Sends a GET_CAPABIL message.
     * @param device Device name that is specified in the message
     * @return boolean, indicating success
     */
    bool
    getCapability(std::string const &device = std::string());

    /**
     * @brief Sends a GET_COLORT message.
     * @param device Device name that is specified in the message
     * @return boolean, indicating success
     */
    bool
    getColorTable(std::string const &device = std::string());

    /**
     * @brief Sends a GET_IMAGE message.
     * @param device Device name that is specified in the message
     * @return boolean, indicating success
     */
    bool
    getImage(std::string const &device = std::string());

    /**
     * @brief Sends a GET_IMGMETA message.
     * @param device Device name that is specified in the message
     * @return boolean, indicating success
     */
    bool
    getImageMeta(std::string const &device = std::string());

    /**
     * @brief Sends a GET_LBMETA message.
     * @param device Device name that is specified in the message
     * @return boolean, indicating success
     */
    bool
    getLabelMeta(std::string const &device = std::string());

    /**
     * @brief Sends a GET_POINT message.
     * @param device Device name that is specified in the message
     * @return boolean, indicating success
     */
    bool
    getPoint(std::string const &device = std::string());

    /**
     * @brief Sends a GET_POLYDATA message.
     * @param device Device name that is specified in the message
     * @return boolean, indicating success
     */
    bool
    getPolyData(std::string const &device = std::string());

    /**
     * @brief Sends a GET_STATUS message.
     * @param device Device name that is specified in the message
     * @return boolean, indicating success
     */
    bool
    getStatus(std::string const &device = std::string());

    /**
     * @brief Sends a GET_TRAJ message.
     * @param device Device name that is specified in the message
     * @return boolean, indicating success
     */
    bool
    getTrajectory(std::string const &device = std::string());

    /**
     * @brief Sends a GET_TRANSFORM message.
     * @param device Device name that is specified in the message
     * @return boolean, indicating success
     */
    bool
    getTransform(std::string const &device = std::string());

    /**
     * @brief Sends a STT_QTDATA message
     * @param resolution Minimum time between to frames (in ms, 0 = as fast as possible)
     * @param coordinateSystem Name of the coordinate system to use (can be emtpy)
     * @param device Device name that is specified in the message
     * @return boolean, indicating success
     */
    bool
    startRequestingQuaternionTrackingData(int resolution,
                                          std::string const &coordinateSystem = std::string(),
                                          std::string const &device = std::string());

    /**
     * @brief Sends a STP_QTDATA message
     * @param device Device name that is specified in the message
     * @return boolean, indicating success
     */
    bool
    stopRequestingQuaternionTrackingData(std::string const &device = std::string());

    /**
     * @brief Sends a STT_TDATA message
     * @param resolution Minimum time between to frames (in ms, 0 = as fast as possible)
     * @param coordinateSystem Name of the coordinate system to use (can be emtpy)
     * @param device Device name that is specified in the message
     * @return boolean, indicating success
     */
    bool
    startRequestingTrackingData(int resolution,
                                std::string const &coordinateSystem = std::string(),
                                std::string const &device = std::string());

    /**
     * @brief Sends a STP_TDATA message
     * @param device Device name that is specified in the message
     * @return boolean, indicating success
     */
    bool
    stopRequestingTrackingData(std::string const &device = std::string());

    /**
     * @brief Closes the connection and stops receiving and parsing of messages.
     */
    void
    close();

private:
    /**
     * @brief Sends a message object
     * @param message Pointer to any message object
     * @param device Device name that is specified in the message
     * @return boolean, indicating success
     */
    bool
    sendMessage(igtl::MessageBase::Pointer message, std::string const &device = std::string());

    /**
     * @brief Message loop that receives messages and dispatches them to message handlers.
     * @note This runs in another thread.
     */
    void
    receiveMessagesLoop();

    /**
     * @brief Start the thread that runs receiveMessagesLoop()
     */
    void
    startReceiveMessageThread();

    /**
     * @brief Closes the thread that runs receiveMessageLoop()
     */
    void
    stopReceiveMessageThread();

    /**
     * @brief Internal method to connect to the OpenIGTLink server.
     * @return boolean, indicating success
     */
    bool
    connect();

    /**
     * @brief Internal method to disconnect from the OpenIGTLink server.
     * @return boolean, indicating success
     */
    void
    disconnect();

private:
    /**
     * @brief Client socket that is used to send and receive messages
     */
    igtl::ClientSocket::Pointer m_clientSocket;

    /**
     * @brief Message header that is used for receiving messages
     */
    igtl::MessageHeader::Pointer m_messageHeader;

    /**
     * @brief Host name of the OpenIGTLink server
     */
    std::string m_hostName;

    /**
     * @brief Port of the OpenIGTLink server
     */
    int m_port;

    /**
     * @brief Map from device types / message names to matching message handlers
     */
    std::map<std::string, igtl::MessageHandler *> m_messageHandlers;

    /**
     * @brief Thread that receives messages
     */
    std::thread m_receiveMessageThread;

    /**
     * @brief Flag that is used to stop the loop in the thread that receives the messages
     */
    bool m_receivingMessages = false;
};
