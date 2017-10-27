#pragma once

#include <igtlSessionManager.h>

#include <set>
#include <thread>

#include "ThreadSafePrinter.h"

class OpenIGTLinkConnection {
public:
    OpenIGTLinkConnection(std::string const &host, int port);

    ~OpenIGTLinkConnection();

    bool
    addMessageHandler(igtl::MessageHandler *messageHandler);

    bool
    open();

    bool
    getBind(std::set<std::pair<std::string, std::string>> messageRequests,
            std::string const &device = std::string());

    bool
    getCapability(std::string const &device = std::string());

    bool
    getColorTable(std::string const &device = std::string());

    bool
    getImage(std::string const &device = std::string());

    bool
    getImageMeta(std::string const &device = std::string());

    bool
    getLabelMeta(std::string const &device = std::string());

    bool
    getPoint(std::string const &device = std::string());

    bool
    getPolyData(std::string const &device = std::string());

    bool
    getStatus(std::string const &device = std::string());

    bool
    getTrajectory(std::string const &device = std::string());

    bool
    getTransform(std::string const &device = std::string());

    bool
    startRequestingBind(std::set<std::pair<std::string, std::string>> messageRequests,
                        double resolution,
                        std::string const &device = std::string());

    bool
    stopRequestingBind(std::string const &device = std::string());

    bool
    startRequestingQuaternionTrackingData(int resolution,
                                          std::string const &coordinateSystem = std::string(),
                                          std::string const &device = std::string());

    bool
    stopRequestingQuaternionTrackingData(std::string const &device = std::string());

    bool
    startRequestingTrackingData(int resolution,
                                std::string const &coordinateSystem = std::string(),
                                std::string const &device = std::string());

    bool
    stopRequestingTrackingData(std::string const &device = std::string());

    void
    close();

private:
    bool
    sendMessage(igtl::MessageBase::Pointer message, std::string const &device = std::string());

    void
    receiveMessagesLoop();

    void
    startReceiveMessageThread();

    void
    stopReceiveMessageThread();

    bool
    connect();

    void
    disconnect();

private:
    igtl::SessionManager::Pointer m_sessionManager;

    std::thread m_receiveMessageThread;

    bool m_receivingMessages = false;

    bool m_connected = false;
};
