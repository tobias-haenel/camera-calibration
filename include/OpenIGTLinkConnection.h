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

    void
    getBind(std::set<std::pair<std::string, std::string>> messageRequests,
            std::string const &device = std::string());

    void
    getCapability(std::string const &device = std::string());

    void
    getColorTable(std::string const &device = std::string());

    void
    getImage(std::string const &device = std::string());

    void
    getImageMeta(std::string const &device = std::string());

    void
    getLabelMeta(std::string const &device = std::string());

    void
    getPoint(std::string const &device = std::string());

    void
    getPolyData(std::string const &device = std::string());

    void
    getStatus(std::string const &device = std::string());

    void
    getTrajectory(std::string const &device = std::string());

    void
    getTransform(std::string const &device = std::string());

    void
    startRequestingBind(std::set<std::pair<std::string, std::string>> messageRequests,
                        double resolution,
                        std::string const &device = std::string());

    void
    stopRequestingBind(std::string const &device = std::string());

    void
    startRequestingQuaternionTrackingData(int resolution,
                                          std::string const &coordinateSystem = std::string(),
                                          std::string const &device = std::string());

    void
    stopRequestingQuaternionTrackingData(std::string const &device = std::string());

    void
    startRequestingTrackingData(int resolution,
                                std::string const &coordinateSystem = std::string(),
                                std::string const &device = std::string());

    void
    stopRequestingTrackingData(std::string const &device = std::string());

    void
    close();

private:
    void
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
