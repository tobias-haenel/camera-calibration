#include "OpenIGTLinkConnection.h"

#include <igtlBindMessage.h>
#include <igtlCapabilityMessage.h>
#include <igtlColorTableMessage.h>
#include <igtlImageMessage.h>
#include <igtlImageMetaMessage.h>
#include <igtlLabelMetaMessage.h>
#include <igtlNDArrayMessage.h>
#include <igtlPointMessage.h>
#include <igtlPolyDataMessage.h>
#include <igtlPositionMessage.h>
#include <igtlQuaternionTrackingDataMessage.h>
#include <igtlSensorMessage.h>
#include <igtlStatusMessage.h>
#include <igtlStringMessage.h>
#include <igtlTrackingDataMessage.h>
#include <igtlTrajectoryMessage.h>
#include <igtlTransformMessage.h>

using namespace igtl;
using namespace std;

OpenIGTLinkConnection::OpenIGTLinkConnection(const string &host, int port)
    : m_sessionManager{SessionManager::New()} {
    m_sessionManager->SetHostname(host.c_str());
    m_sessionManager->SetPort(port);
    m_sessionManager->SetMode(SessionManager::MODE_CLIENT);
}

OpenIGTLinkConnection::~OpenIGTLinkConnection() {
    close();
}

bool
OpenIGTLinkConnection::addMessageHandler(MessageHandler *messageHandler) {
    if (m_receivingMessages) {
        return false;
    }

    return m_sessionManager->AddMessageHandler(messageHandler);
}

bool
OpenIGTLinkConnection::open() {
    if (not connect()) {
        return false;
    }

    startReceiveMessageThread();

    return true;
}

void
OpenIGTLinkConnection::getBind(set<pair<string, string>> messageRequests, const string &device) {
    MessageBase::Pointer message;
    auto getNindMessage = GetBindMessage::New();
    for (auto const &messageRequest : messageRequests) {
        std::string const &device = messageRequest.first;
        std::string const &messageType = messageRequest.second;
        getNindMessage->AppendChildMessage(messageType.c_str(), device.c_str());
    }
    message = getNindMessage;
    sendMessage(message, device);
}

void
OpenIGTLinkConnection::getCapability(const string &device) {
    MessageBase::Pointer message;
    message = GetCapabilityMessage::New();
    sendMessage(message, device);
}

void
OpenIGTLinkConnection::getColorTable(const string &device) {
    MessageBase::Pointer message;
    message = GetColorTableMessage::New();
    sendMessage(message, device);
}

void
OpenIGTLinkConnection::getImage(const string &device) {
    MessageBase::Pointer message;
    message = GetImageMessage::New();
    sendMessage(message, device);
}

void
OpenIGTLinkConnection::getImageMeta(const string &device) {
    MessageBase::Pointer message;
    message = GetImageMetaMessage::New();
    sendMessage(message, device);
}

void
OpenIGTLinkConnection::getLabelMeta(const string &device) {
    MessageBase::Pointer message;
    message = GetLabelMetaMessage::New();
    sendMessage(message, device);
}

void
OpenIGTLinkConnection::getPoint(const string &device) {
    MessageBase::Pointer message;
    message = GetPointMessage::New();
    sendMessage(message, device);
}

void
OpenIGTLinkConnection::getPolyData(const string &device) {
    MessageBase::Pointer message;
    message = GetPolyDataMessage::New();
    sendMessage(message, device);
}

void
OpenIGTLinkConnection::getStatus(const string &device) {
    MessageBase::Pointer message;
    message = GetStatusMessage::New();
    sendMessage(message, device);
}

void
OpenIGTLinkConnection::getTrajectory(const string &device) {
    MessageBase::Pointer message;
    message = GetTrajectoryMessage::New();
    sendMessage(message, device);
}

void
OpenIGTLinkConnection::getTransform(const string &device) {
    MessageBase::Pointer message;
    message = GetTransformMessage::New();
    sendMessage(message, device);
}

void
OpenIGTLinkConnection::startRequestingBind(std::set<std::pair<string, string>> messageRequests,
                                           double resolution,
                                           const string &device) {
    MessageBase::Pointer message;
    auto startBindMessage = StartBindMessage::New();
    for (auto const &messageRequest : messageRequests) {
        std::string const &device = messageRequest.first;
        std::string const &messageType = messageRequest.second;
        startBindMessage->AppendChildMessage(messageType.c_str(), device.c_str());
    }
    auto resolutionTimeStamp = TimeStamp::New();
    resolutionTimeStamp->SetTime(resolution);
    startBindMessage->SetResolution(resolutionTimeStamp->GetTimeStampUint64());
    message = startBindMessage;
    sendMessage(message, device);
}

void
OpenIGTLinkConnection::stopRequestingBind(const string &device) {
    MessageBase::Pointer message;
    message = StopBindMessage::New();
    sendMessage(message, device);
}

void
OpenIGTLinkConnection::startRequestingQuaternionTrackingData(int resolution,
                                                             const string &coordinateSystem,
                                                             const string &device) {

    MessageBase::Pointer message;
    auto quaternionTrackingDataMessage = StartQuaternionTrackingDataMessage::New();
    quaternionTrackingDataMessage->SetResolution(resolution);
    quaternionTrackingDataMessage->SetCoordinateName(coordinateSystem.c_str());
    message = quaternionTrackingDataMessage;
    sendMessage(message, device);
}

void
OpenIGTLinkConnection::stopRequestingQuaternionTrackingData(const string &device) {
    MessageBase::Pointer message;
    message = StopQuaternionTrackingDataMessage::New();
    sendMessage(message, device);
}

void
OpenIGTLinkConnection::startRequestingTrackingData(int resolution,
                                                   const string &coordinateSystem,
                                                   const string &device) {
    MessageBase::Pointer message;
    auto trackingDataMessage = StartTrackingDataMessage::New();
    trackingDataMessage->SetResolution(resolution);
    trackingDataMessage->SetCoordinateName(coordinateSystem.c_str());
    message = trackingDataMessage;
    sendMessage(message, device);
}

void
OpenIGTLinkConnection::stopRequestingTrackingData(const string &device) {
    MessageBase::Pointer message;
    message = StopTrackingDataMessage::New();
    sendMessage(message, device);
}

void
OpenIGTLinkConnection::close() {
    stopReceiveMessageThread(); // triggers connection close at end
}

void
OpenIGTLinkConnection::sendMessage(MessageBase::Pointer message, const string &device) {
    std::string task = std::string("Sending message '") + message->GetDeviceType() + "' ... ";
    if (not device.empty()) {
        message->SetDeviceName(device.c_str());
    }
    message->Pack();
    if (m_sessionManager->PushMessage(message) == 1) {
        printOut << task << "success" << std::endl;
    } else {
        printOut << task << "error" << std::endl;
    }
}

void
OpenIGTLinkConnection::receiveMessagesLoop() {
    while (m_receivingMessages) {
        if (m_sessionManager->ProcessMessage() == 0) {
            break;
        }
    }
    disconnect();
}

void
OpenIGTLinkConnection::startReceiveMessageThread() {
    stopReceiveMessageThread();

    m_receivingMessages = true;
    m_receiveMessageThread = thread(&OpenIGTLinkConnection::receiveMessagesLoop, ref(*this));
}

void
OpenIGTLinkConnection::stopReceiveMessageThread() {
    m_receivingMessages = false;
    if (m_receiveMessageThread.joinable()) {
        m_receiveMessageThread.join();
    }
}

bool
OpenIGTLinkConnection::connect() {
    string task = "Opening connection to " + string(m_sessionManager->GetHostname()) + ':' +
        to_string(m_sessionManager->GetPort()) + " ... ";
    if (m_sessionManager->Connect() == 1) {
        printOut << task << "success" << endl;
        return true;
    } else {
        printOut << task << "failed" << endl;
        return false;
    }
}

void
OpenIGTLinkConnection::disconnect() {
    m_sessionManager->Disconnect();
    printOut << "Closing connection to " << m_sessionManager->GetHostname() << ':'
             << m_sessionManager->GetPort() << " ... success" << endl;
}
