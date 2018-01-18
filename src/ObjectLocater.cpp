#include "ObjectLocater.h"

#include "ThreadSafePrinter.h"

using namespace std;
using namespace cv;
using namespace igtl;

int
TrackingInformationUpdater::Process(TrackingDataMessage *message, void *data) {
    TrackingInformation *trackingInformation = static_cast<TrackingInformation *>(data);
    if (trackingInformation == nullptr) {
        return 0;
    }

    trackingInformation->updateFromMessage(message);
    return 1;
}

ObjectLocater::ObjectLocater() : m_trackingInformationUpdater{TrackingInformationUpdater::New()} {}

bool
ObjectLocater::readSettings(const FileNode &node) {
    node["HostName"] >> m_hostName;
    node["Port"] >> m_port;
    node["UpdateInterval"] >> m_updateInterval;
    node["PointerDeviceName"] >> m_pointerDeviceName;
    node["ReferenceElementDeviceName"] >> m_referenceElementDeviceName;

    validateSettings();

    if (m_settingsValid) {
        m_trackingInformation =
            make_shared<TrackingInformation>(m_pointerDeviceName, m_referenceElementDeviceName);
        m_trackingInformationUpdater->SetData(m_trackingInformation.get());
        m_trackingConnection = make_shared<OpenIGTLinkConnection>(m_hostName, m_port);
        m_trackingConnection->addMessageHandler(m_trackingInformationUpdater);
    }

    return m_settingsValid;
}

bool
ObjectLocater::settingsValid() const {
    return m_settingsValid;
}

shared_ptr<TrackingInformation>
ObjectLocater::trackingInformation() const {
    return m_trackingInformation;
}

bool
ObjectLocater::openConnection() {
    return m_trackingConnection->open();
}

bool
ObjectLocater::closeConnection() {
    if (m_trackingConnection == nullptr or not m_trackingConnection->isOpen()) {
        return false;
    }
    m_trackingConnection->close();
    return  true;
}

bool
ObjectLocater::startTracking() {
    if (m_trackingConnection == nullptr or not m_trackingConnection->isOpen()) {
        return false;
    }

    if (not m_trackingConnection->startRequestingTrackingData(m_updateInterval)) {
        return false;
    }

    return true;
}

void
ObjectLocater::stopTracking() {
    if (m_trackingConnection == nullptr) {
        return;
    }

    m_trackingConnection->stopRequestingTrackingData();
}

void
ObjectLocater::validateSettings() {
    m_settingsValid = true;

    if (m_hostName.empty()) {
        printErr << "Hostname can't be empty" << endl;
        m_settingsValid = false;
    }

    if (m_port < 0 or m_port > 65535) {
        printErr << "Invalid port number" << endl;
        m_settingsValid = false;
    }

    if (m_updateInterval < 0) {
        printErr << "Invalid update interval" << endl;
        m_settingsValid = false;
    }

    if (m_referenceElementDeviceName.empty()) {
        printErr << "Invalid device for the reference element" << endl;
        m_settingsValid = false;
    }

    if (m_pointerDeviceName.empty()) {
        printErr << "Invalid device name for the pointer" << endl;
        m_settingsValid = false;
    }
}

void
read(const FileNode &node, ObjectLocater &objectLocater, const ObjectLocater &defaultValue) {
    if (node.empty()) {
        objectLocater = defaultValue;
    } else {
        objectLocater.readSettings(node);
    }
}
