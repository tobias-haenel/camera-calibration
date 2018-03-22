#include "CameraInput.h"

#include "ThreadSafePrinter.h"

using namespace std;
using namespace cv;
using namespace igtl;

int
ImageWithFocusUpdater::Process(ImageWithFocusMessage *message, CameraImage *cameraImage) {
    if (cameraImage == nullptr) {
        return 0;
    }

    cameraImage->updateFromMessage(message);
    return 1;
}

int
ImageUpdater::Process(ImageMessage *message, CameraImage *cameraImage) {
    if (cameraImage == nullptr) {
        return 0;
    }

    cameraImage->updateFromMessage(message);
    return 1;
}

CameraInput::CameraInput()
    : m_imageWithFocusUpdater{ImageWithFocusUpdater::New()},
      m_imageUpdater{ImageUpdater::New()},
      m_cameraImage{make_shared<CameraImage>()} {
    m_imageWithFocusUpdater->SetData(m_cameraImage.get());
    m_imageUpdater->SetData(m_cameraImage.get());
    m_imageWithFocusUpdater->CheckCRC(1);
    m_imageUpdater->CheckCRC(1);
}

CameraInput::~CameraInput() {
    stopImaging();
}

bool
CameraInput::readSettings(const FileNode &node) {
    node["HostName"] >> m_hostName;
    node["Port"] >> m_port;

    validateSettings();

    if (m_settingsValid) {
        m_imagingConnection = make_shared<OpenIGTLinkConnection>(m_hostName, m_port);
        m_imagingConnection->addMessageHandler(m_imageUpdater);
        m_imagingConnection->addMessageHandler(m_imageWithFocusUpdater);
    }

    return m_settingsValid;
}

bool
CameraInput::settingsValid() const {
    return m_settingsValid;
}

std::shared_ptr<CameraImage>
CameraInput::cameraImage() const {
    return m_cameraImage;
}

bool
CameraInput::startImaging() {
    if (m_imagingConnection == nullptr) {
        return false;
    }

    if (not m_imagingConnection->open()) {
        return false;
    }

    return true;
}

bool
CameraInput::isImaging() {
    if (m_imagingConnection == nullptr) {
        return false;
    }

    return m_imagingConnection->isOpen();
}

void
CameraInput::stopImaging() {
    if (m_imagingConnection == nullptr) {
        return;
    }

    m_imagingConnection->close();
}

void
CameraInput::validateSettings() {
    m_settingsValid = true;

    if (m_hostName.empty()) {
        printErr << "Hostname can't be empty" << endl;
        m_settingsValid = false;
    }

    if (m_port < 0 or m_port > 65535) {
        printErr << "Invalid port number" << endl;
        m_settingsValid = false;
    }
}

void
read(const FileNode &node, CameraInput &cameraInput, const CameraInput &defaultValue) {
    if (node.empty()) {
        cameraInput = defaultValue;
    } else {
        cameraInput.readSettings(node);
    }
}
