#include "CameraInput.h"

using namespace std;
using namespace cv;
using namespace igtl;

const char *
ImageUpdater::GetMessageType() {
    return this->m_Message->GetDeviceType();
}

int
ImageUpdater::Process(MyImageMessage *message, void *data) {
    CameraImage *cameraImage = static_cast<CameraImage *>(data);
    if (cameraImage == nullptr) {
        return 0;
    }

    cameraImage->updateFromMessage(message);
    return 1;
}

int
ImageUpdater::ReceiveMessage(Socket *socket, MessageBase *header, int pos) {
    if (pos == 0) /* New body */
    {
        this->m_Message->SetMessageHeader(header);
        this->m_Message->AllocatePack();
    }
    int s = socket->Receive((void *) ((char *) this->m_Message->GetPackBodyPointer() + pos),
                            this->m_Message->GetPackBodySize() - pos);
    if (s < 0) /* Time out */
    {
        return pos;
    }
    if (s + pos >= this->m_Message->GetPackBodySize()) {
        int r = this->m_Message->Unpack(this->m_CheckCRC);
        if (r) {
            Process(this->m_Message, this->m_Data);
        } else {
            return -1;
        }
    }
    return s + pos; /* return current position in the body */
}

void
ImageUpdater::CheckCRC(int i) {
    if (i == 0) {
        this->m_CheckCRC = 0;
    } else {
        this->m_CheckCRC = 1;
    }
}

void
ImageUpdater::SetData(void *p) {
    this->m_Data = p;
}

void *
ImageUpdater::GetData() {
    return this->m_Data;
}

ImageUpdater::ImageUpdater() {
    this->m_Message = igtl::MyImageMessage::New();
    this->m_CheckCRC = 1;
    this->m_Data = nullptr;
}

CameraInput::CameraInput()
    : m_imageUpdater{ImageUpdater::New()}, m_cameraImage{make_shared<CameraImage>()} {
    m_imageUpdater->SetData(m_cameraImage.get());
    m_imageUpdater->CheckCRC(0);
}

bool
CameraInput::readSettings(const FileNode &node) {
    bool flipHorizontal, flipVertical;
    node["HostName"] >> m_hostName;
    node["Port"] >> m_port;
    node["FlipHorizontal"] >> flipHorizontal;
    node["FlipVertical"] >> flipVertical;

    validateSettings();

    if (m_settingsValid) {
        m_cameraImage->setFlipHorizontal(flipHorizontal);
        m_cameraImage->setFlipVertical(flipVertical);
        m_imagingConnection = make_shared<OpenIGTLinkConnection>(m_hostName, m_port);
        m_imagingConnection->addMessageHandler(m_imageUpdater);
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
