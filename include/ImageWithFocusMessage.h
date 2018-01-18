#pragma once

#include "igtlImageMessage.h"
#include "igtl_myImage.h"

namespace igtl {
class ImageWithFocusMessage : public ImageMessage {

public:
    typedef ImageWithFocusMessage Self;
    typedef ImageMessage Superclass;
    typedef SmartPointer<Self> Pointer;
    typedef SmartPointer<const Self> ConstPointer;

    igtlTypeMacro(igtl::ImageWithFocusMessage, igtl::ImageMessage);
    igtlNewMacro(igtl::ImageWithFocusMessage);

    void
    SetFocusValue(const int f) {
        m_FocusValue = f;
    }
    int
    GetFocusValue() {
        return m_FocusValue;
    }

    ImageWithFocusMessage();
    ~ImageWithFocusMessage();

    virtual int
    GetBodyPackSize();
    virtual int
    PackBody();
    virtual int
    UnpackBody();

    void
    AllocateScalars();

protected:
    int m_FocusValue;
};
} // namespace igtl
