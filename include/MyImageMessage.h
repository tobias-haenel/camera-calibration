#pragma once

#include "igtlImageMessage.h"
#include "igtl_myImage.h"

namespace igtl {
class MyImageMessage : public ImageMessage {

public:
    typedef MyImageMessage Self;
    typedef ImageMessage Superclass;
    typedef SmartPointer<Self> Pointer;
    typedef SmartPointer<const Self> ConstPointer;

    igtlTypeMacro(igtl::MyImageMessage, igtl::ImageMessage);
    igtlNewMacro(igtl::MyImageMessage);

    void
    SetFocusValue(const int f) {
        m_FocusValue = f;
    }
    int
    GetFocusValue() {
        return m_FocusValue;
    }

    MyImageMessage();
    ~MyImageMessage();

    virtual int
    GetBodyPackSize();
    virtual int
    PackBody();
    virtual int
    UnpackBody();

protected:
    int m_FocusValue;
};
} // namespace igtl
