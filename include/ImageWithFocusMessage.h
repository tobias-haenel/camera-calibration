#pragma once

#include "igtlImageMessage.h"
#include "igtl_myImage.h"

namespace igtl {
/**
 * @brief The ImageWithFocusMessage class is a custom OpenIGTLink image message that stores a focus
 * value.
 *
 * It was developed by Florian Weidner and is used in his OpenIGTLink imaging server
 */
class ImageWithFocusMessage : public ImageMessage {

public:
    typedef ImageWithFocusMessage Self;
    typedef ImageMessage Superclass;
    typedef SmartPointer<Self> Pointer;
    typedef SmartPointer<const Self> ConstPointer;

    igtlTypeMacro(igtl::ImageWithFocusMessage, igtl::ImageMessage);
    igtlNewMacro(igtl::ImageWithFocusMessage);

    /**
     * @brief Sets a new focus value
     * @param f Focus value
     */
    void
    SetFocusValue(const int f);

    /**
     * @brief Gets the focus value.
     * @return Focus value
     */
    int
    GetFocusValue();

    /**
     * @brief Creates a new ImageWithFocusMessage.
     */
    ImageWithFocusMessage();

    /**
     * @brief Destroys the ImageWithFocusMessage.
     */
    virtual ~ImageWithFocusMessage();

    /**
     * @brief Gets the body size of the message (OpenIGTLinkv2 is used)
     * @return Size in bytes
     */
    virtual int
    GetBodyPackSize();

    /**
     * @brief Stores the current state of this object into a message buffer.
     * @return boolean, indicating success
     */
    virtual int
    PackBody();

    /**
     * @brief Loads the state of this object from a message buffer.
     * @return boolean, indicating success
     */
    virtual int
    UnpackBody();

    /**
     * @brief Allocates the scalars of the image.
     */
    void
    AllocateScalars();

protected:
    /**
     * @brief Discrete focus value (1 - 16000)
     */
    int m_FocusValue;
};
} // namespace igtl
