#include "ImageWithFocusMessage.h"

#include "igtl_myImage.h"

namespace igtl {
void ImageWithFocusMessage::SetFocusValue(const int f) {
    m_FocusValue = f;
}

int ImageWithFocusMessage::GetFocusValue() {
    return m_FocusValue;
}

ImageWithFocusMessage::ImageWithFocusMessage() : ImageMessage() {
    m_DefaultBodyType = "MYIMAGE";
}

ImageWithFocusMessage::~ImageWithFocusMessage() {}

int
ImageWithFocusMessage::GetBodyPackSize() {
    return GetSubVolumeImageSize() + IGTL_MYIMAGE_HEADER_SIZE;
}

int
ImageWithFocusMessage::PackBody() {
    igtl_myimage_header *image_header = (igtl_myimage_header *) m_ImageHeader;

    image_header->version = IGTL_MYIMAGE_HEADER_VERSION;
    image_header->num_components = this->numComponents;
    image_header->scalar_type = this->scalarType;
    image_header->endian = this->endian;
    image_header->coord = this->coordinate;
    image_header->size[0] = this->dimensions[0];
    image_header->size[1] = this->dimensions[1];
    image_header->size[2] = this->dimensions[2];
    image_header->subvol_offset[0] = this->subOffset[0];
    image_header->subvol_offset[1] = this->subOffset[1];
    image_header->subvol_offset[2] = this->subOffset[2];
    image_header->subvol_size[0] = this->subDimensions[0];
    image_header->subvol_size[1] = this->subDimensions[1];
    image_header->subvol_size[2] = this->subDimensions[2];
    image_header->focus_value = this->m_FocusValue;

    float origin[3];
    float norm_i[3];
    float norm_j[3];
    float norm_k[3];

    for (int i = 0; i < 3; i++) {
        norm_i[i] = matrix[i][0];
        norm_j[i] = matrix[i][1];
        norm_k[i] = matrix[i][2];
        origin[i] = matrix[i][3];
    }

    igtl_image_set_matrix(this->spacing, origin, norm_i, norm_j, norm_k, image_header);

    igtl_image_convert_byte_order(image_header);

    return 1;
}

int
ImageWithFocusMessage::UnpackBody() {

    m_ImageHeader = m_Body;

    igtl_myimage_header *image_header = (igtl_myimage_header *) m_ImageHeader;
    igtl_image_convert_byte_order(image_header);

    if (image_header->version == IGTL_MYIMAGE_HEADER_VERSION) {
        // Image format version 1
        this->scalarType = image_header->scalar_type;
        this->numComponents = image_header->num_components;
        this->endian = image_header->endian;
        this->coordinate = image_header->coord;
        this->dimensions[0] = image_header->size[0];
        this->dimensions[1] = image_header->size[1];
        this->dimensions[2] = image_header->size[2];
        this->subOffset[0] = image_header->subvol_offset[0];
        this->subOffset[1] = image_header->subvol_offset[1];
        this->subOffset[2] = image_header->subvol_offset[2];
        this->subDimensions[0] = image_header->subvol_size[0];
        this->subDimensions[1] = image_header->subvol_size[1];
        this->subDimensions[2] = image_header->subvol_size[2];
        this->m_FocusValue = image_header->focus_value;

        // Set image orientation
        float rspacing[3];
        float origin[3];
        float norm_i[3];
        float norm_j[3];
        float norm_k[3];

        igtl_image_get_matrix(rspacing, origin, norm_i, norm_j, norm_k, image_header);

        for (int i = 0; i < 3; i++) {
            this->spacing[i] = rspacing[i];
            matrix[i][0] = norm_i[i];
            matrix[i][1] = norm_j[i];
            matrix[i][2] = norm_k[i];
            matrix[i][3] = origin[i];
        }

        matrix[3][0] = 0.0;
        matrix[3][1] = 0.0;
        matrix[3][2] = 0.0;
        matrix[3][3] = 1.0;

        m_ImageHeader = m_Body;
        m_Image = &m_ImageHeader[IGTL_MYIMAGE_HEADER_SIZE];

        return 1;
    } else {
        // Incompatible version.
        return 0;
    }
}

void
ImageWithFocusMessage::AllocateScalars() {
    AllocatePack();
    m_ImageHeader = m_Body;
    m_Image = &m_ImageHeader[IGTL_MYIMAGE_HEADER_SIZE];
}
} // namespace igtl
