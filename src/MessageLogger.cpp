#include "MessageLogger.h"

using namespace std;
using namespace igtl;

ThreadSafePrinter &
operator<<(ThreadSafePrinter &out, MessageBase *message) {
    string messageType = message->GetDeviceType();
    string device = message->GetDeviceName();
    TimeStamp::Pointer timeStamp = TimeStamp::New();
    message->GetTimeStamp(timeStamp);

    out << "Logging " << messageType << " message:" << endl;
    out << "  Device: " << device << endl;
    out << "  Time stamp: " << timeStamp->GetTimeStamp() << endl;
    return out;
}

int
CapabilityLogger::Process(CapabilityMessage *capability, void *) {
    vector<string> types = capability->GetTypes();

    ThreadSafePrinter out{cout};
    out << capability;
    out << "  Interpretable message types: ";
    for (size_t i = 0; i < types.size(); ++i) {
        if (i != 0) {
            out << ", ";
        }
        out << types[i];
    }
    out << endl;

    return 1;
}

int
ImageLogger::Process(ImageMessage *image, void *) {
    int numComponents = image->GetNumComponents();
    int scalarType = image->GetScalarType();
    int endian = image->GetEndian();
    int coordinateSystem = image->GetCoordinateSystem();
    int dimensions[3];
    image->GetDimensions(dimensions);
    Matrix4x4 m;
    image->GetMatrix(m);
    int subDimensions[3], subOffsets[3];
    image->GetSubVolume(subDimensions, subOffsets);

    ThreadSafePrinter out{cout};
    out << image;
    out << "  Number of image components: " << numComponents << endl;
    out << "  Scalar type: ";
    switch (scalarType) {
    case ImageMessage::TYPE_INT8:
        out << "int8" << endl;
        break;
    case ImageMessage::TYPE_UINT8:
        out << "uint8" << endl;
        break;
    case ImageMessage::TYPE_INT16:
        out << "int16" << endl;
        break;
    case ImageMessage::TYPE_UINT16:
        out << "uint16" << endl;
        break;
    case ImageMessage::TYPE_INT32:
        out << "int32" << endl;
        break;
    case ImageMessage::TYPE_UINT32:
        out << "uint32" << endl;
        break;
    case ImageMessage::TYPE_FLOAT32:
        out << "float32" << endl;
        break;
    case ImageMessage::TYPE_FLOAT64:
        out << "float64" << endl;
        break;
    default:
        out << "unknown" << endl;
        break;
    }
    out << "  Data endianness: "
        << (endian == ImageMessage::ENDIAN_BIG
                ? "big"
                : (endian == ImageMessage::ENDIAN_LITTLE ? "little" : "unknown"))
        << endl;
    out << "  Coordinate system: "
        << (coordinateSystem == ImageMessage::COORDINATE_LPS
                ? "LPS"
                : (coordinateSystem == ImageMessage::COORDINATE_RAS ? "RAS" : "unknown"))
        << endl;
    out << "  Dimensions: " << dimensions[0] << 'x' << dimensions[1] << 'x' << dimensions[2]
        << endl;
    out << "  Transformation matrix:" << endl << scientific << showpos;
    out << "    " << m[0][0] << ", " << m[0][1] << ", " << m[0][2] << ", " << m[0][3] << endl;
    out << "    " << m[1][0] << ", " << m[1][1] << ", " << m[1][2] << ", " << m[1][3] << endl;
    out << "    " << m[2][0] << ", " << m[2][1] << ", " << m[2][2] << ", " << m[2][3] << endl;
    out << "    " << m[3][0] << ", " << m[3][1] << ", " << m[3][2] << ", " << m[3][3] << endl;
    out << defaultfloat << noshowpos;
    out << "  Starting indices for subvolume: " << subOffsets[0] << ", " << subOffsets[1] << ", "
        << subOffsets[2] << endl;
    out << "  Subvolume dimensions: " << subDimensions[0] << 'x' << subDimensions[1] << 'x'
        << subDimensions[2] << endl;

    return 1;
}

int
PositionLogger::Process(PositionMessage *position, void *) {
    float pos[3];
    position->GetPosition(pos);
    float quat[4];
    position->GetQuaternion(quat);

    ThreadSafePrinter out{cout};
    out << position;
    out << "  Position: " << pos[0] << ", " << pos[1] << ", " << pos[2] << endl;
    out << "  Orientation quaternion: " << quat[0] << ", " << quat[1] << ", " << quat[2] << ", "
        << quat[3] << endl;

    return 1;
}

int
StatusLogger::Process(StatusMessage *status, void *) {
    int code = status->GetCode();
    igtlInt64 subCode = status->GetSubCode();
    string errorName = status->GetErrorName();
    string statusString = status->GetStatusString();

    ThreadSafePrinter out{cout};
    out << status;
    out << "  Code: " << code << " (";
    switch (code) {
    case StatusMessage::STATUS_INVALID:
        out << "invalid";
        break;
    case StatusMessage::STATUS_OK:
        out << "OK";
        break;
    case StatusMessage::STATUS_UNKNOWN_ERROR:
        out << "unknown error";
        break;
    case StatusMessage::STATUS_PANICK_MODE:
        out << "panic mode";
        break;
    case StatusMessage::STATUS_NOT_FOUND:
        out << "not found";
        break;
    case StatusMessage::STATUS_ACCESS_DENIED:
        out << "access denied";
        break;
    case StatusMessage::STATUS_BUSY:
        out << "busy";
        break;
    case StatusMessage::STATUS_TIME_OUT:
        out << "time out";
        break;
    case StatusMessage::STATUS_OVERFLOW:
        out << "overflow";
        break;
    case StatusMessage::STATUS_CHECKSUM_ERROR:
        out << "checksum error";
        break;
    case StatusMessage::STATUS_CONFIG_ERROR:
        out << "configuration error";
        break;
    case StatusMessage::STATUS_RESOURCE_ERROR:
        out << "not enough resources";
        break;
    case StatusMessage::STATUS_UNKNOWN_INSTRUCTION:
        out << "unknown instruction";
        break;
    case StatusMessage::STATUS_NOT_READY:
        out << "device not ready";
        break;
    case StatusMessage::STATUS_MANUAL_MODE:
        out << "manual mode";
        break;
    case StatusMessage::STATUS_DISABLED:
        out << "device disabled";
        break;
    case StatusMessage::STATUS_NOT_PRESENT:
        out << "device not present";
        break;
    case StatusMessage::STATUS_UNKNOWN_VERSION:
        out << "device version not known";
        break;
    case StatusMessage::STATUS_HARDWARE_FAILURE:
        out << "hardware failure";
        break;
    case StatusMessage::STATUS_SHUT_DOWN:
        out << "shut down in progress";
        break;
    }
    out << ")" << endl;
    out << "  Sub-code: " << subCode << endl;
    out << "  Error name: " << errorName << endl;
    out << "  Message: " << statusString << endl;

    return 1;
}

int
TransformLogger::Process(TransformMessage *transform, void *) {
    Matrix4x4 m;
    transform->GetMatrix(m);

    ThreadSafePrinter out{cout};
    out << transform;
    out << "  Transformation matrix:" << endl;
    out << "    " << m[0][0] << ", " << m[0][1] << ", " << m[0][2] << ", " << m[0][3] << endl;
    out << "    " << m[1][0] << ", " << m[1][1] << ", " << m[1][2] << ", " << m[1][3] << endl;
    out << "    " << m[2][0] << ", " << m[2][1] << ", " << m[2][2] << ", " << m[2][3] << endl;
    out << "    " << m[3][0] << ", " << m[3][1] << ", " << m[3][2] << ", " << m[3][3] << endl;

    return 1;
}

int
BindLogger::Process(BindMessage *bindMsg, void *) {
    ThreadSafePrinter out{cout};
    out << bindMsg;
    for (unsigned i = 0; static_cast<int>(i) < bindMsg->GetNumberOfChildMessages(); ++i) {
        string messageType = bindMsg->GetChildMessageType(i);
        out << "  Message " << i + 1 << ": " << messageType << endl;

        // BUG GetChildMessage doesn't seem to work properly, it results in free errors
    }

    return 1;
}

int
RTSBindLogger::Process(RTSBindMessage *rtsBind, void *) {
    igtlUint8 status = rtsBind->GetStatus();

    ThreadSafePrinter out{cout};
    out << rtsBind;
    out << "  Status: "
        << (status == RTSBindMessage::STATUS_SUCCESS
                ? "success"
                : (status == RTSBindMessage::STATUS_ERROR ? "error" : "unknown"))
        << endl;

    return 1;
}

int
ColorTableLogger::Process(ColorTableMessage *colorTable, void *) {
    int indexType = colorTable->GetIndexType();
    int mapType = colorTable->GetMapType();
    int size = colorTable->GetColorTableSize();

    ThreadSafePrinter out{cout};
    out << colorTable;
    out << "  Index type: "
        << (indexType == ColorTableMessage::INDEX_UINT8
                ? "uint8"
                : (indexType == ColorTableMessage::INDEX_UINT16 ? "uint16" : "unknown"))
        << endl;
    out << "  Map type: ";
    switch (mapType) {
    case ColorTableMessage::MAP_UINT8: {
        out << "uint8" << endl;
        break;
    }
    case ColorTableMessage::MAP_UINT16: {
        out << "uint16" << endl;
        break;
    }
    case ColorTableMessage::MAP_RGB: {
        out << "RGB" << endl;
        break;
    }
    }
    out << "Table size: " << size << endl;

    return 1;
}

int
ImageMetaLogger::Process(ImageMetaMessage *imageMeta, void *) {
    ThreadSafePrinter out{cout};
    out << imageMeta;
    for (int i = 0; i < imageMeta->GetNumberOfImageMetaElement(); ++i) {
        out << "  Image Meta Element " << i + 1 << ":" << endl;

        ImageMetaElement::Pointer metaElement;
        imageMeta->GetImageMetaElement(i, metaElement);

        string name = metaElement->GetName();
        string id = metaElement->GetDeviceName();
        string modality = metaElement->GetModality();
        string patientName = metaElement->GetPatientName();
        string patientId = metaElement->GetPatientID();
        TimeStamp::Pointer scanTime;
        metaElement->GetTimeStamp(scanTime);
        igtlUint8 scalarType = metaElement->GetScalarType();
        igtlUint16 dimensions[3];
        metaElement->GetSize(dimensions);

        out << "    Name: " << name << endl;
        out << "    ID: " << id << endl;
        out << "    Modality: " << modality << endl;
        out << "    Patient name: " << patientName << endl;
        out << "    Patient ID: " << patientId << endl;
        out << "    Scan time: " << scanTime->GetTimeStampInNanoseconds() << endl;
        out << "    Scalar type: ";
        switch (scalarType) {
        case ImageMessage::TYPE_INT8:
            out << "int8" << endl;
            break;
        case ImageMessage::TYPE_UINT8:
            out << "uint8" << endl;
            break;
        case ImageMessage::TYPE_INT16:
            out << "int16" << endl;
            break;
        case ImageMessage::TYPE_UINT16:
            out << "uint16" << endl;
            break;
        case ImageMessage::TYPE_INT32:
            out << "int32" << endl;
            break;
        case ImageMessage::TYPE_UINT32:
            out << "uint32" << endl;
            break;
        case ImageMessage::TYPE_FLOAT32:
            out << "float32" << endl;
            break;
        case ImageMessage::TYPE_FLOAT64:
            out << "float64" << endl;
            break;
        default:
            out << "unknown" << endl;
            break;
        }
        out << "    Dimensions: " << dimensions[0] << 'x' << dimensions[1] << 'x' << dimensions[2]
            << endl;
    }

    return 1;
}

int
LabelMetaLogger::Process(LabelMetaMessage *labelMeta, void *) {
    ThreadSafePrinter out{cout};
    out << labelMeta;
    for (int i = 0; i < labelMeta->GetNumberOfLabelMetaElement(); ++i) {
        out << "  Label Meta Element " << i + 1 << ":" << endl;

        LabelMetaElement::Pointer metaElement;
        labelMeta->GetLabelMetaElement(i, metaElement);

        string name = metaElement->GetName();
        string id = metaElement->GetDeviceName();
        igtlUint8 label = metaElement->GetLabel();
        igtlUint8 color[4];
        metaElement->GetRGBA(color);
        igtlUint16 dimensions[3];
        metaElement->GetSize(dimensions);
        string ownerImageId = metaElement->GetOwner();

        out << "    Name: " << name << endl;
        out << "    ID: " << id << endl;
        out << "    Label: " << static_cast<int>(label) << endl;
        out << "    Color: " << static_cast<int>(color[0]) << ", " << static_cast<int>(color[1])
            << ", " << static_cast<int>(color[2]) << ", " << static_cast<int>(color[3]) << " (RGBA)"
            << endl;
        out << "    Dimensions: " << dimensions[0] << 'x' << dimensions[1] << 'x'
            << static_cast<int>(dimensions[2]) << endl;
        out << "    Owner image ID: " << ownerImageId << endl;
    }

    return 1;
}

int
NDArrayLogger::Process(NDArrayMessage *ndArray, void *) {
    int type = ndArray->GetType();
    ArrayBase *array = ndArray->GetArray();
    if (array == nullptr) {
        return 0;
    }
    ArrayBase::IndexType dimensions = array->GetSize();

    ThreadSafePrinter out{cout};
    out << ndArray;
    out << "  Type: ";
    switch (type) {
    case NDArrayMessage::TYPE_INT8:
        out << "int8" << endl;
        break;
    case NDArrayMessage::TYPE_UINT8:
        out << "uint8" << endl;
        break;
    case NDArrayMessage::TYPE_INT16:
        out << "int16" << endl;
        break;
    case NDArrayMessage::TYPE_UINT16:
        out << "uint16" << endl;
        break;
    case NDArrayMessage::TYPE_INT32:
        out << "int32" << endl;
        break;
    case NDArrayMessage::TYPE_UINT32:
        out << "uint32" << endl;
        break;
    case NDArrayMessage::TYPE_FLOAT32:
        out << "float32" << endl;
        break;
    case NDArrayMessage::TYPE_FLOAT64:
        out << "float64" << endl;
        break;
    case NDArrayMessage::TYPE_COMPLEX:
        out << "complex" << endl;
        break;
    default:
        out << "unknown" << endl;
        break;
    }
    out << "  Dimensions: ";
    size_t i = 0;
    if (dimensions.size() >= 1) {
        out << dimensions[i];
    } else {
        out << "-";
    }
    for (i = 1; i < dimensions.size(); ++i) {
        out << "x" << dimensions[i];
    }
    out << endl;

    return 1;
}

int
PointLogger::Process(PointMessage *points, void *) {
    ThreadSafePrinter out{cout};
    out << points;
    for (int i = 0; i < points->GetNumberOfPointElement(); ++i) {
        out << "  Point element " << i + 1 << endl;

        PointElement::Pointer point;
        points->GetPointElement(i, point);

        string name = point->GetName();
        string group = point->GetGroupName();
        igtlUint8 color[4];
        point->GetRGBA(color);
        float position[3];
        point->GetPosition(position);
        float radius = point->GetRadius();
        string owner = point->GetOwner();

        out << "    Name: " << name << endl;
        out << "    Group: " << group << endl;
        out << "    Color: " << static_cast<int>(color[0]) << ", " << static_cast<int>(color[1])
            << ", " << static_cast<int>(color[2]) << ", " << static_cast<int>(color[3]) << " (RGBA)"
            << endl;
        out << "    Position: " << position[0] << ", " << position[1] << ", " << position[2]
            << endl;
        out << "    Radius: " << radius << endl;
        out << "    Owner image id: " << owner << endl;
    }

    return 1;
}

int
PolyDataLogger::Process(PolyDataMessage *polyData, void *) {
    int points = polyData->GetPoints()->GetNumberOfPoints();
    igtlUint32 lines = polyData->GetLines()->GetNumberOfCells();
    igtlUint32 polygons = polyData->GetPolygons()->GetNumberOfCells();
    igtlUint32 triangles = polyData->GetTriangleStrips()->GetNumberOfCells();

    ThreadSafePrinter out{cout};
    out << polyData;
    out << "  Points: " << points << endl;
    out << "  Lines: " << lines << endl;
    out << "  Polygons: " << polygons << endl;
    out << "  Triangles: " << triangles << endl;
    for (unsigned i = 0; static_cast<int>(i) < polyData->GetNumberOfAttributes(); ++i) {
        out << "  Attribute " << i + 1 << ":" << endl;

        PolyDataAttribute *attribute = polyData->GetAttribute(i);
        igtlUint8 type = attribute->GetType();
        igtlUint32 components = attribute->GetNumberOfComponents();
        igtlUint32 size = attribute->GetSize();
        string name = attribute->GetName();

        out << "    Type: ";
        switch (type) {
        case PolyDataAttribute::POINT_SCALAR:
            out << "point scalar" << endl;
            break;
        case PolyDataAttribute::POINT_VECTOR:
            out << "point vector" << endl;
            break;
        case PolyDataAttribute::POINT_NORMAL:
            out << "point normal" << endl;
            break;
        case PolyDataAttribute::POINT_TENSOR:
            out << "point tensor" << endl;
            break;
        case PolyDataAttribute::POINT_RGBA:
            out << "point rgba" << endl;
            break;
        case PolyDataAttribute::CELL_SCALAR:
            out << "cell scalar" << endl;
            break;
        case PolyDataAttribute::CELL_VECTOR:
            out << "cell vector" << endl;
            break;
        case PolyDataAttribute::CELL_NORMAL:
            out << "cell normal" << endl;
            break;
        case PolyDataAttribute::CELL_TENSOR:
            out << "cell tensor" << endl;
            break;
        case PolyDataAttribute::CELL_RGBA:
            out << "cell rgba" << endl;
            break;
        default:
            out << "unknown" << endl;
            break;
        }
        out << "    Components: " << components << endl;
        out << "    Size: " << size << endl;
        out << "    Name: " << name << endl;
    }

    return 1;
}

int
QuaternionTrackingDataLogger::Process(QuaternionTrackingDataMessage *quatTrackData, void *) {
    ThreadSafePrinter out{cout};
    out << quatTrackData;
    for (int i = 0; i < quatTrackData->GetNumberOfQuaternionTrackingDataElements(); ++i) {
        out << "  Quaternion Tracking Element " << i + 1 << endl;

        QuaternionTrackingDataElement::Pointer quatTrackElement;
        quatTrackData->GetQuaternionTrackingDataElement(i, quatTrackElement);
        string name = quatTrackElement->GetName();
        igtlUint8 type = quatTrackElement->GetType();
        float position[3];
        quatTrackElement->GetPosition(position);
        float quat[4];
        quatTrackElement->GetQuaternion(quat);

        out << "    Name: " << name << endl;
        out << "    Type: ";
        switch (type) {
        case QuaternionTrackingDataElement::TYPE_TRACKER:
            out << "tracker" << endl;
            break;
        case QuaternionTrackingDataElement::TYPE_6D:
            out << "6D instrument" << endl;
            break;
        case QuaternionTrackingDataElement::TYPE_3D:
            out << "3D instrument" << endl;
            break;
        case QuaternionTrackingDataElement::TYPE_5D:
            out << "5D instrument" << endl;
            break;
        default:
            out << "unknown" << endl;
            break;
        }
        out << "    Position: " << position[0] << ", " << position[1] << ", " << position[2]
            << endl;
        out << "    Quaternion: " << quat[0] << ", " << quat[1] << ", " << quat[2] << ", "
            << quat[3] << endl;
    }

    return 1;
}

int
RTSQuaternionTrackingDataLogger::Process(RTSQuaternionTrackingDataMessage *rtsQTrack, void *) {
    igtlUint8 status = rtsQTrack->GetStatus();

    ThreadSafePrinter out{cout};
    out << rtsQTrack;
    out << "  Status: "
        << (status == RTSQuaternionTrackingDataMessage::STATUS_SUCCESS
                ? "success"
                : (status == RTSQuaternionTrackingDataMessage::STATUS_ERROR ? "error" : "unknown"))
        << endl;

    return 1;
}

int
SensorLogger::Process(SensorMessage *sensor, void *) {
    igtlUnit unit = sensor->GetUnit();

    ThreadSafePrinter out{cout};
    out << sensor;
    out << "  Unit: " << unit << endl;
    for (unsigned i = 0; i < sensor->GetLength(); ++i) {
        out << "  Value " << i + 1 << ": " << sensor->GetValue(i) << endl;
    }

    return 1;
}

int
StringLogger::Process(StringMessage *stringMsg, void *) {
    igtlUint16 encoding = stringMsg->GetEncoding();
    char const *stringValue = stringMsg->GetString();

    ThreadSafePrinter out{cout};
    out << stringMsg;
    out << "  Encoding type (MIB enum value): " << encoding << endl;
    out << "  String value: " << endl;
    out << "    \"" << stringValue << '"' << endl;

    return 1;
}

int
TrackingDataLogger::Process(TrackingDataMessage *trackData, void *) {
    ThreadSafePrinter out{cout};
    out << trackData;
    for (int i = 0; i < trackData->GetNumberOfTrackingDataElements(); ++i) {
        out << "  Tracking Element " << i + 1 << endl;

        TrackingDataElement::Pointer trackElement;
        trackData->GetTrackingDataElement(i, trackElement);
        string name = trackElement->GetName();
        igtlUint8 type = trackElement->GetType();
        float position[3];
        trackElement->GetPosition(position);
        Matrix4x4 m;
        trackElement->GetMatrix(m);

        out << "    Name: " << name << endl;
        out << "    Type: ";
        switch (type) {
        case QuaternionTrackingDataElement::TYPE_TRACKER:
            out << "tracker" << endl;
            break;
        case QuaternionTrackingDataElement::TYPE_6D:
            out << "6D instrument" << endl;
            break;
        case QuaternionTrackingDataElement::TYPE_3D:
            out << "3D instrument" << endl;
            break;
        case QuaternionTrackingDataElement::TYPE_5D:
            out << "5D instrument" << endl;
            break;
        default:
            out << "unknown" << endl;
            break;
        }
        out << "    Position: " << position[0] << ", " << position[1] << ", " << position[2]
            << endl;
        out << "    Transformation matrix:" << endl << scientific << showpos;
        out << "      " << m[0][0] << ", " << m[0][1] << ", " << m[0][2] << ", " << m[0][3] << endl;
        out << "      " << m[1][0] << ", " << m[1][1] << ", " << m[1][2] << ", " << m[1][3] << endl;
        out << "      " << m[2][0] << ", " << m[2][1] << ", " << m[2][2] << ", " << m[2][3] << endl;
        out << "      " << m[3][0] << ", " << m[3][1] << ", " << m[3][2] << ", " << m[3][3] << endl;
        out << defaultfloat << noshowpos;
    }

    return 1;
}

int
RTSTrackingDataLogger::Process(RTSTrackingDataMessage *rtsTrack, void *) {
    igtlUint8 status = rtsTrack->GetStatus();

    ThreadSafePrinter out{cout};
    out << rtsTrack;
    out << "  Status: "
        << (status == RTSTrackingDataMessage::STATUS_SUCCESS
                ? "success"
                : (status == RTSTrackingDataMessage::STATUS_ERROR ? "error" : "unknown"))
        << endl;

    return 1;
}

int
TrajectoryLogger::Process(TrajectoryMessage *trajectory, void *) {
    ThreadSafePrinter out{cout};
    out << trajectory;
    for (int i = 0; i < trajectory->GetNumberOfTrajectoryElement(); ++i) {
        out << "  Trajectory element " << i + 1 << ":" << endl;

        TrajectoryElement::Pointer trajectoryElement;
        trajectory->GetTrajectoryElement(i, trajectoryElement);

        string name = trajectoryElement->GetName();
        string group = trajectoryElement->GetGroupName();
        igtlUint8 type = trajectoryElement->GetType();
        igtlUint8 color[4];
        trajectoryElement->GetRGBA(color);
        float entryPosition[3];
        trajectoryElement->GetEntryPosition(entryPosition);
        float targetPosition[3];
        trajectoryElement->GetTargetPosition(targetPosition);
        float radius = trajectoryElement->GetRadius();
        string owner = trajectoryElement->GetOwner();

        out << "    Name: " << name << endl;
        out << "    Group: " << group << endl;
        out << "    Color: " << static_cast<int>(color[0]) << ", " << static_cast<int>(color[1])
            << ", " << static_cast<int>(color[2]) << ", " << static_cast<int>(color[3]) << " (RGBA)"
            << endl;
        out << "    Type: ";
        switch (type) {
        case TrajectoryElement::TYPE_ENTRY_ONLY:
            out << "entry only" << endl;
            break;
        case TrajectoryElement::TYPE_TARGET_ONLY:
            out << "target only" << endl;
            break;
        case TrajectoryElement::TYPE_ENTRY_TARGET:
            out << "entry and target" << endl;
            break;
        default:
            break;
        }
        out << "    Entry position: " << entryPosition[0] << ", " << entryPosition[1] << ", "
            << entryPosition[2] << endl;
        out << "    Target position: " << targetPosition[0] << ", " << targetPosition[1] << ", "
            << targetPosition[2] << endl;
        out << "    Radius: " << radius << endl;
        out << "    Owner image ID: " << owner << endl;
    }

    return 1;
}

MessageLogger::MessageLogger()
    : m_capabilityLogger{CapabilityLogger::New()},
      m_imageLogger{ImageLogger::New()},
      m_positionLogger{PositionLogger::New()},
      m_statusLogger{StatusLogger::New()},
      m_transformLogger{TransformLogger::New()},
      m_bindLogger{BindLogger::New()},
      m_rtsBindLogger{RTSBindLogger::New()},
      m_colorTableLogger{ColorTableLogger::New()},
      m_imageMetaLogger{ImageMetaLogger::New()},
      m_labelMetaLogger{LabelMetaLogger::New()},
      m_ndArrayLogger{NDArrayLogger::New()},
      m_pointLogger{PointLogger::New()},
      m_polyDataLogger{PolyDataLogger::New()},
      m_quaternionTrackingDataLogger{QuaternionTrackingDataLogger::New()},
      m_rtsQuaternionTrackingDataLogger{RTSQuaternionTrackingDataLogger::New()},
      m_sensorLogger{SensorLogger::New()},
      m_stringLogger{StringLogger::New()},
      m_trackingDataLogger{TrackingDataLogger::New()},
      m_rtsTrackingDataLogger{RTSTrackingDataLogger::New()},
      m_trajectoryLogger{TrajectoryLogger::New()} {}

void
MessageLogger::registerLoggingMessageHandlers(OpenIGTLinkConnection &igtLink) {
    igtLink.addMessageHandler(m_capabilityLogger);
    igtLink.addMessageHandler(m_imageLogger);
    igtLink.addMessageHandler(m_positionLogger);
    igtLink.addMessageHandler(m_statusLogger);
    igtLink.addMessageHandler(m_transformLogger);
    igtLink.addMessageHandler(m_bindLogger);
    igtLink.addMessageHandler(m_rtsBindLogger);
    igtLink.addMessageHandler(m_colorTableLogger);
    igtLink.addMessageHandler(m_imageMetaLogger);
    igtLink.addMessageHandler(m_labelMetaLogger);
    igtLink.addMessageHandler(m_ndArrayLogger);
    igtLink.addMessageHandler(m_pointLogger);
    igtLink.addMessageHandler(m_polyDataLogger);
    igtLink.addMessageHandler(m_quaternionTrackingDataLogger);
    igtLink.addMessageHandler(m_rtsQuaternionTrackingDataLogger);
    igtLink.addMessageHandler(m_sensorLogger);
    igtLink.addMessageHandler(m_stringLogger);
    igtLink.addMessageHandler(m_trackingDataLogger);
    igtLink.addMessageHandler(m_rtsTrackingDataLogger);
    igtLink.addMessageHandler(m_trajectoryLogger);
}
