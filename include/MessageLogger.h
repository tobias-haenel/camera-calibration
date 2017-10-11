#pragma once

#include <igtlBindMessage.h>
#include <igtlCapabilityMessage.h>
#include <igtlColorTableMessage.h>
#include <igtlImageMessage.h>
#include <igtlImageMetaMessage.h>
#include <igtlLabelMetaMessage.h>
#include <igtlMessageHandler.h>
#include <igtlMessageHandlerMacro.h>
#include <igtlNDArrayMessage.h>
#include <igtlPointMessage.h>
#include <igtlPolyDataMessage.h>
#include <igtlPositionMessage.h>
#include <igtlQuaternionTrackingDataMessage.h>
#include <igtlSensorMessage.h>
#include <igtlSessionManager.h>
#include <igtlStatusMessage.h>
#include <igtlStringMessage.h>
#include <igtlTrackingDataMessage.h>
#include <igtlTrajectoryMessage.h>
#include <igtlTransformMessage.h>

#include <cassert>
#include <functional>
#include <memory>
#include <unordered_map>

#include "OpenIGTLinkConnection.h"
#include "ThreadSafePrinter.h"

ThreadSafePrinter & operator <<(ThreadSafePrinter &out, igtl::MessageBase *message);

igtlMessageHandlerClassMacro(igtl::CapabilityMessage, CapabilityLogger, void);
igtlMessageHandlerClassMacro(igtl::ImageMessage, ImageLogger, void);
igtlMessageHandlerClassMacro(igtl::PositionMessage, PositionLogger, void);
igtlMessageHandlerClassMacro(igtl::StatusMessage, StatusLogger, void);
igtlMessageHandlerClassMacro(igtl::TransformMessage, TransformLogger, void);
igtlMessageHandlerClassMacro(igtl::BindMessage, BindLogger, void);
igtlMessageHandlerClassMacro(igtl::RTSBindMessage, RTSBindLogger, void);
igtlMessageHandlerClassMacro(igtl::ColorTableMessage, ColorTableLogger, void);
igtlMessageHandlerClassMacro(igtl::ImageMetaMessage, ImageMetaLogger, void);
igtlMessageHandlerClassMacro(igtl::LabelMetaMessage, LabelMetaLogger, void);
igtlMessageHandlerClassMacro(igtl::NDArrayMessage, NDArrayLogger, void);
igtlMessageHandlerClassMacro(igtl::PointMessage, PointLogger, void);
igtlMessageHandlerClassMacro(igtl::PolyDataMessage, PolyDataLogger, void);
igtlMessageHandlerClassMacro(igtl::QuaternionTrackingDataMessage,
                             QuaternionTrackingDataLogger,
                             void);
igtlMessageHandlerClassMacro(igtl::RTSQuaternionTrackingDataMessage,
                             RTSQuaternionTrackingDataLogger,
                             void);
igtlMessageHandlerClassMacro(igtl::SensorMessage, SensorLogger, void);
igtlMessageHandlerClassMacro(igtl::StringMessage, StringLogger, void);
igtlMessageHandlerClassMacro(igtl::TrackingDataMessage, TrackingDataLogger, void);
igtlMessageHandlerClassMacro(igtl::RTSTrackingDataMessage, RTSTrackingDataLogger, void);
igtlMessageHandlerClassMacro(igtl::TrajectoryMessage, TrajectoryLogger, void);

class MessageLogger {
public:
    MessageLogger();

    void
    registerLoggingMessageHandlers(OpenIGTLinkConnection &igtLink);

private:
    CapabilityLogger::Pointer m_capabilityLogger;
    ImageLogger::Pointer m_imageLogger;
    PositionLogger::Pointer m_positionLogger;
    StatusLogger::Pointer m_statusLogger;
    TransformLogger::Pointer m_transformLogger;
    BindLogger::Pointer m_bindLogger;
    RTSBindLogger::Pointer m_rtsBindLogger;
    ColorTableLogger::Pointer m_colorTableLogger;
    ImageMetaLogger::Pointer m_imageMetaLogger;
    LabelMetaLogger::Pointer m_labelMetaLogger;
    NDArrayLogger::Pointer m_ndArrayLogger;
    PointLogger::Pointer m_pointLogger;
    PolyDataLogger::Pointer m_polyDataLogger;
    QuaternionTrackingDataLogger::Pointer m_quaternionTrackingDataLogger;
    RTSQuaternionTrackingDataLogger::Pointer m_rtsQuaternionTrackingDataLogger;
    SensorLogger::Pointer m_sensorLogger;
    StringLogger::Pointer m_stringLogger;
    TrackingDataLogger::Pointer m_trackingDataLogger;
    RTSTrackingDataLogger::Pointer m_rtsTrackingDataLogger;
    TrajectoryLogger::Pointer m_trajectoryLogger;
};
