package autoquad

import (
	"fmt"

	"github.com/SpaceLeap/go-mavlink/mavlink"
	"github.com/SpaceLeap/go-mavlink/mavlink/common"
)

const (
	PROTOCOL_NAME    = "autoquad"
	PROTOCOL_VERSION = 3
	PROTOCOL_INCLUDE = common.PROTOCOL_NAME
)

func init() {
	mavlink.ProtocolName = PROTOCOL_NAME
	mavlink.ProtocolVersion = PROTOCOL_VERSION

	mavlink.MessageFactory[150] = func() mavlink.Message { return new(AqTelemetryF) }
}

////////////////////////////////////////////////////////////////////////////////
// Enums
////////////////////////////////////////////////////////////////////////////////

// AUTOQUAD_NAV_STATUS: Available operating modes/statuses for AutoQuad flight controller.
// 				Bitmask up to 32 bits. Low side bits for base modes, high side for
// 				additional active features/modifiers/constraints.
const (
	AQ_NAV_STATUS_INIT            = 0 // System is initializing
	AQ_NAV_STATUS_STANDBY         = 0 // System is standing by, not active
	AQ_NAV_STATUS_MANUAL          = 0 // Stabilized, under full manual control
	AQ_NAV_STATUS_ALTHOLD         = 0 // Altitude hold engaged
	AQ_NAV_STATUS_POSHOLD         = 0 // Position hold engaged
	AQ_NAV_STATUS_DVH             = 0 // Dynamic Velocity Hold is active
	AQ_NAV_STATUS_MISSION         = 0 // Autonomous mission execution mode
	AQ_NAV_STATUS_FAILSAFE        = 0 // System is in failsafe recovery mode
	AQ_NAV_STATUS_RTH             = 0 // Automatic Return to Home is active
	AQ_NAV_STATUS_HF_LOCKED       = 0 // Heading-Free locked mode active
	AQ_NAV_STATUS_HF_DYNAMIC      = 0 // Heading-Free dynamic mode active
	AQ_NAV_STATUS_CEILING         = 0 // Ceiling altitude is set
	AQ_NAV_STATUS_CEILING_REACHED = 0 // Craft is at ceiling altitude
)

// MAV_CMD:
const (
	MAV_CMD_AQ_TELEMETRY       = 2 // Start/stop AutoQuad telemetry values stream.
	MAV_CMD_AQ_FOLLOW          = 3 // Command AutoQuad to go to a particular place at a set speed.
	MAV_CMD_AQ_REQUEST_VERSION = 4 // Request AutoQuad firmware version number.
)

////////////////////////////////////////////////////////////////////////////////
// Messages
////////////////////////////////////////////////////////////////////////////////

// Sends up to 20 raw float values.
type AqTelemetryF struct {
	Value1  float32 // value1
	Value2  float32 // value2
	Value3  float32 // value3
	Value4  float32 // value4
	Value5  float32 // value5
	Value6  float32 // value6
	Value7  float32 // value7
	Value8  float32 // value8
	Value9  float32 // value9
	Value10 float32 // value10
	Value11 float32 // value11
	Value12 float32 // value12
	Value13 float32 // value13
	Value14 float32 // value14
	Value15 float32 // value15
	Value16 float32 // value16
	Value17 float32 // value17
	Value18 float32 // value18
	Value19 float32 // value19
	Value20 float32 // value20
	Index   uint16  // Index of message
}

func (self *AqTelemetryF) TypeID() uint8 {
	return 150
}

func (self *AqTelemetryF) TypeName() string {
	return "AQ_TELEMETRY_F"
}

func (self *AqTelemetryF) TypeSize() uint8 {
	return 82
}

func (self *AqTelemetryF) TypeCRCExtra() uint8 {
	return 241
}

func (self *AqTelemetryF) FieldsString() string {
	return fmt.Sprintf("Value1=%d Value2=%d Value3=%d Value4=%d Value5=%d Value6=%d Value7=%d Value8=%d Value9=%d Value10=%d Value11=%d Value12=%d Value13=%d Value14=%d Value15=%d Value16=%d Value17=%d Value18=%d Value19=%d Value20=%d Index=%d", self.Value1, self.Value2, self.Value3, self.Value4, self.Value5, self.Value6, self.Value7, self.Value8, self.Value9, self.Value10, self.Value11, self.Value12, self.Value13, self.Value14, self.Value15, self.Value16, self.Value17, self.Value18, self.Value19, self.Value20, self.Index)
}

func (self *AqTelemetryF) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}
