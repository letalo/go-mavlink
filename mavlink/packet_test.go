package mavlink

import (
	"fmt"
	"reflect"
	"testing"
)

func TestParsePacket(t *testing.T) {
	ProtocolName = PROTOCOL_NAME
	ProtocolVersion = PROTOCOL_VERSION
	MessageFactory[0] = func() Message { return new(Heartbeat) }

	message := &Heartbeat{
		CustomMode:     666,
		Type:           1,
		Autopilot:      2,
		BaseMode:       3,
		SystemStatus:   4,
		MavlinkVersion: PROTOCOL_VERSION,
	}

	packet := NewPacket(0, 1, 2, message)
	s := packet.String()

	if s != "HEARTBEAT(0){SystemID=0 ComponentID=1 PacketSequence=2 PayloadLength=9 CustomMode=666 Type=1 Autopilot=2 BaseMode=3 SystemStatus=4 MavlinkVersion=2}" {
		t.Fatal("packet.String() returned invalid string")
	}

	parsedPacket, err := ParsePacket(s)
	if err != nil {
		t.Fatal(err)
	}

	if parsedPacket.Header != packet.Header {
		t.Fatalf("Invalid header: %#v != %#v", parsedPacket.Header, packet.Header)
	}
	if !reflect.DeepEqual(packet.Message, parsedPacket.Message) {
		t.Fatalf("Invalid message: %#v != %#v", parsedPacket.Message, packet.Message)
	}
}

// Minimal MAVLink implementation:
// (can't import ./minimal because of dependency cycle)

const (
	PROTOCOL_NAME    = "minimal"
	PROTOCOL_VERSION = 2
)

// The heartbeat message shows that a system is present and responding. The type of the MAV and Autopilot hardware allow the receiving system to treat further messages from this system appropriate (e.g. by laying out the user interface based on the autopilot).
type Heartbeat struct {
	CustomMode     uint32 // A bitfield for use for autopilot-specific flags.
	Type           uint8  // Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
	Autopilot      uint8  // Autopilot type / class. defined in MAV_AUTOPILOT ENUM
	BaseMode       uint8  // System mode bitfield, see MAV_MODE_FLAGS ENUM in mavlink/include/mavlink_types.h
	SystemStatus   uint8  // System status flag, see MAV_STATE ENUM
	MavlinkVersion uint8  // MAVLink version
}

func NewHeartbeat() *Heartbeat {
	return &Heartbeat{MavlinkVersion: PROTOCOL_VERSION}
}

func (self *Heartbeat) TypeID() uint8 {
	return 0
}

func (self *Heartbeat) TypeName() string {
	return "HEARTBEAT"
}

func (self *Heartbeat) TypeSize() uint8 {
	return 9
}

func (self *Heartbeat) TypeCRCExtra() uint8 {
	return 239
}

func (self *Heartbeat) FieldsString() string {
	return fmt.Sprintf("CustomMode=%d Type=%d Autopilot=%d BaseMode=%d SystemStatus=%d MavlinkVersion=%d", self.CustomMode, self.Type, self.Autopilot, self.BaseMode, self.SystemStatus, self.MavlinkVersion)
}

func (self *Heartbeat) String() string {
	return NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}
