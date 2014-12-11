package minimal

import (
	"fmt"

	"github.com/SpaceLeap/go-mavlink/mavlink"
)

const (
	PROTOCOL_NAME    = "minimal"
	PROTOCOL_VERSION = 2
)

func Init() {
	for i := range mavlink.MessageFactory {
		mavlink.MessageFactory[i] = nil
	}

	mavlink.ProtocolName = PROTOCOL_NAME
	mavlink.ProtocolVersion = PROTOCOL_VERSION

	mavlink.MessageFactory[0] = func() mavlink.Message { return new(Heartbeat) }
}

////////////////////////////////////////////////////////////////////////////////
// Enums
////////////////////////////////////////////////////////////////////////////////

// MAV_AUTOPILOT: Micro air vehicle / autopilot classes. This identifies the individual model.
const (
	MAV_AUTOPILOT_GENERIC                                      = 0  // Generic autopilot, full support for everything
	MAV_AUTOPILOT_PIXHAWK                                      = 1  // PIXHAWK autopilot, http://pixhawk.ethz.ch
	MAV_AUTOPILOT_SLUGS                                        = 2  // SLUGS autopilot, http://slugsuav.soe.ucsc.edu
	MAV_AUTOPILOT_ARDUPILOTMEGA                                = 3  // ArduPilotMega / ArduCopter, http://diydrones.com
	MAV_AUTOPILOT_OPENPILOT                                    = 4  // OpenPilot, http://openpilot.org
	MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY                       = 5  // Generic autopilot only supporting simple waypoints
	MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY = 6  // Generic autopilot supporting waypoints and other simple navigation commands
	MAV_AUTOPILOT_GENERIC_MISSION_FULL                         = 7  // Generic autopilot supporting the full mission command set
	MAV_AUTOPILOT_INVALID                                      = 8  // No valid autopilot, e.g. a GCS or other MAVLink component
	MAV_AUTOPILOT_PPZ                                          = 9  // PPZ UAV - http://nongnu.org/paparazzi
	MAV_AUTOPILOT_UDB                                          = 10 // UAV Dev Board
	MAV_AUTOPILOT_FP                                           = 11 // FlexiPilot
)

// MAV_TYPE:
const (
	MAV_TYPE_GENERIC         = 0  // Generic micro air vehicle.
	MAV_TYPE_FIXED_WING      = 1  // Fixed wing aircraft.
	MAV_TYPE_QUADROTOR       = 2  // Quadrotor
	MAV_TYPE_COAXIAL         = 3  // Coaxial helicopter
	MAV_TYPE_HELICOPTER      = 4  // Normal helicopter with tail rotor.
	MAV_TYPE_ANTENNA_TRACKER = 5  // Ground installation
	MAV_TYPE_GCS             = 6  // Operator control unit / ground control station
	MAV_TYPE_AIRSHIP         = 7  // Airship, controlled
	MAV_TYPE_FREE_BALLOON    = 8  // Free balloon, uncontrolled
	MAV_TYPE_ROCKET          = 9  // Rocket
	MAV_TYPE_GROUND_ROVER    = 10 // Ground rover
	MAV_TYPE_SURFACE_BOAT    = 11 // Surface vessel, boat, ship
	MAV_TYPE_SUBMARINE       = 12 // Submarine
	MAV_TYPE_HEXAROTOR       = 13 // Hexarotor
	MAV_TYPE_OCTOROTOR       = 14 // Octorotor
	MAV_TYPE_TRICOPTER       = 15 // Octorotor
	MAV_TYPE_FLAPPING_WING   = 16 // Flapping wing
)

// MAV_MODE_FLAG: These flags encode the MAV mode.
const (
	MAV_MODE_FLAG_SAFETY_ARMED         = 128 // 0b10000000 MAV safety set to armed. Motors are enabled / running / can start. Ready to fly.
	MAV_MODE_FLAG_MANUAL_INPUT_ENABLED = 64  // 0b01000000 remote control input is enabled.
	MAV_MODE_FLAG_HIL_ENABLED          = 32  // 0b00100000 hardware in the loop simulation. All motors / actuators are blocked, but internal software is full operational.
	MAV_MODE_FLAG_STABILIZE_ENABLED    = 16  // 0b00010000 system stabilizes electronically its attitude (and optionally position). It needs however further control inputs to move around.
	MAV_MODE_FLAG_GUIDED_ENABLED       = 8   // 0b00001000 guided mode enabled, system flies MISSIONs / mission items.
	MAV_MODE_FLAG_AUTO_ENABLED         = 4   // 0b00000100 autonomous mode enabled, system finds its own goal positions. Guided flag can be set or not, depends on the actual implementation.
	MAV_MODE_FLAG_TEST_ENABLED         = 2   // 0b00000010 system has a test mode enabled. This flag is intended for temporary system tests and should not be used for stable implementations.
	MAV_MODE_FLAG_CUSTOM_MODE_ENABLED  = 1   // 0b00000001 Reserved for future use.
)

// MAV_MODE_FLAG_DECODE_POSITION: These values encode the bit positions of the decode position. These values can be used to read the value of a flag bit by combining the base_mode variable with AND with the flag position value. The result will be either 0 or 1, depending on if the flag is set or not.
const (
	MAV_MODE_FLAG_DECODE_POSITION_SAFETY      = 128 // First bit:  10000000
	MAV_MODE_FLAG_DECODE_POSITION_MANUAL      = 64  // Second bit: 01000000
	MAV_MODE_FLAG_DECODE_POSITION_HIL         = 32  // Third bit:  00100000
	MAV_MODE_FLAG_DECODE_POSITION_STABILIZE   = 16  // Fourth bit: 00010000
	MAV_MODE_FLAG_DECODE_POSITION_GUIDED      = 8   // Fifth bit:  00001000
	MAV_MODE_FLAG_DECODE_POSITION_AUTO        = 4   // Sixt bit:   00000100
	MAV_MODE_FLAG_DECODE_POSITION_TEST        = 2   // Seventh bit: 00000010
	MAV_MODE_FLAG_DECODE_POSITION_CUSTOM_MODE = 1   // Eighth bit: 00000001
)

// MAV_STATE:
const (
	MAV_STATE_UNINIT      = 0 // Uninitialized system, state is unknown.
	MAV_STATE_BOOT        = 1 // System is booting up.
	MAV_STATE_CALIBRATING = 2 // System is calibrating and not flight-ready.
	MAV_STATE_STANDBY     = 3 // System is grounded and on standby. It can be launched any time.
	MAV_STATE_ACTIVE      = 4 // System is active and might be already airborne. Motors are engaged.
	MAV_STATE_CRITICAL    = 5 // System is in a non-normal flight mode. It can however still navigate.
	MAV_STATE_EMERGENCY   = 6 // System is in a non-normal flight mode. It lost control over parts or over the whole airframe. It is in mayday and going down.
	MAV_STATE_POWEROFF    = 7 // System just initialized its power-down sequence, will shut down now.
)

////////////////////////////////////////////////////////////////////////////////
// Messages
////////////////////////////////////////////////////////////////////////////////

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
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}
