package ualberta

import (
	"github.com/SpaceLeap/go-mavlink/mavlink"
	"github.com/SpaceLeap/go-mavlink/mavlink/common"
)

const (
	PROTOCOL_NAME    = "ualberta"
	PROTOCOL_VERSION = ""
	PROTOCOL_INCLUDE = common.PROTOCOL_NAME
)

func init() {
	mavlink.ProtocolName = PROTOCOL_NAME
	mavlink.ProtocolVersion = PROTOCOL_VERSION

	mavlink.MessageFactory[220] = func() mavlink.Message { return new(NavFilterBias) }
	mavlink.MessageFactory[221] = func() mavlink.Message { return new(RadioCalibration) }
	mavlink.MessageFactory[222] = func() mavlink.Message { return new(UalbertaSysStatus) }
}

////////////////////////////////////////////////////////////////////////////////
// Enums
////////////////////////////////////////////////////////////////////////////////

// UALBERTA_AUTOPILOT_MODE: Available autopilot modes for ualberta uav
const (
	MODE_MANUAL_DIRECT = 0 // Raw input pulse widts sent to output
	MODE_MANUAL_SCALED = 0 // Inputs are normalized using calibration, the converted back to raw pulse widths for output
	MODE_AUTO_PID_ATT  = 0 //  dfsdfs
	MODE_AUTO_PID_VEL  = 0 //  dfsfds
	MODE_AUTO_PID_POS  = 0 //  dfsdfsdfs
)

// UALBERTA_NAV_MODE: Navigation filter mode
const (
	NAV_AHRS_INIT    = 0 //
	NAV_AHRS         = 0 // AHRS mode
	NAV_INS_GPS_INIT = 0 // INS/GPS initialization mode
	NAV_INS_GPS      = 0 // INS/GPS mode
)

// UALBERTA_PILOT_MODE: Mode currently commanded by pilot
const (
	PILOT_MANUAL = 0 //  sdf
	PILOT_AUTO   = 0 //  dfs
	PILOT_ROTO   = 0 //  Rotomotion mode
)

////////////////////////////////////////////////////////////////////////////////
// Messages
////////////////////////////////////////////////////////////////////////////////

// Accelerometer and Gyro biases from the navigation filter
type NavFilterBias struct {
	Usec   uint64  // Timestamp (microseconds)
	Gyro2  float32 // b_f[2]
	Gyro1  float32 // b_f[1]
	Gyro0  float32 // b_f[0]
	Accel2 float32 // b_f[2]
	Accel1 float32 // b_f[1]
	Accel0 float32 // b_f[0]
}

func (self *NavFilterBias) TypeID() uint8 {
	return 220
}

func (self *NavFilterBias) TypeName() string {
	return "NAV_FILTER_BIAS"
}

func (self *NavFilterBias) TypeSize() uint8 {
	return 32
}

func (self *NavFilterBias) TypeCRCExtra() uint8 {
	return 142
}

// Complete set of calibration parameters for the radio
type RadioCalibration struct {
	Throttle [5]uint16 // Throttle curve setpoints (every 25%)
	Pitch    [5]uint16 // Pitch curve setpoints (every 25%)
	Gyro     [2]uint16 // Tail gyro mode/gain setpoints: heading hold, rate mode
	Rudder   [3]uint16 // Rudder setpoints: nose left, center, nose right
	Elevator [3]uint16 // Elevator setpoints: nose down, center, nose up
	Aileron  [3]uint16 // Aileron setpoints: left, center, right
}

func (self *RadioCalibration) TypeID() uint8 {
	return 221
}

func (self *RadioCalibration) TypeName() string {
	return "RADIO_CALIBRATION"
}

func (self *RadioCalibration) TypeSize() uint8 {
	return 12
}

func (self *RadioCalibration) TypeCRCExtra() uint8 {
	return 79
}

// System status specific to ualberta uav
type UalbertaSysStatus struct {
	Pilot   uint8 // Pilot mode, see UALBERTA_PILOT_MODE
	NavMode uint8 // Navigation mode, see UALBERTA_NAV_MODE ENUM
	Mode    uint8 // System mode, see UALBERTA_AUTOPILOT_MODE ENUM
}

func (self *UalbertaSysStatus) TypeID() uint8 {
	return 222
}

func (self *UalbertaSysStatus) TypeName() string {
	return "UALBERTA_SYS_STATUS"
}

func (self *UalbertaSysStatus) TypeSize() uint8 {
	return 3
}

func (self *UalbertaSysStatus) TypeCRCExtra() uint8 {
	return 43
}

////////////////////////////////////////////////////////////////////////////////
// String Helpers
////////////////////////////////////////////////////////////////////////////////

func truncate(chars []byte) []byte {
	for i, c := range chars {
		if c == 0 {
			return chars[:i]
		}
	}
	return chars
}

type Char2 [2]byte

func (chars *Char2) String() string {
	return string(truncate(chars[:]))
}

type Char3 [3]byte

func (chars *Char3) String() string {
	return string(truncate(chars[:]))
}

type Char5 [5]byte

func (chars *Char5) String() string {
	return string(truncate(chars[:]))
}
