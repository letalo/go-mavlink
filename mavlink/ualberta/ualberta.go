package ualberta

import (
	"fmt"

	"github.com/SpaceLeap/go-mavlink/mavlink"
	"github.com/SpaceLeap/go-mavlink/mavlink/common"
)

const (
	PROTOCOL_NAME    = "ualberta"
	PROTOCOL_VERSION = 0
	PROTOCOL_INCLUDE = common.PROTOCOL_NAME
)

func Init() {
	common.Init()

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
	MODE_MANUAL_SCALED = 1 // Inputs are normalized using calibration, the converted back to raw pulse widths for output
	MODE_AUTO_PID_ATT  = 2 //  dfsdfs
	MODE_AUTO_PID_VEL  = 3 //  dfsfds
	MODE_AUTO_PID_POS  = 4 //  dfsdfsdfs
)

// UALBERTA_NAV_MODE: Navigation filter mode
const (
	NAV_AHRS_INIT    = 0 //
	NAV_AHRS         = 1 // AHRS mode
	NAV_INS_GPS_INIT = 2 // INS/GPS initialization mode
	NAV_INS_GPS      = 3 // INS/GPS mode
)

// UALBERTA_PILOT_MODE: Mode currently commanded by pilot
const (
	PILOT_MANUAL = 0 //  sdf
	PILOT_AUTO   = 1 //  dfs
	PILOT_ROTO   = 2 //  Rotomotion mode
)

////////////////////////////////////////////////////////////////////////////////
// Messages
////////////////////////////////////////////////////////////////////////////////

// Accelerometer and Gyro biases from the navigation filter
type NavFilterBias struct {
	Usec   uint64  // Timestamp (microseconds)
	Accel0 float32 // b_f[0]
	Accel1 float32 // b_f[1]
	Accel2 float32 // b_f[2]
	Gyro0  float32 // b_f[0]
	Gyro1  float32 // b_f[1]
	Gyro2  float32 // b_f[2]
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
	return 34
}

func (self *NavFilterBias) FieldsString() string {
	return fmt.Sprintf("Usec=%d Accel0=%d Accel1=%d Accel2=%d Gyro0=%d Gyro1=%d Gyro2=%d", self.Usec, self.Accel0, self.Accel1, self.Accel2, self.Gyro0, self.Gyro1, self.Gyro2)
}

func (self *NavFilterBias) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Complete set of calibration parameters for the radio
type RadioCalibration struct {
	Aileron  [3]uint16 // Aileron setpoints: left, center, right
	Elevator [3]uint16 // Elevator setpoints: nose down, center, nose up
	Rudder   [3]uint16 // Rudder setpoints: nose left, center, nose right
	Gyro     [2]uint16 // Tail gyro mode/gain setpoints: heading hold, rate mode
	Pitch    [5]uint16 // Pitch curve setpoints (every 25%)
	Throttle [5]uint16 // Throttle curve setpoints (every 25%)
}

func (self *RadioCalibration) TypeID() uint8 {
	return 221
}

func (self *RadioCalibration) TypeName() string {
	return "RADIO_CALIBRATION"
}

func (self *RadioCalibration) TypeSize() uint8 {
	return 42
}

func (self *RadioCalibration) TypeCRCExtra() uint8 {
	return 230
}

func (self *RadioCalibration) FieldsString() string {
	return fmt.Sprintf("Aileron=%v Elevator=%v Rudder=%v Gyro=%v Pitch=%v Throttle=%v", self.Aileron, self.Elevator, self.Rudder, self.Gyro, self.Pitch, self.Throttle)
}

func (self *RadioCalibration) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// System status specific to ualberta uav
type UalbertaSysStatus struct {
	Mode    uint8 // System mode, see UALBERTA_AUTOPILOT_MODE ENUM
	NavMode uint8 // Navigation mode, see UALBERTA_NAV_MODE ENUM
	Pilot   uint8 // Pilot mode, see UALBERTA_PILOT_MODE
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
	return 15
}

func (self *UalbertaSysStatus) FieldsString() string {
	return fmt.Sprintf("Mode=%d NavMode=%d Pilot=%d", self.Mode, self.NavMode, self.Pilot)
}

func (self *UalbertaSysStatus) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

////////////////////////////////////////////////////////////////////////////////
// String Helpers
////////////////////////////////////////////////////////////////////////////////

type Char2 [2]byte

func (chars *Char2) String() string {
	return mavlink.FixString(chars[:])
}

type Char3 [3]byte

func (chars *Char3) String() string {
	return mavlink.FixString(chars[:])
}

type Char5 [5]byte

func (chars *Char5) String() string {
	return mavlink.FixString(chars[:])
}
