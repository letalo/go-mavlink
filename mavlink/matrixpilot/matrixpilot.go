package matrixpilot

import (
	"github.com/SpaceLeap/go-mavlink/mavlink"
	"github.com/SpaceLeap/go-mavlink/mavlink/common"
)

const (
	PROTOCOL_NAME    = "matrixpilot"
	PROTOCOL_VERSION = ""
	PROTOCOL_INCLUDE = common.PROTOCOL_NAME
)

func init() {
	mavlink.ProtocolName = PROTOCOL_NAME
	mavlink.ProtocolVersion = PROTOCOL_VERSION

	mavlink.MessageFactory[150] = func() mavlink.Message { return new(FlexifunctionSet) }
	mavlink.MessageFactory[151] = func() mavlink.Message { return new(FlexifunctionReadReq) }
	mavlink.MessageFactory[152] = func() mavlink.Message { return new(FlexifunctionBufferFunction) }
	mavlink.MessageFactory[153] = func() mavlink.Message { return new(FlexifunctionBufferFunctionAck) }
	mavlink.MessageFactory[155] = func() mavlink.Message { return new(FlexifunctionDirectory) }
	mavlink.MessageFactory[156] = func() mavlink.Message { return new(FlexifunctionDirectoryAck) }
	mavlink.MessageFactory[157] = func() mavlink.Message { return new(FlexifunctionCommand) }
	mavlink.MessageFactory[158] = func() mavlink.Message { return new(FlexifunctionCommandAck) }
	mavlink.MessageFactory[170] = func() mavlink.Message { return new(SerialUdbExtraF2A) }
	mavlink.MessageFactory[171] = func() mavlink.Message { return new(SerialUdbExtraF2B) }
	mavlink.MessageFactory[172] = func() mavlink.Message { return new(SerialUdbExtraF4) }
	mavlink.MessageFactory[173] = func() mavlink.Message { return new(SerialUdbExtraF5) }
	mavlink.MessageFactory[174] = func() mavlink.Message { return new(SerialUdbExtraF6) }
	mavlink.MessageFactory[175] = func() mavlink.Message { return new(SerialUdbExtraF7) }
	mavlink.MessageFactory[176] = func() mavlink.Message { return new(SerialUdbExtraF8) }
	mavlink.MessageFactory[177] = func() mavlink.Message { return new(SerialUdbExtraF13) }
	mavlink.MessageFactory[178] = func() mavlink.Message { return new(SerialUdbExtraF14) }
	mavlink.MessageFactory[179] = func() mavlink.Message { return new(SerialUdbExtraF15) }
	mavlink.MessageFactory[180] = func() mavlink.Message { return new(SerialUdbExtraF16) }
	mavlink.MessageFactory[181] = func() mavlink.Message { return new(Altitudes) }
	mavlink.MessageFactory[182] = func() mavlink.Message { return new(Airspeeds) }
}

////////////////////////////////////////////////////////////////////////////////
// Enums
////////////////////////////////////////////////////////////////////////////////

// MAV_PREFLIGHT_STORAGE_ACTION: Action required when performing CMD_PREFLIGHT_STORAGE
const (
	MAV_PFS_CMD_READ_ALL       = 0 // Read all parameters from storage
	MAV_PFS_CMD_WRITE_ALL      = 1 // Write all parameters to storage
	MAV_PFS_CMD_CLEAR_ALL      = 2 // Clear all  parameters in storage
	MAV_PFS_CMD_READ_SPECIFIC  = 3 // Read specific parameters from storage
	MAV_PFS_CMD_WRITE_SPECIFIC = 4 // Write specific parameters to storage
	MAV_PFS_CMD_CLEAR_SPECIFIC = 5 // Clear specific parameters in storage
	MAV_PFS_CMD_DO_NOTHING     = 6 // do nothing
)

// MAV_CMD:
const (
	MAV_CMD_PREFLIGHT_STORAGE_ADVANCED = 0 // Request storage of different parameter values and logs. This command will be only accepted if in pre-flight mode.
)

////////////////////////////////////////////////////////////////////////////////
// Messages
////////////////////////////////////////////////////////////////////////////////

// Depreciated but used as a compiler flag.  Do not remove
type FlexifunctionSet struct {
	TargetComponent uint8 // Component ID
	TargetSystem    uint8 // System ID
}

func (self *FlexifunctionSet) TypeID() uint8 {
	return 150
}

func (self *FlexifunctionSet) TypeName() string {
	return "FLEXIFUNCTION_SET"
}

func (self *FlexifunctionSet) TypeSize() uint8 {
	return 2
}

func (self *FlexifunctionSet) TypeCRCExtra() uint8 {
	return 70
}

// Reqest reading of flexifunction data
type FlexifunctionReadReq struct {
	DataIndex       int16 // index into data where needed
	ReadReqType     int16 // Type of flexifunction data requested
	TargetComponent uint8 // Component ID
	TargetSystem    uint8 // System ID
}

func (self *FlexifunctionReadReq) TypeID() uint8 {
	return 151
}

func (self *FlexifunctionReadReq) TypeName() string {
	return "FLEXIFUNCTION_READ_REQ"
}

func (self *FlexifunctionReadReq) TypeSize() uint8 {
	return 6
}

func (self *FlexifunctionReadReq) TypeCRCExtra() uint8 {
	return 176
}

// Flexifunction type and parameters for component at function index from buffer
type FlexifunctionBufferFunction struct {
	DataSize        uint16   // Size of the
	DataAddress     uint16   // Address in the flexifunction data, Set to 0xFFFF to use address in target memory
	FuncCount       uint16   // Total count of functions
	FuncIndex       uint16   // Function index
	Data            [48]int8 // Settings data
	TargetComponent uint8    // Component ID
	TargetSystem    uint8    // System ID
}

func (self *FlexifunctionBufferFunction) TypeID() uint8 {
	return 152
}

func (self *FlexifunctionBufferFunction) TypeName() string {
	return "FLEXIFUNCTION_BUFFER_FUNCTION"
}

func (self *FlexifunctionBufferFunction) TypeSize() uint8 {
	return 11
}

func (self *FlexifunctionBufferFunction) TypeCRCExtra() uint8 {
	return 106
}

// Flexifunction type and parameters for component at function index from buffer
type FlexifunctionBufferFunctionAck struct {
	Result          uint16 // result of acknowledge, 0=fail, 1=good
	FuncIndex       uint16 // Function index
	TargetComponent uint8  // Component ID
	TargetSystem    uint8  // System ID
}

func (self *FlexifunctionBufferFunctionAck) TypeID() uint8 {
	return 153
}

func (self *FlexifunctionBufferFunctionAck) TypeName() string {
	return "FLEXIFUNCTION_BUFFER_FUNCTION_ACK"
}

func (self *FlexifunctionBufferFunctionAck) TypeSize() uint8 {
	return 6
}

func (self *FlexifunctionBufferFunctionAck) TypeCRCExtra() uint8 {
	return 16
}

// Acknowldge sucess or failure of a flexifunction command
type FlexifunctionDirectory struct {
	DirectoryData   [48]int8 // Settings data
	Count           uint8    // count of directory entries to write
	StartIndex      uint8    // index of first directory entry to write
	DirectoryType   uint8    // 0=inputs, 1=outputs
	TargetComponent uint8    // Component ID
	TargetSystem    uint8    // System ID
}

func (self *FlexifunctionDirectory) TypeID() uint8 {
	return 155
}

func (self *FlexifunctionDirectory) TypeName() string {
	return "FLEXIFUNCTION_DIRECTORY"
}

func (self *FlexifunctionDirectory) TypeSize() uint8 {
	return 6
}

func (self *FlexifunctionDirectory) TypeCRCExtra() uint8 {
	return 18
}

// Acknowldge sucess or failure of a flexifunction command
type FlexifunctionDirectoryAck struct {
	Result          uint16 // result of acknowledge, 0=fail, 1=good
	Count           uint8  // count of directory entries to write
	StartIndex      uint8  // index of first directory entry to write
	DirectoryType   uint8  // 0=inputs, 1=outputs
	TargetComponent uint8  // Component ID
	TargetSystem    uint8  // System ID
}

func (self *FlexifunctionDirectoryAck) TypeID() uint8 {
	return 156
}

func (self *FlexifunctionDirectoryAck) TypeName() string {
	return "FLEXIFUNCTION_DIRECTORY_ACK"
}

func (self *FlexifunctionDirectoryAck) TypeSize() uint8 {
	return 7
}

func (self *FlexifunctionDirectoryAck) TypeCRCExtra() uint8 {
	return 28
}

// Acknowldge sucess or failure of a flexifunction command
type FlexifunctionCommand struct {
	CommandType     uint8 // Flexifunction command type
	TargetComponent uint8 // Component ID
	TargetSystem    uint8 // System ID
}

func (self *FlexifunctionCommand) TypeID() uint8 {
	return 157
}

func (self *FlexifunctionCommand) TypeName() string {
	return "FLEXIFUNCTION_COMMAND"
}

func (self *FlexifunctionCommand) TypeSize() uint8 {
	return 3
}

func (self *FlexifunctionCommand) TypeCRCExtra() uint8 {
	return 127
}

// Acknowldge sucess or failure of a flexifunction command
type FlexifunctionCommandAck struct {
	Result      uint16 // result of acknowledge
	CommandType uint16 // Command acknowledged
}

func (self *FlexifunctionCommandAck) TypeID() uint8 {
	return 158
}

func (self *FlexifunctionCommandAck) TypeName() string {
	return "FLEXIFUNCTION_COMMAND_ACK"
}

func (self *FlexifunctionCommandAck) TypeSize() uint8 {
	return 4
}

func (self *FlexifunctionCommandAck) TypeCRCExtra() uint8 {
	return 55
}

// Backwards compatible MAVLink version of SERIAL_UDB_EXTRA - F2: Format Part A
type SerialUdbExtraF2A struct {
	SueAltitude       int32  // Serial UDB Extra Altitude
	SueLongitude      int32  // Serial UDB Extra Longitude
	SueLatitude       int32  // Serial UDB Extra Latitude
	SueTime           uint32 // Serial UDB Extra Time
	SueHdop           int16  // Serial UDB Extra GPS Horizontal Dilution of Precision
	SueSvs            int16  // Serial UDB Extra Number of Sattelites in View
	SueMagfieldearth2 int16  // Serial UDB Extra Magnetic Field Earth 2
	SueMagfieldearth1 int16  // Serial UDB Extra Magnetic Field Earth 1
	SueMagfieldearth0 int16  // Serial UDB Extra Magnetic Field Earth 0
	SueEstimatedWind2 int16  // Serial UDB Extra Estimated Wind 2
	SueEstimatedWind1 int16  // Serial UDB Extra Estimated Wind 1
	SueEstimatedWind0 int16  // Serial UDB Extra Estimated Wind 0
	SueAirSpeed3dimu  uint16 // Serial UDB Extra 3D IMU Air Speed
	SueVoltageMilis   int16  // Serial UDB Extra Voltage in MilliVolts
	SueCpuLoad        uint16 // Serial UDB Extra CPU Load
	SueSog            int16  // Serial UDB Extra Speed Over Ground
	SueCog            uint16 // Serial UDB Extra GPS Course Over Ground
	SueRmat8          int16  // Serial UDB Extra Rmat 8
	SueRmat7          int16  // Serial UDB Extra Rmat 7
	SueRmat6          int16  // Serial UDB Extra Rmat 6
	SueRmat5          int16  // Serial UDB Extra Rmat 5
	SueRmat4          int16  // Serial UDB Extra Rmat 4
	SueRmat3          int16  // Serial UDB Extra Rmat 3
	SueRmat2          int16  // Serial UDB Extra Rmat 2
	SueRmat1          int16  // Serial UDB Extra Rmat 1
	SueRmat0          int16  // Serial UDB Extra Rmat 0
	SueWaypointIndex  uint16 // Serial UDB Extra Waypoint Index
	SueStatus         uint8  // Serial UDB Extra Status
}

func (self *SerialUdbExtraF2A) TypeID() uint8 {
	return 170
}

func (self *SerialUdbExtraF2A) TypeName() string {
	return "SERIAL_UDB_EXTRA_F2_A"
}

func (self *SerialUdbExtraF2A) TypeSize() uint8 {
	return 63
}

func (self *SerialUdbExtraF2A) TypeCRCExtra() uint8 {
	return 231
}

// Backwards compatible version of SERIAL_UDB_EXTRA - F2: Part B
type SerialUdbExtraF2B struct {
	SueFlags           uint32 // Serial UDB Extra Status Flags
	SueTime            uint32 // Serial UDB Extra Time
	SueMemoryStackFree int16  // Serial UDB Extra Stack Memory Free
	SueWaypointGoalZ   int16  // Serial UDB Extra Current Waypoint Goal Z
	SueWaypointGoalY   int16  // Serial UDB Extra Current Waypoint Goal Y
	SueWaypointGoalX   int16  // Serial UDB Extra Current Waypoint Goal X
	SueImuVelocityZ    int16  // Serial UDB Extra IMU Velocity Z
	SueImuVelocityY    int16  // Serial UDB Extra IMU Velocity Y
	SueImuVelocityX    int16  // Serial UDB Extra IMU Velocity X
	SueOscFails        int16  // Serial UDB Extra Oscillator Failure Count
	SueImuLocationZ    int16  // Serial UDB Extra IMU Location Z
	SueImuLocationY    int16  // Serial UDB Extra IMU Location Y
	SueImuLocationX    int16  // Serial UDB Extra IMU Location X
	SuePwmOutput10     int16  // Serial UDB Extra PWM Output Channel 10
	SuePwmOutput9      int16  // Serial UDB Extra PWM Output Channel 9
	SuePwmOutput8      int16  // Serial UDB Extra PWM Output Channel 8
	SuePwmOutput7      int16  // Serial UDB Extra PWM Output Channel 7
	SuePwmOutput6      int16  // Serial UDB Extra PWM Output Channel 6
	SuePwmOutput5      int16  // Serial UDB Extra PWM Output Channel 5
	SuePwmOutput4      int16  // Serial UDB Extra PWM Output Channel 4
	SuePwmOutput3      int16  // Serial UDB Extra PWM Output Channel 3
	SuePwmOutput2      int16  // Serial UDB Extra PWM Output Channel 2
	SuePwmOutput1      int16  // Serial UDB Extra PWM Output Channel 1
	SuePwmInput10      int16  // Serial UDB Extra PWM Input Channel 10
	SuePwmInput9       int16  // Serial UDB Extra PWM Input Channel 9
	SuePwmInput8       int16  // Serial UDB Extra PWM Input Channel 8
	SuePwmInput7       int16  // Serial UDB Extra PWM Input Channel 7
	SuePwmInput6       int16  // Serial UDB Extra PWM Input Channel 6
	SuePwmInput5       int16  // Serial UDB Extra PWM Input Channel 5
	SuePwmInput4       int16  // Serial UDB Extra PWM Input Channel 4
	SuePwmInput3       int16  // Serial UDB Extra PWM Input Channel 3
	SuePwmInput2       int16  // Serial UDB Extra PWM Input Channel 2
	SuePwmInput1       int16  // Serial UDB Extra PWM Input Channel 1
}

func (self *SerialUdbExtraF2B) TypeID() uint8 {
	return 171
}

func (self *SerialUdbExtraF2B) TypeName() string {
	return "SERIAL_UDB_EXTRA_F2_B"
}

func (self *SerialUdbExtraF2B) TypeSize() uint8 {
	return 70
}

func (self *SerialUdbExtraF2B) TypeCRCExtra() uint8 {
	return 201
}

// Backwards compatible version of SERIAL_UDB_EXTRA F4: format
type SerialUdbExtraF4 struct {
	SueRacingMode                uint8 // Serial UDB Extra Firmware racing mode enabled
	SueAltitudeholdWaypoint      uint8 // Serial UDB Extra Type of Alitude Hold when in Waypoint Mode
	SueAltitudeholdStabilized    uint8 // Serial UDB Extra Type of Alitude Hold when in Stabilized Mode
	SueRudderNavigation          uint8 // Serial UDB Extra Navigation with Rudder Enabled
	SueAileronNavigation         uint8 // Serial UDB Extra Navigation with Ailerons Enabled
	SueYawStabilizationAileron   uint8 // Serial UDB Extra Yaw Stabilization using Ailerons Enabled
	SueYawStabilizationRudder    uint8 // Serial UDB Extra Yaw Stabilization using Rudder Enabled
	SuePitchStabilization        uint8 // Serial UDB Extra Pitch Stabilization Enabled
	SueRollStabilizationRudder   uint8 // Serial UDB Extra Roll Stabilization with Rudder Enabled
	SueRollStabilizationAilerons uint8 // Serial UDB Extra Roll Stabilization with Ailerons Enabled
}

func (self *SerialUdbExtraF4) TypeID() uint8 {
	return 172
}

func (self *SerialUdbExtraF4) TypeName() string {
	return "SERIAL_UDB_EXTRA_F4"
}

func (self *SerialUdbExtraF4) TypeSize() uint8 {
	return 10
}

func (self *SerialUdbExtraF4) TypeCRCExtra() uint8 {
	return 248
}

// Backwards compatible version of SERIAL_UDB_EXTRA F5: format
type SerialUdbExtraF5 struct {
	SueAileronBoost            float32 // Gain For Boosting Manual Aileron control When Plane Stabilized
	SueYawStabilizationAileron float32 // YAW_STABILIZATION_AILERON Proportional control
	SueRollkd                  float32 // Serial UDB Extra ROLLKD Gain for Rate control of roll stabilization
	SueRollkp                  float32 // Serial UDB Extra ROLLKP Gain for Proportional control of roll stabilization
	SueYawkdAileron            float32 // Serial UDB YAWKD_AILERON Gain for Rate control of navigation
	SueYawkpAileron            float32 // Serial UDB YAWKP_AILERON Gain for Proporional control of navigation
}

func (self *SerialUdbExtraF5) TypeID() uint8 {
	return 173
}

func (self *SerialUdbExtraF5) TypeName() string {
	return "SERIAL_UDB_EXTRA_F5"
}

func (self *SerialUdbExtraF5) TypeSize() uint8 {
	return 24
}

func (self *SerialUdbExtraF5) TypeCRCExtra() uint8 {
	return 151
}

// Backwards compatible version of SERIAL_UDB_EXTRA F6: format
type SerialUdbExtraF6 struct {
	SueElevatorBoost float32 // Gain For Boosting Manual Elevator control When Plane Stabilized
	SueRollElevMix   float32 // Serial UDB Extra Roll to Elevator Mix
	SueRudderElevMix float32 // Serial UDB Extra Rudder to Elevator Mix
	SuePitchkd       float32 // Serial UDB Extra Pitch Rate Control
	SuePitchgain     float32 // Serial UDB Extra PITCHGAIN Proportional Control
}

func (self *SerialUdbExtraF6) TypeID() uint8 {
	return 174
}

func (self *SerialUdbExtraF6) TypeName() string {
	return "SERIAL_UDB_EXTRA_F6"
}

func (self *SerialUdbExtraF6) TypeSize() uint8 {
	return 20
}

func (self *SerialUdbExtraF6) TypeCRCExtra() uint8 {
	return 187
}

// Backwards compatible version of SERIAL_UDB_EXTRA F7: format
type SerialUdbExtraF7 struct {
	SueRtlPitchDown float32 // Serial UDB Extra Return To Landing - Angle to Pitch Plane Down
	SueRudderBoost  float32 // SERIAL UDB EXTRA Rudder Boost Gain to Manual Control when stabilized
	SueRollkdRudder float32 // Serial UDB Extra ROLLKD_RUDDER Gain for Rate control of roll stabilization
	SueRollkpRudder float32 // Serial UDB Extra ROLLKP_RUDDER Gain for Proportional control of roll stabilization
	SueYawkdRudder  float32 // Serial UDB YAWKD_RUDDER Gain for Rate control of navigation
	SueYawkpRudder  float32 // Serial UDB YAWKP_RUDDER Gain for Proporional control of navigation
}

func (self *SerialUdbExtraF7) TypeID() uint8 {
	return 175
}

func (self *SerialUdbExtraF7) TypeName() string {
	return "SERIAL_UDB_EXTRA_F7"
}

func (self *SerialUdbExtraF7) TypeSize() uint8 {
	return 24
}

func (self *SerialUdbExtraF7) TypeCRCExtra() uint8 {
	return 61
}

// Backwards compatible version of SERIAL_UDB_EXTRA F8: format
type SerialUdbExtraF8 struct {
	SueAltHoldPitchHigh   float32 // Serial UDB Extra ALT_HOLD_PITCH_HIGH
	SueAltHoldPitchMax    float32 // Serial UDB Extra ALT_HOLD_PITCH_MAX
	SueAltHoldPitchMin    float32 // Serial UDB Extra ALT_HOLD_PITCH_MIN
	SueAltHoldThrottleMax float32 // Serial UDB Extra ALT_HOLD_THROTTLE_MAX
	SueAltHoldThrottleMin float32 // Serial UDB Extra ALT_HOLD_THROTTLE_MIN
	SueHeightTargetMin    float32 // Serial UDB Extra HEIGHT_TARGET_MIN
	SueHeightTargetMax    float32 // Serial UDB Extra HEIGHT_TARGET_MAX
}

func (self *SerialUdbExtraF8) TypeID() uint8 {
	return 176
}

func (self *SerialUdbExtraF8) TypeName() string {
	return "SERIAL_UDB_EXTRA_F8"
}

func (self *SerialUdbExtraF8) TypeSize() uint8 {
	return 28
}

func (self *SerialUdbExtraF8) TypeCRCExtra() uint8 {
	return 204
}

// Backwards compatible version of SERIAL_UDB_EXTRA F13: format
type SerialUdbExtraF13 struct {
	SueAltOrigin int32 // Serial UDB Extra MP Origin Altitude Above Sea Level
	SueLonOrigin int32 // Serial UDB Extra MP Origin Longitude
	SueLatOrigin int32 // Serial UDB Extra MP Origin Latitude
	SueWeekNo    int16 // Serial UDB Extra GPS Week Number
}

func (self *SerialUdbExtraF13) TypeID() uint8 {
	return 177
}

func (self *SerialUdbExtraF13) TypeName() string {
	return "SERIAL_UDB_EXTRA_F13"
}

func (self *SerialUdbExtraF13) TypeSize() uint8 {
	return 14
}

func (self *SerialUdbExtraF13) TypeCRCExtra() uint8 {
	return 15
}

// Backwards compatible version of SERIAL_UDB_EXTRA F14: format
type SerialUdbExtraF14 struct {
	SueTrapSource     uint32 // Serial UDB Extra Type Program Address of Last Trap
	SueOscFailCount   int16  // Serial UDB Extra Number of Ocillator Failures
	SueTrapFlags      int16  // Serial UDB Extra  Last dspic Trap Flags
	SueRcon           int16  // Serial UDB Extra Reboot Regitster of DSPIC
	SueFlightPlanType uint8  // Serial UDB Extra Type of Flight Plan
	SueClockConfig    uint8  // Serial UDB Extra UDB Internal Clock Configuration
	SueAirframe       uint8  // Serial UDB Extra Type of Airframe
	SueBoardType      uint8  // Serial UDB Extra Type of UDB Hardware
	SueDr             uint8  // Serial UDB Extra Dead Reckoning Enabled
	SueGpsType        uint8  // Serial UDB Extra Type of GPS Unit
	SueWindEstimation uint8  // Serial UDB Extra Wind Estimation Enabled
}

func (self *SerialUdbExtraF14) TypeID() uint8 {
	return 178
}

func (self *SerialUdbExtraF14) TypeName() string {
	return "SERIAL_UDB_EXTRA_F14"
}

func (self *SerialUdbExtraF14) TypeSize() uint8 {
	return 17
}

func (self *SerialUdbExtraF14) TypeCRCExtra() uint8 {
	return 202
}

// Backwards compatible version of SERIAL_UDB_EXTRA F15 and F16: format
type SerialUdbExtraF15 struct {
	SueIdVehicleRegistration [20]uint8 // Serial UDB Extra Registraton Number of Vehicle
	SueIdVehicleModelName    [40]uint8 // Serial UDB Extra Model Name Of Vehicle
}

func (self *SerialUdbExtraF15) TypeID() uint8 {
	return 179
}

func (self *SerialUdbExtraF15) TypeName() string {
	return "SERIAL_UDB_EXTRA_F15"
}

func (self *SerialUdbExtraF15) TypeSize() uint8 {
	return 2
}

func (self *SerialUdbExtraF15) TypeCRCExtra() uint8 {
	return 161
}

//
type SerialUdbExtraF16 struct {
	SueIdDiyDronesUrl [70]uint8 // Serial UDB Extra URL of Lead Pilot or Team
	SueIdLeadPilot    [40]uint8 // Serial UDB Extra Name of Expected Lead Pilot
}

func (self *SerialUdbExtraF16) TypeID() uint8 {
	return 180
}

func (self *SerialUdbExtraF16) TypeName() string {
	return "SERIAL_UDB_EXTRA_F16"
}

func (self *SerialUdbExtraF16) TypeSize() uint8 {
	return 2
}

func (self *SerialUdbExtraF16) TypeCRCExtra() uint8 {
	return 85
}

// The altitude measured by sensors and IMU
type Altitudes struct {
	AltExtra       int32  // Extra altitude above ground in meters, expressed as * 1000 (millimeters)
	AltRangeFinder int32  // Rangefinder Altitude above ground in meters, expressed as * 1000 (millimeters)
	AltOpticalFlow int32  // Optical flow altitude above ground in meters, expressed as * 1000 (millimeters)
	AltBarometric  int32  // barometeric altitude above ground in meters, expressed as * 1000 (millimeters)
	AltImu         int32  // IMU altitude above ground in meters, expressed as * 1000 (millimeters)
	AltGps         int32  // GPS altitude in meters, expressed as * 1000 (millimeters), above MSL
	TimeBootMs     uint32 // Timestamp (milliseconds since system boot)
}

func (self *Altitudes) TypeID() uint8 {
	return 181
}

func (self *Altitudes) TypeName() string {
	return "ALTITUDES"
}

func (self *Altitudes) TypeSize() uint8 {
	return 28
}

func (self *Altitudes) TypeCRCExtra() uint8 {
	return 81
}

// The airspeed measured by sensors and IMU
type Airspeeds struct {
	TimeBootMs         uint32 // Timestamp (milliseconds since system boot)
	Aoy                int16  // Yaw angle sensor, degrees * 10
	Aoa                int16  // Angle of attack sensor, degrees * 10
	AirspeedUltrasonic int16  // Ultrasonic measured airspeed, cm/s
	AirspeedHotWire    int16  // Hot wire anenometer measured airspeed, cm/s
	AirspeedPitot      int16  // Pitot measured forward airpseed, cm/s
	AirspeedImu        int16  // Airspeed estimate from IMU, cm/s
}

func (self *Airspeeds) TypeID() uint8 {
	return 182
}

func (self *Airspeeds) TypeName() string {
	return "AIRSPEEDS"
}

func (self *Airspeeds) TypeSize() uint8 {
	return 16
}

func (self *Airspeeds) TypeCRCExtra() uint8 {
	return 240
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

type Char20 [20]byte

func (chars *Char20) String() string {
	return string(truncate(chars[:]))
}

type Char40 [40]byte

func (chars *Char40) String() string {
	return string(truncate(chars[:]))
}

type Char48 [48]byte

func (chars *Char48) String() string {
	return string(truncate(chars[:]))
}

type Char70 [70]byte

func (chars *Char70) String() string {
	return string(truncate(chars[:]))
}
