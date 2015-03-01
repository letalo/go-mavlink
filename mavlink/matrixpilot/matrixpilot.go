package matrixpilot

import (
	"fmt"

	"github.com/SpaceLeap/go-mavlink/mavlink"
	"github.com/SpaceLeap/go-mavlink/mavlink/common"
)

const (
	PROTOCOL_NAME = "matrixpilot"

	PROTOCOL_INCLUDE = common.PROTOCOL_NAME
)

// Init initializes mavlink.ProtocolName, mavlink.ProtocolVersion, and mavlink.MessageFactory.
func Init() {
	common.Init()

	mavlink.ProtocolName = PROTOCOL_NAME

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

// MessageNameIDMap returns a map from message name to message ID.
func MessageNameIDMap() map[string]int {
	return map[string]int{
		"FLEXIFUNCTION_SET":                 150,
		"FLEXIFUNCTION_READ_REQ":            151,
		"FLEXIFUNCTION_BUFFER_FUNCTION":     152,
		"FLEXIFUNCTION_BUFFER_FUNCTION_ACK": 153,
		"FLEXIFUNCTION_DIRECTORY":           155,
		"FLEXIFUNCTION_DIRECTORY_ACK":       156,
		"FLEXIFUNCTION_COMMAND":             157,
		"FLEXIFUNCTION_COMMAND_ACK":         158,
		"SERIAL_UDB_EXTRA_F2_A":             170,
		"SERIAL_UDB_EXTRA_F2_B":             171,
		"SERIAL_UDB_EXTRA_F4":               172,
		"SERIAL_UDB_EXTRA_F5":               173,
		"SERIAL_UDB_EXTRA_F6":               174,
		"SERIAL_UDB_EXTRA_F7":               175,
		"SERIAL_UDB_EXTRA_F8":               176,
		"SERIAL_UDB_EXTRA_F13":              177,
		"SERIAL_UDB_EXTRA_F14":              178,
		"SERIAL_UDB_EXTRA_F15":              179,
		"SERIAL_UDB_EXTRA_F16":              180,
		"ALTITUDES":                         181,
		"AIRSPEEDS":                         182,
	}
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
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
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
	return 181
}

func (self *FlexifunctionSet) FieldsString() string {
	return fmt.Sprintf("TargetSystem=%d TargetComponent=%d", self.TargetSystem, self.TargetComponent)
}

func (self *FlexifunctionSet) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Reqest reading of flexifunction data
type FlexifunctionReadReq struct {
	ReadReqType     int16 // Type of flexifunction data requested
	DataIndex       int16 // index into data where needed
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
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
	return 26
}

func (self *FlexifunctionReadReq) FieldsString() string {
	return fmt.Sprintf("ReadReqType=%d DataIndex=%d TargetSystem=%d TargetComponent=%d", self.ReadReqType, self.DataIndex, self.TargetSystem, self.TargetComponent)
}

func (self *FlexifunctionReadReq) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Flexifunction type and parameters for component at function index from buffer
type FlexifunctionBufferFunction struct {
	FuncIndex       uint16   // Function index
	FuncCount       uint16   // Total count of functions
	DataAddress     uint16   // Address in the flexifunction data, Set to 0xFFFF to use address in target memory
	DataSize        uint16   // Size of the
	TargetSystem    uint8    // System ID
	TargetComponent uint8    // Component ID
	Data            [48]int8 // Settings data
}

func (self *FlexifunctionBufferFunction) TypeID() uint8 {
	return 152
}

func (self *FlexifunctionBufferFunction) TypeName() string {
	return "FLEXIFUNCTION_BUFFER_FUNCTION"
}

func (self *FlexifunctionBufferFunction) TypeSize() uint8 {
	return 58
}

func (self *FlexifunctionBufferFunction) TypeCRCExtra() uint8 {
	return 105
}

func (self *FlexifunctionBufferFunction) FieldsString() string {
	return fmt.Sprintf("FuncIndex=%d FuncCount=%d DataAddress=%d DataSize=%d TargetSystem=%d TargetComponent=%d Data=%v", self.FuncIndex, self.FuncCount, self.DataAddress, self.DataSize, self.TargetSystem, self.TargetComponent, self.Data)
}

func (self *FlexifunctionBufferFunction) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Flexifunction type and parameters for component at function index from buffer
type FlexifunctionBufferFunctionAck struct {
	FuncIndex       uint16 // Function index
	Result          uint16 // result of acknowledge, 0=fail, 1=good
	TargetSystem    uint8  // System ID
	TargetComponent uint8  // Component ID
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
	return 109
}

func (self *FlexifunctionBufferFunctionAck) FieldsString() string {
	return fmt.Sprintf("FuncIndex=%d Result=%d TargetSystem=%d TargetComponent=%d", self.FuncIndex, self.Result, self.TargetSystem, self.TargetComponent)
}

func (self *FlexifunctionBufferFunctionAck) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Acknowldge sucess or failure of a flexifunction command
type FlexifunctionDirectory struct {
	TargetSystem    uint8    // System ID
	TargetComponent uint8    // Component ID
	DirectoryType   uint8    // 0=inputs, 1=outputs
	StartIndex      uint8    // index of first directory entry to write
	Count           uint8    // count of directory entries to write
	DirectoryData   [48]int8 // Settings data
}

func (self *FlexifunctionDirectory) TypeID() uint8 {
	return 155
}

func (self *FlexifunctionDirectory) TypeName() string {
	return "FLEXIFUNCTION_DIRECTORY"
}

func (self *FlexifunctionDirectory) TypeSize() uint8 {
	return 53
}

func (self *FlexifunctionDirectory) TypeCRCExtra() uint8 {
	return 163
}

func (self *FlexifunctionDirectory) FieldsString() string {
	return fmt.Sprintf("TargetSystem=%d TargetComponent=%d DirectoryType=%d StartIndex=%d Count=%d DirectoryData=%v", self.TargetSystem, self.TargetComponent, self.DirectoryType, self.StartIndex, self.Count, self.DirectoryData)
}

func (self *FlexifunctionDirectory) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Acknowldge sucess or failure of a flexifunction command
type FlexifunctionDirectoryAck struct {
	Result          uint16 // result of acknowledge, 0=fail, 1=good
	TargetSystem    uint8  // System ID
	TargetComponent uint8  // Component ID
	DirectoryType   uint8  // 0=inputs, 1=outputs
	StartIndex      uint8  // index of first directory entry to write
	Count           uint8  // count of directory entries to write
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
	return 218
}

func (self *FlexifunctionDirectoryAck) FieldsString() string {
	return fmt.Sprintf("Result=%d TargetSystem=%d TargetComponent=%d DirectoryType=%d StartIndex=%d Count=%d", self.Result, self.TargetSystem, self.TargetComponent, self.DirectoryType, self.StartIndex, self.Count)
}

func (self *FlexifunctionDirectoryAck) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Acknowldge sucess or failure of a flexifunction command
type FlexifunctionCommand struct {
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
	CommandType     uint8 // Flexifunction command type
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
	return 133
}

func (self *FlexifunctionCommand) FieldsString() string {
	return fmt.Sprintf("TargetSystem=%d TargetComponent=%d CommandType=%d", self.TargetSystem, self.TargetComponent, self.CommandType)
}

func (self *FlexifunctionCommand) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Acknowldge sucess or failure of a flexifunction command
type FlexifunctionCommandAck struct {
	CommandType uint16 // Command acknowledged
	Result      uint16 // result of acknowledge
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
	return 208
}

func (self *FlexifunctionCommandAck) FieldsString() string {
	return fmt.Sprintf("CommandType=%d Result=%d", self.CommandType, self.Result)
}

func (self *FlexifunctionCommandAck) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Backwards compatible MAVLink version of SERIAL_UDB_EXTRA - F2: Format Part A
type SerialUdbExtraF2A struct {
	SueTime           uint32 // Serial UDB Extra Time
	SueLatitude       int32  // Serial UDB Extra Latitude
	SueLongitude      int32  // Serial UDB Extra Longitude
	SueAltitude       int32  // Serial UDB Extra Altitude
	SueWaypointIndex  uint16 // Serial UDB Extra Waypoint Index
	SueRmat0          int16  // Serial UDB Extra Rmat 0
	SueRmat1          int16  // Serial UDB Extra Rmat 1
	SueRmat2          int16  // Serial UDB Extra Rmat 2
	SueRmat3          int16  // Serial UDB Extra Rmat 3
	SueRmat4          int16  // Serial UDB Extra Rmat 4
	SueRmat5          int16  // Serial UDB Extra Rmat 5
	SueRmat6          int16  // Serial UDB Extra Rmat 6
	SueRmat7          int16  // Serial UDB Extra Rmat 7
	SueRmat8          int16  // Serial UDB Extra Rmat 8
	SueCog            uint16 // Serial UDB Extra GPS Course Over Ground
	SueSog            int16  // Serial UDB Extra Speed Over Ground
	SueCpuLoad        uint16 // Serial UDB Extra CPU Load
	SueVoltageMilis   int16  // Serial UDB Extra Voltage in MilliVolts
	SueAirSpeed3dimu  uint16 // Serial UDB Extra 3D IMU Air Speed
	SueEstimatedWind0 int16  // Serial UDB Extra Estimated Wind 0
	SueEstimatedWind1 int16  // Serial UDB Extra Estimated Wind 1
	SueEstimatedWind2 int16  // Serial UDB Extra Estimated Wind 2
	SueMagfieldearth0 int16  // Serial UDB Extra Magnetic Field Earth 0
	SueMagfieldearth1 int16  // Serial UDB Extra Magnetic Field Earth 1
	SueMagfieldearth2 int16  // Serial UDB Extra Magnetic Field Earth 2
	SueSvs            int16  // Serial UDB Extra Number of Sattelites in View
	SueHdop           int16  // Serial UDB Extra GPS Horizontal Dilution of Precision
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
	return 150
}

func (self *SerialUdbExtraF2A) FieldsString() string {
	return fmt.Sprintf("SueTime=%d SueLatitude=%d SueLongitude=%d SueAltitude=%d SueWaypointIndex=%d SueRmat0=%d SueRmat1=%d SueRmat2=%d SueRmat3=%d SueRmat4=%d SueRmat5=%d SueRmat6=%d SueRmat7=%d SueRmat8=%d SueCog=%d SueSog=%d SueCpuLoad=%d SueVoltageMilis=%d SueAirSpeed3dimu=%d SueEstimatedWind0=%d SueEstimatedWind1=%d SueEstimatedWind2=%d SueMagfieldearth0=%d SueMagfieldearth1=%d SueMagfieldearth2=%d SueSvs=%d SueHdop=%d SueStatus=%d", self.SueTime, self.SueLatitude, self.SueLongitude, self.SueAltitude, self.SueWaypointIndex, self.SueRmat0, self.SueRmat1, self.SueRmat2, self.SueRmat3, self.SueRmat4, self.SueRmat5, self.SueRmat6, self.SueRmat7, self.SueRmat8, self.SueCog, self.SueSog, self.SueCpuLoad, self.SueVoltageMilis, self.SueAirSpeed3dimu, self.SueEstimatedWind0, self.SueEstimatedWind1, self.SueEstimatedWind2, self.SueMagfieldearth0, self.SueMagfieldearth1, self.SueMagfieldearth2, self.SueSvs, self.SueHdop, self.SueStatus)
}

func (self *SerialUdbExtraF2A) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Backwards compatible version of SERIAL_UDB_EXTRA - F2: Part B
type SerialUdbExtraF2B struct {
	SueTime            uint32 // Serial UDB Extra Time
	SueFlags           uint32 // Serial UDB Extra Status Flags
	SuePwmInput1       int16  // Serial UDB Extra PWM Input Channel 1
	SuePwmInput2       int16  // Serial UDB Extra PWM Input Channel 2
	SuePwmInput3       int16  // Serial UDB Extra PWM Input Channel 3
	SuePwmInput4       int16  // Serial UDB Extra PWM Input Channel 4
	SuePwmInput5       int16  // Serial UDB Extra PWM Input Channel 5
	SuePwmInput6       int16  // Serial UDB Extra PWM Input Channel 6
	SuePwmInput7       int16  // Serial UDB Extra PWM Input Channel 7
	SuePwmInput8       int16  // Serial UDB Extra PWM Input Channel 8
	SuePwmInput9       int16  // Serial UDB Extra PWM Input Channel 9
	SuePwmInput10      int16  // Serial UDB Extra PWM Input Channel 10
	SuePwmOutput1      int16  // Serial UDB Extra PWM Output Channel 1
	SuePwmOutput2      int16  // Serial UDB Extra PWM Output Channel 2
	SuePwmOutput3      int16  // Serial UDB Extra PWM Output Channel 3
	SuePwmOutput4      int16  // Serial UDB Extra PWM Output Channel 4
	SuePwmOutput5      int16  // Serial UDB Extra PWM Output Channel 5
	SuePwmOutput6      int16  // Serial UDB Extra PWM Output Channel 6
	SuePwmOutput7      int16  // Serial UDB Extra PWM Output Channel 7
	SuePwmOutput8      int16  // Serial UDB Extra PWM Output Channel 8
	SuePwmOutput9      int16  // Serial UDB Extra PWM Output Channel 9
	SuePwmOutput10     int16  // Serial UDB Extra PWM Output Channel 10
	SueImuLocationX    int16  // Serial UDB Extra IMU Location X
	SueImuLocationY    int16  // Serial UDB Extra IMU Location Y
	SueImuLocationZ    int16  // Serial UDB Extra IMU Location Z
	SueOscFails        int16  // Serial UDB Extra Oscillator Failure Count
	SueImuVelocityX    int16  // Serial UDB Extra IMU Velocity X
	SueImuVelocityY    int16  // Serial UDB Extra IMU Velocity Y
	SueImuVelocityZ    int16  // Serial UDB Extra IMU Velocity Z
	SueWaypointGoalX   int16  // Serial UDB Extra Current Waypoint Goal X
	SueWaypointGoalY   int16  // Serial UDB Extra Current Waypoint Goal Y
	SueWaypointGoalZ   int16  // Serial UDB Extra Current Waypoint Goal Z
	SueMemoryStackFree int16  // Serial UDB Extra Stack Memory Free
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
	return 169
}

func (self *SerialUdbExtraF2B) FieldsString() string {
	return fmt.Sprintf("SueTime=%d SueFlags=%d SuePwmInput1=%d SuePwmInput2=%d SuePwmInput3=%d SuePwmInput4=%d SuePwmInput5=%d SuePwmInput6=%d SuePwmInput7=%d SuePwmInput8=%d SuePwmInput9=%d SuePwmInput10=%d SuePwmOutput1=%d SuePwmOutput2=%d SuePwmOutput3=%d SuePwmOutput4=%d SuePwmOutput5=%d SuePwmOutput6=%d SuePwmOutput7=%d SuePwmOutput8=%d SuePwmOutput9=%d SuePwmOutput10=%d SueImuLocationX=%d SueImuLocationY=%d SueImuLocationZ=%d SueOscFails=%d SueImuVelocityX=%d SueImuVelocityY=%d SueImuVelocityZ=%d SueWaypointGoalX=%d SueWaypointGoalY=%d SueWaypointGoalZ=%d SueMemoryStackFree=%d", self.SueTime, self.SueFlags, self.SuePwmInput1, self.SuePwmInput2, self.SuePwmInput3, self.SuePwmInput4, self.SuePwmInput5, self.SuePwmInput6, self.SuePwmInput7, self.SuePwmInput8, self.SuePwmInput9, self.SuePwmInput10, self.SuePwmOutput1, self.SuePwmOutput2, self.SuePwmOutput3, self.SuePwmOutput4, self.SuePwmOutput5, self.SuePwmOutput6, self.SuePwmOutput7, self.SuePwmOutput8, self.SuePwmOutput9, self.SuePwmOutput10, self.SueImuLocationX, self.SueImuLocationY, self.SueImuLocationZ, self.SueOscFails, self.SueImuVelocityX, self.SueImuVelocityY, self.SueImuVelocityZ, self.SueWaypointGoalX, self.SueWaypointGoalY, self.SueWaypointGoalZ, self.SueMemoryStackFree)
}

func (self *SerialUdbExtraF2B) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Backwards compatible version of SERIAL_UDB_EXTRA F4: format
type SerialUdbExtraF4 struct {
	SueRollStabilizationAilerons uint8 // Serial UDB Extra Roll Stabilization with Ailerons Enabled
	SueRollStabilizationRudder   uint8 // Serial UDB Extra Roll Stabilization with Rudder Enabled
	SuePitchStabilization        uint8 // Serial UDB Extra Pitch Stabilization Enabled
	SueYawStabilizationRudder    uint8 // Serial UDB Extra Yaw Stabilization using Rudder Enabled
	SueYawStabilizationAileron   uint8 // Serial UDB Extra Yaw Stabilization using Ailerons Enabled
	SueAileronNavigation         uint8 // Serial UDB Extra Navigation with Ailerons Enabled
	SueRudderNavigation          uint8 // Serial UDB Extra Navigation with Rudder Enabled
	SueAltitudeholdStabilized    uint8 // Serial UDB Extra Type of Alitude Hold when in Stabilized Mode
	SueAltitudeholdWaypoint      uint8 // Serial UDB Extra Type of Alitude Hold when in Waypoint Mode
	SueRacingMode                uint8 // Serial UDB Extra Firmware racing mode enabled
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
	return 191
}

func (self *SerialUdbExtraF4) FieldsString() string {
	return fmt.Sprintf("SueRollStabilizationAilerons=%d SueRollStabilizationRudder=%d SuePitchStabilization=%d SueYawStabilizationRudder=%d SueYawStabilizationAileron=%d SueAileronNavigation=%d SueRudderNavigation=%d SueAltitudeholdStabilized=%d SueAltitudeholdWaypoint=%d SueRacingMode=%d", self.SueRollStabilizationAilerons, self.SueRollStabilizationRudder, self.SuePitchStabilization, self.SueYawStabilizationRudder, self.SueYawStabilizationAileron, self.SueAileronNavigation, self.SueRudderNavigation, self.SueAltitudeholdStabilized, self.SueAltitudeholdWaypoint, self.SueRacingMode)
}

func (self *SerialUdbExtraF4) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Backwards compatible version of SERIAL_UDB_EXTRA F5: format
type SerialUdbExtraF5 struct {
	SueYawkpAileron            float32 // Serial UDB YAWKP_AILERON Gain for Proporional control of navigation
	SueYawkdAileron            float32 // Serial UDB YAWKD_AILERON Gain for Rate control of navigation
	SueRollkp                  float32 // Serial UDB Extra ROLLKP Gain for Proportional control of roll stabilization
	SueRollkd                  float32 // Serial UDB Extra ROLLKD Gain for Rate control of roll stabilization
	SueYawStabilizationAileron float32 // YAW_STABILIZATION_AILERON Proportional control
	SueAileronBoost            float32 // Gain For Boosting Manual Aileron control When Plane Stabilized
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
	return 121
}

func (self *SerialUdbExtraF5) FieldsString() string {
	return fmt.Sprintf("SueYawkpAileron=%f SueYawkdAileron=%f SueRollkp=%f SueRollkd=%f SueYawStabilizationAileron=%f SueAileronBoost=%f", self.SueYawkpAileron, self.SueYawkdAileron, self.SueRollkp, self.SueRollkd, self.SueYawStabilizationAileron, self.SueAileronBoost)
}

func (self *SerialUdbExtraF5) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Backwards compatible version of SERIAL_UDB_EXTRA F6: format
type SerialUdbExtraF6 struct {
	SuePitchgain     float32 // Serial UDB Extra PITCHGAIN Proportional Control
	SuePitchkd       float32 // Serial UDB Extra Pitch Rate Control
	SueRudderElevMix float32 // Serial UDB Extra Rudder to Elevator Mix
	SueRollElevMix   float32 // Serial UDB Extra Roll to Elevator Mix
	SueElevatorBoost float32 // Gain For Boosting Manual Elevator control When Plane Stabilized
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
	return 54
}

func (self *SerialUdbExtraF6) FieldsString() string {
	return fmt.Sprintf("SuePitchgain=%f SuePitchkd=%f SueRudderElevMix=%f SueRollElevMix=%f SueElevatorBoost=%f", self.SuePitchgain, self.SuePitchkd, self.SueRudderElevMix, self.SueRollElevMix, self.SueElevatorBoost)
}

func (self *SerialUdbExtraF6) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Backwards compatible version of SERIAL_UDB_EXTRA F7: format
type SerialUdbExtraF7 struct {
	SueYawkpRudder  float32 // Serial UDB YAWKP_RUDDER Gain for Proporional control of navigation
	SueYawkdRudder  float32 // Serial UDB YAWKD_RUDDER Gain for Rate control of navigation
	SueRollkpRudder float32 // Serial UDB Extra ROLLKP_RUDDER Gain for Proportional control of roll stabilization
	SueRollkdRudder float32 // Serial UDB Extra ROLLKD_RUDDER Gain for Rate control of roll stabilization
	SueRudderBoost  float32 // SERIAL UDB EXTRA Rudder Boost Gain to Manual Control when stabilized
	SueRtlPitchDown float32 // Serial UDB Extra Return To Landing - Angle to Pitch Plane Down
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
	return 171
}

func (self *SerialUdbExtraF7) FieldsString() string {
	return fmt.Sprintf("SueYawkpRudder=%f SueYawkdRudder=%f SueRollkpRudder=%f SueRollkdRudder=%f SueRudderBoost=%f SueRtlPitchDown=%f", self.SueYawkpRudder, self.SueYawkdRudder, self.SueRollkpRudder, self.SueRollkdRudder, self.SueRudderBoost, self.SueRtlPitchDown)
}

func (self *SerialUdbExtraF7) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Backwards compatible version of SERIAL_UDB_EXTRA F8: format
type SerialUdbExtraF8 struct {
	SueHeightTargetMax    float32 // Serial UDB Extra HEIGHT_TARGET_MAX
	SueHeightTargetMin    float32 // Serial UDB Extra HEIGHT_TARGET_MIN
	SueAltHoldThrottleMin float32 // Serial UDB Extra ALT_HOLD_THROTTLE_MIN
	SueAltHoldThrottleMax float32 // Serial UDB Extra ALT_HOLD_THROTTLE_MAX
	SueAltHoldPitchMin    float32 // Serial UDB Extra ALT_HOLD_PITCH_MIN
	SueAltHoldPitchMax    float32 // Serial UDB Extra ALT_HOLD_PITCH_MAX
	SueAltHoldPitchHigh   float32 // Serial UDB Extra ALT_HOLD_PITCH_HIGH
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
	return 142
}

func (self *SerialUdbExtraF8) FieldsString() string {
	return fmt.Sprintf("SueHeightTargetMax=%f SueHeightTargetMin=%f SueAltHoldThrottleMin=%f SueAltHoldThrottleMax=%f SueAltHoldPitchMin=%f SueAltHoldPitchMax=%f SueAltHoldPitchHigh=%f", self.SueHeightTargetMax, self.SueHeightTargetMin, self.SueAltHoldThrottleMin, self.SueAltHoldThrottleMax, self.SueAltHoldPitchMin, self.SueAltHoldPitchMax, self.SueAltHoldPitchHigh)
}

func (self *SerialUdbExtraF8) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Backwards compatible version of SERIAL_UDB_EXTRA F13: format
type SerialUdbExtraF13 struct {
	SueLatOrigin int32 // Serial UDB Extra MP Origin Latitude
	SueLonOrigin int32 // Serial UDB Extra MP Origin Longitude
	SueAltOrigin int32 // Serial UDB Extra MP Origin Altitude Above Sea Level
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
	return 249
}

func (self *SerialUdbExtraF13) FieldsString() string {
	return fmt.Sprintf("SueLatOrigin=%d SueLonOrigin=%d SueAltOrigin=%d SueWeekNo=%d", self.SueLatOrigin, self.SueLonOrigin, self.SueAltOrigin, self.SueWeekNo)
}

func (self *SerialUdbExtraF13) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Backwards compatible version of SERIAL_UDB_EXTRA F14: format
type SerialUdbExtraF14 struct {
	SueTrapSource     uint32 // Serial UDB Extra Type Program Address of Last Trap
	SueRcon           int16  // Serial UDB Extra Reboot Regitster of DSPIC
	SueTrapFlags      int16  // Serial UDB Extra  Last dspic Trap Flags
	SueOscFailCount   int16  // Serial UDB Extra Number of Ocillator Failures
	SueWindEstimation uint8  // Serial UDB Extra Wind Estimation Enabled
	SueGpsType        uint8  // Serial UDB Extra Type of GPS Unit
	SueDr             uint8  // Serial UDB Extra Dead Reckoning Enabled
	SueBoardType      uint8  // Serial UDB Extra Type of UDB Hardware
	SueAirframe       uint8  // Serial UDB Extra Type of Airframe
	SueClockConfig    uint8  // Serial UDB Extra UDB Internal Clock Configuration
	SueFlightPlanType uint8  // Serial UDB Extra Type of Flight Plan
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
	return 123
}

func (self *SerialUdbExtraF14) FieldsString() string {
	return fmt.Sprintf("SueTrapSource=%d SueRcon=%d SueTrapFlags=%d SueOscFailCount=%d SueWindEstimation=%d SueGpsType=%d SueDr=%d SueBoardType=%d SueAirframe=%d SueClockConfig=%d SueFlightPlanType=%d", self.SueTrapSource, self.SueRcon, self.SueTrapFlags, self.SueOscFailCount, self.SueWindEstimation, self.SueGpsType, self.SueDr, self.SueBoardType, self.SueAirframe, self.SueClockConfig, self.SueFlightPlanType)
}

func (self *SerialUdbExtraF14) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Backwards compatible version of SERIAL_UDB_EXTRA F15 and F16: format
type SerialUdbExtraF15 struct {
	SueIdVehicleModelName    [40]uint8 // Serial UDB Extra Model Name Of Vehicle
	SueIdVehicleRegistration [20]uint8 // Serial UDB Extra Registraton Number of Vehicle
}

func (self *SerialUdbExtraF15) TypeID() uint8 {
	return 179
}

func (self *SerialUdbExtraF15) TypeName() string {
	return "SERIAL_UDB_EXTRA_F15"
}

func (self *SerialUdbExtraF15) TypeSize() uint8 {
	return 60
}

func (self *SerialUdbExtraF15) TypeCRCExtra() uint8 {
	return 188
}

func (self *SerialUdbExtraF15) FieldsString() string {
	return fmt.Sprintf("SueIdVehicleModelName=%v SueIdVehicleRegistration=%v", self.SueIdVehicleModelName, self.SueIdVehicleRegistration)
}

func (self *SerialUdbExtraF15) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

//
type SerialUdbExtraF16 struct {
	SueIdLeadPilot    [40]uint8 // Serial UDB Extra Name of Expected Lead Pilot
	SueIdDiyDronesUrl [70]uint8 // Serial UDB Extra URL of Lead Pilot or Team
}

func (self *SerialUdbExtraF16) TypeID() uint8 {
	return 180
}

func (self *SerialUdbExtraF16) TypeName() string {
	return "SERIAL_UDB_EXTRA_F16"
}

func (self *SerialUdbExtraF16) TypeSize() uint8 {
	return 110
}

func (self *SerialUdbExtraF16) TypeCRCExtra() uint8 {
	return 9
}

func (self *SerialUdbExtraF16) FieldsString() string {
	return fmt.Sprintf("SueIdLeadPilot=%v SueIdDiyDronesUrl=%v", self.SueIdLeadPilot, self.SueIdDiyDronesUrl)
}

func (self *SerialUdbExtraF16) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// The altitude measured by sensors and IMU
type Altitudes struct {
	TimeBootMs     uint32 // Timestamp (milliseconds since system boot)
	AltGps         int32  // GPS altitude in meters, expressed as * 1000 (millimeters), above MSL
	AltImu         int32  // IMU altitude above ground in meters, expressed as * 1000 (millimeters)
	AltBarometric  int32  // barometeric altitude above ground in meters, expressed as * 1000 (millimeters)
	AltOpticalFlow int32  // Optical flow altitude above ground in meters, expressed as * 1000 (millimeters)
	AltRangeFinder int32  // Rangefinder Altitude above ground in meters, expressed as * 1000 (millimeters)
	AltExtra       int32  // Extra altitude above ground in meters, expressed as * 1000 (millimeters)
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
	return 55
}

func (self *Altitudes) FieldsString() string {
	return fmt.Sprintf("TimeBootMs=%d AltGps=%d AltImu=%d AltBarometric=%d AltOpticalFlow=%d AltRangeFinder=%d AltExtra=%d", self.TimeBootMs, self.AltGps, self.AltImu, self.AltBarometric, self.AltOpticalFlow, self.AltRangeFinder, self.AltExtra)
}

func (self *Altitudes) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// The airspeed measured by sensors and IMU
type Airspeeds struct {
	TimeBootMs         uint32 // Timestamp (milliseconds since system boot)
	AirspeedImu        int16  // Airspeed estimate from IMU, cm/s
	AirspeedPitot      int16  // Pitot measured forward airpseed, cm/s
	AirspeedHotWire    int16  // Hot wire anenometer measured airspeed, cm/s
	AirspeedUltrasonic int16  // Ultrasonic measured airspeed, cm/s
	Aoa                int16  // Angle of attack sensor, degrees * 10
	Aoy                int16  // Yaw angle sensor, degrees * 10
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
	return 154
}

func (self *Airspeeds) FieldsString() string {
	return fmt.Sprintf("TimeBootMs=%d AirspeedImu=%d AirspeedPitot=%d AirspeedHotWire=%d AirspeedUltrasonic=%d Aoa=%d Aoy=%d", self.TimeBootMs, self.AirspeedImu, self.AirspeedPitot, self.AirspeedHotWire, self.AirspeedUltrasonic, self.Aoa, self.Aoy)
}

func (self *Airspeeds) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

////////////////////////////////////////////////////////////////////////////////
// String Helpers
////////////////////////////////////////////////////////////////////////////////

type Char20 [20]byte

func (chars *Char20) String() string {
	return mavlink.FixString(chars[:])
}

type Char40 [40]byte

func (chars *Char40) String() string {
	return mavlink.FixString(chars[:])
}

type Char48 [48]byte

func (chars *Char48) String() string {
	return mavlink.FixString(chars[:])
}

type Char70 [70]byte

func (chars *Char70) String() string {
	return mavlink.FixString(chars[:])
}
