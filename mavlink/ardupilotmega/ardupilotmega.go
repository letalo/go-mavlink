package ardupilotmega

import (
	"github.com/SpaceLeap/go-mavlink/mavlink"
	"github.com/SpaceLeap/go-mavlink/mavlink/common"
)

const (
	PROTOCOL_NAME    = "ardupilotmega"
	PROTOCOL_VERSION = ""
	PROTOCOL_INCLUDE = common.PROTOCOL_NAME
)

func init() {
	mavlink.ProtocolName = PROTOCOL_NAME
	mavlink.ProtocolVersion = PROTOCOL_VERSION

	mavlink.MessageFactory[150] = func() mavlink.Message { return new(SensorOffsets) }
	mavlink.MessageFactory[151] = func() mavlink.Message { return new(SetMagOffsets) }
	mavlink.MessageFactory[152] = func() mavlink.Message { return new(Meminfo) }
	mavlink.MessageFactory[153] = func() mavlink.Message { return new(ApAdc) }
	mavlink.MessageFactory[154] = func() mavlink.Message { return new(DigicamConfigure) }
	mavlink.MessageFactory[155] = func() mavlink.Message { return new(DigicamControl) }
	mavlink.MessageFactory[156] = func() mavlink.Message { return new(MountConfigure) }
	mavlink.MessageFactory[157] = func() mavlink.Message { return new(MountControl) }
	mavlink.MessageFactory[158] = func() mavlink.Message { return new(MountStatus) }
	mavlink.MessageFactory[160] = func() mavlink.Message { return new(FencePoint) }
	mavlink.MessageFactory[161] = func() mavlink.Message { return new(FenceFetchPoint) }
	mavlink.MessageFactory[162] = func() mavlink.Message { return new(FenceStatus) }
	mavlink.MessageFactory[163] = func() mavlink.Message { return new(Ahrs) }
	mavlink.MessageFactory[164] = func() mavlink.Message { return new(Simstate) }
	mavlink.MessageFactory[165] = func() mavlink.Message { return new(Hwstatus) }
	mavlink.MessageFactory[166] = func() mavlink.Message { return new(Radio) }
	mavlink.MessageFactory[167] = func() mavlink.Message { return new(LimitsStatus) }
	mavlink.MessageFactory[168] = func() mavlink.Message { return new(Wind) }
	mavlink.MessageFactory[169] = func() mavlink.Message { return new(Data16) }
	mavlink.MessageFactory[170] = func() mavlink.Message { return new(Data32) }
	mavlink.MessageFactory[171] = func() mavlink.Message { return new(Data64) }
	mavlink.MessageFactory[172] = func() mavlink.Message { return new(Data96) }
	mavlink.MessageFactory[173] = func() mavlink.Message { return new(Rangefinder) }
	mavlink.MessageFactory[174] = func() mavlink.Message { return new(AirspeedAutocal) }
	mavlink.MessageFactory[175] = func() mavlink.Message { return new(RallyPoint) }
	mavlink.MessageFactory[176] = func() mavlink.Message { return new(RallyFetchPoint) }
	mavlink.MessageFactory[177] = func() mavlink.Message { return new(CompassmotStatus) }
	mavlink.MessageFactory[178] = func() mavlink.Message { return new(Ahrs2) }
	mavlink.MessageFactory[179] = func() mavlink.Message { return new(CameraStatus) }
	mavlink.MessageFactory[180] = func() mavlink.Message { return new(CameraFeedback) }
	mavlink.MessageFactory[181] = func() mavlink.Message { return new(Battery2) }
}

////////////////////////////////////////////////////////////////////////////////
// Enums
////////////////////////////////////////////////////////////////////////////////

// MAV_CMD:
const (
	MAV_CMD_DO_MOTOR_TEST = 209 // Mission command to perform motor test
	MAV_CMD_DO_GRIPPER    = 211 // Mission command to operate EPM gripper
)

// LIMITS_STATE:
const (
	LIMITS_INIT       = 0 //  pre-initialization
	LIMITS_DISABLED   = 1 //  disabled
	LIMITS_ENABLED    = 2 //  checking limits
	LIMITS_TRIGGERED  = 3 //  a limit has been breached
	LIMITS_RECOVERING = 4 //  taking action eg. RTL
	LIMITS_RECOVERED  = 5 //  we're no longer in breach of a limit
)

// LIMIT_MODULE:
const (
	LIMIT_GPSLOCK  = 1 //  pre-initialization
	LIMIT_GEOFENCE = 2 //  disabled
	LIMIT_ALTITUDE = 4 //  checking limits
)

// RALLY_FLAGS: Flags in RALLY_POINT message
const (
	FAVORABLE_WIND   = 1 // Flag set when requiring favorable winds for landing.
	LAND_IMMEDIATELY = 2 // Flag set when plane is to immediately descend to break altitude and land without GCS intervention.  Flag not set when plane is to loiter at Rally point until commanded to land.
)

// PARACHUTE_ACTION:
const (
	PARACHUTE_DISABLE = 0 // Disable parachute release
	PARACHUTE_ENABLE  = 1 // Enable parachute release
	PARACHUTE_RELEASE = 2 // Release parachute
)

// MOTOR_TEST_THROTTLE_TYPE:
const (
	MOTOR_TEST_THROTTLE_PERCENT = 0 // throttle as a percentage from 0 ~ 100
	MOTOR_TEST_THROTTLE_PWM     = 1 // throttle as an absolute PWM value (normally in range of 1000~2000)
	MOTOR_TEST_THROTTLE_PILOT   = 2 // throttle pass-through from pilot's transmitter
)

// GRIPPER_ACTIONS: Gripper actions.
const (
	GRIPPER_ACTION_RELEASE = 0 // gripper release of cargo
	GRIPPER_ACTION_GRAB    = 1 // gripper grabs onto cargo
)

// CAMERA_STATUS_TYPES:
const (
	CAMERA_STATUS_TYPE_HEARTBEAT  = 0 // Camera heartbeat, announce camera component ID at 1hz
	CAMERA_STATUS_TYPE_TRIGGER    = 1 // Camera image triggered
	CAMERA_STATUS_TYPE_DISCONNECT = 2 // Camera connection lost
	CAMERA_STATUS_TYPE_ERROR      = 3 // Camera unknown error
	CAMERA_STATUS_TYPE_LOWBATT    = 4 // Camera battery low. Parameter p1 shows reported voltage
	CAMERA_STATUS_TYPE_LOWSTORE   = 5 // Camera storage low. Parameter p1 shows reported shots remaining
	CAMERA_STATUS_TYPE_LOWSTOREV  = 6 // Camera storage low. Parameter p1 shows reported video minutes remaining
)

// CAMERA_FEEDBACK_FLAGS:
const (
	VIDEO       = 1 // Shooting video, not stills
	BADEXPOSURE = 2 // Unable to achieve requested exposure (e.g. shutter speed too low)
	CLOSEDLOOP  = 3 // Closed loop feedback from camera, we know for sure it has successfully taken a picture
	OPENLOOP    = 4 // Open loop camera, an image trigger has been requested but we can't know for sure it has successfully taken a picture
)

////////////////////////////////////////////////////////////////////////////////
// Messages
////////////////////////////////////////////////////////////////////////////////

// Offsets and calibrations values for hardware
//         sensors. This makes it easier to debug the calibration process.
type SensorOffsets struct {
	AccelCalZ      float32 // accel Z calibration
	AccelCalY      float32 // accel Y calibration
	AccelCalX      float32 // accel X calibration
	GyroCalZ       float32 // gyro Z calibration
	GyroCalY       float32 // gyro Y calibration
	GyroCalX       float32 // gyro X calibration
	RawTemp        int32   // raw temperature from barometer
	RawPress       int32   // raw pressure from barometer
	MagDeclination float32 // magnetic declination (radians)
	MagOfsZ        int16   // magnetometer Z offset
	MagOfsY        int16   // magnetometer Y offset
	MagOfsX        int16   // magnetometer X offset
}

func (self *SensorOffsets) TypeID() uint8 {
	return 150
}

func (self *SensorOffsets) TypeName() string {
	return "SENSOR_OFFSETS"
}

func (self *SensorOffsets) TypeSize() uint8 {
	return 42
}

func (self *SensorOffsets) TypeCRCExtra() uint8 {
	return 16
}

// Deprecated. Use MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS instead. Set the magnetometer offsets
type SetMagOffsets struct {
	MagOfsZ         int16 // magnetometer Z offset
	MagOfsY         int16 // magnetometer Y offset
	MagOfsX         int16 // magnetometer X offset
	TargetComponent uint8 // Component ID
	TargetSystem    uint8 // System ID
}

func (self *SetMagOffsets) TypeID() uint8 {
	return 151
}

func (self *SetMagOffsets) TypeName() string {
	return "SET_MAG_OFFSETS"
}

func (self *SetMagOffsets) TypeSize() uint8 {
	return 8
}

func (self *SetMagOffsets) TypeCRCExtra() uint8 {
	return 163
}

// state of APM memory
type Meminfo struct {
	Freemem uint16 // free memory
	Brkval  uint16 // heap top
}

func (self *Meminfo) TypeID() uint8 {
	return 152
}

func (self *Meminfo) TypeName() string {
	return "MEMINFO"
}

func (self *Meminfo) TypeSize() uint8 {
	return 4
}

func (self *Meminfo) TypeCRCExtra() uint8 {
	return 254
}

// raw ADC output
type ApAdc struct {
	Adc6 uint16 // ADC output 6
	Adc5 uint16 // ADC output 5
	Adc4 uint16 // ADC output 4
	Adc3 uint16 // ADC output 3
	Adc2 uint16 // ADC output 2
	Adc1 uint16 // ADC output 1
}

func (self *ApAdc) TypeID() uint8 {
	return 153
}

func (self *ApAdc) TypeName() string {
	return "AP_ADC"
}

func (self *ApAdc) TypeSize() uint8 {
	return 12
}

func (self *ApAdc) TypeCRCExtra() uint8 {
	return 181
}

// Configure on-board Camera Control System.
type DigicamConfigure struct {
	ExtraValue      float32 // Correspondent value to given extra_param
	ShutterSpeed    uint16  // Divisor number //e.g. 1000 means 1/1000 (0 means ignore)
	ExtraParam      uint8   // Extra parameters enumeration (0 means ignore)
	EngineCutOff    uint8   // Main engine cut-off time before camera trigger in seconds/10 (0 means no cut-off)
	CommandId       uint8   // Command Identity (incremental loop: 0 to 255)//A command sent multiple times will be executed or pooled just once
	ExposureType    uint8   // Exposure type enumeration from 1 to N (0 means ignore)
	Iso             uint8   // ISO enumeration from 1 to N //e.g. 80, 100, 200, Etc (0 means ignore)
	Aperture        uint8   // F stop number x 10 //e.g. 28 means 2.8 (0 means ignore)
	Mode            uint8   // Mode enumeration from 1 to N //P, TV, AV, M, Etc (0 means ignore)
	TargetComponent uint8   // Component ID
	TargetSystem    uint8   // System ID
}

func (self *DigicamConfigure) TypeID() uint8 {
	return 154
}

func (self *DigicamConfigure) TypeName() string {
	return "DIGICAM_CONFIGURE"
}

func (self *DigicamConfigure) TypeSize() uint8 {
	return 15
}

func (self *DigicamConfigure) TypeCRCExtra() uint8 {
	return 93
}

// Control on-board Camera Control System to take shots.
type DigicamControl struct {
	ExtraValue      float32 // Correspondent value to given extra_param
	ExtraParam      uint8   // Extra parameters enumeration (0 means ignore)
	CommandId       uint8   // Command Identity (incremental loop: 0 to 255)//A command sent multiple times will be executed or pooled just once
	Shot            uint8   // 0: ignore, 1: shot or start filming
	FocusLock       uint8   // 0: unlock focus or keep unlocked, 1: lock focus or keep locked, 3: re-lock focus
	ZoomStep        int8    // -100 to 100 //Zooming step value to offset zoom from the current position
	ZoomPos         uint8   // 1 to N //Zoom's absolute position (0 means ignore)
	Session         uint8   // 0: stop, 1: start or keep it up //Session control e.g. show/hide lens
	TargetComponent uint8   // Component ID
	TargetSystem    uint8   // System ID
}

func (self *DigicamControl) TypeID() uint8 {
	return 155
}

func (self *DigicamControl) TypeName() string {
	return "DIGICAM_CONTROL"
}

func (self *DigicamControl) TypeSize() uint8 {
	return 13
}

func (self *DigicamControl) TypeCRCExtra() uint8 {
	return 52
}

// Message to configure a camera mount, directional antenna, etc.
type MountConfigure struct {
	StabYaw         uint8 // (1 = yes, 0 = no)
	StabPitch       uint8 // (1 = yes, 0 = no)
	StabRoll        uint8 // (1 = yes, 0 = no)
	MountMode       uint8 // mount operating mode (see MAV_MOUNT_MODE enum)
	TargetComponent uint8 // Component ID
	TargetSystem    uint8 // System ID
}

func (self *MountConfigure) TypeID() uint8 {
	return 156
}

func (self *MountConfigure) TypeName() string {
	return "MOUNT_CONFIGURE"
}

func (self *MountConfigure) TypeSize() uint8 {
	return 6
}

func (self *MountConfigure) TypeCRCExtra() uint8 {
	return 208
}

// Message to control a camera mount, directional antenna, etc.
type MountControl struct {
	InputC          int32 // yaw(deg*100) or alt (in cm) depending on mount mode
	InputB          int32 // roll(deg*100) or lon depending on mount mode
	InputA          int32 // pitch(deg*100) or lat, depending on mount mode
	SavePosition    uint8 // if "1" it will save current trimmed position on EEPROM (just valid for NEUTRAL and LANDING)
	TargetComponent uint8 // Component ID
	TargetSystem    uint8 // System ID
}

func (self *MountControl) TypeID() uint8 {
	return 157
}

func (self *MountControl) TypeName() string {
	return "MOUNT_CONTROL"
}

func (self *MountControl) TypeSize() uint8 {
	return 15
}

func (self *MountControl) TypeCRCExtra() uint8 {
	return 244
}

// Message with some status from APM to GCS about camera or antenna mount
type MountStatus struct {
	PointingC       int32 // yaw(deg*100) or alt (in cm) depending on mount mode
	PointingB       int32 // roll(deg*100) or lon depending on mount mode
	PointingA       int32 // pitch(deg*100) or lat, depending on mount mode
	TargetComponent uint8 // Component ID
	TargetSystem    uint8 // System ID
}

func (self *MountStatus) TypeID() uint8 {
	return 158
}

func (self *MountStatus) TypeName() string {
	return "MOUNT_STATUS"
}

func (self *MountStatus) TypeSize() uint8 {
	return 14
}

func (self *MountStatus) TypeCRCExtra() uint8 {
	return 47
}

// A fence point. Used to set a point when from
// 	      GCS -> MAV. Also used to return a point from MAV -> GCS
type FencePoint struct {
	Lng             float32 // Longitude of point
	Lat             float32 // Latitude of point
	Count           uint8   // total number of points (for sanity checking)
	Idx             uint8   // point index (first point is 1, 0 is for return point)
	TargetComponent uint8   // Component ID
	TargetSystem    uint8   // System ID
}

func (self *FencePoint) TypeID() uint8 {
	return 160
}

func (self *FencePoint) TypeName() string {
	return "FENCE_POINT"
}

func (self *FencePoint) TypeSize() uint8 {
	return 12
}

func (self *FencePoint) TypeCRCExtra() uint8 {
	return 142
}

// Request a current fence point from MAV
type FenceFetchPoint struct {
	Idx             uint8 // point index (first point is 1, 0 is for return point)
	TargetComponent uint8 // Component ID
	TargetSystem    uint8 // System ID
}

func (self *FenceFetchPoint) TypeID() uint8 {
	return 161
}

func (self *FenceFetchPoint) TypeName() string {
	return "FENCE_FETCH_POINT"
}

func (self *FenceFetchPoint) TypeSize() uint8 {
	return 3
}

func (self *FenceFetchPoint) TypeCRCExtra() uint8 {
	return 5
}

// Status of geo-fencing. Sent in extended
// 	    status stream when fencing enabled
type FenceStatus struct {
	BreachTime   uint32 // time of last breach in milliseconds since boot
	BreachCount  uint16 // number of fence breaches
	BreachType   uint8  // last breach type (see FENCE_BREACH_* enum)
	BreachStatus uint8  // 0 if currently inside fence, 1 if outside
}

func (self *FenceStatus) TypeID() uint8 {
	return 162
}

func (self *FenceStatus) TypeName() string {
	return "FENCE_STATUS"
}

func (self *FenceStatus) TypeSize() uint8 {
	return 8
}

func (self *FenceStatus) TypeCRCExtra() uint8 {
	return 116
}

// Status of DCM attitude estimator
type Ahrs struct {
	ErrorYaw    float32 // average error_yaw value
	ErrorRp     float32 // average error_roll_pitch value
	RenormVal   float32 // average renormalisation value
	AccelWeight float32 // average accel_weight
	Omegaiz     float32 // Z gyro drift estimate rad/s
	Omegaiy     float32 // Y gyro drift estimate rad/s
	Omegaix     float32 // X gyro drift estimate rad/s
}

func (self *Ahrs) TypeID() uint8 {
	return 163
}

func (self *Ahrs) TypeName() string {
	return "AHRS"
}

func (self *Ahrs) TypeSize() uint8 {
	return 28
}

func (self *Ahrs) TypeCRCExtra() uint8 {
	return 145
}

// Status of simulation environment, if used
type Simstate struct {
	Lng   int32   // Longitude in degrees * 1E7
	Lat   int32   // Latitude in degrees * 1E7
	Zgyro float32 // Angular speed around Z axis rad/s
	Ygyro float32 // Angular speed around Y axis rad/s
	Xgyro float32 // Angular speed around X axis rad/s
	Zacc  float32 // Z acceleration m/s/s
	Yacc  float32 // Y acceleration m/s/s
	Xacc  float32 // X acceleration m/s/s
	Yaw   float32 // Yaw angle (rad)
	Pitch float32 // Pitch angle (rad)
	Roll  float32 // Roll angle (rad)
}

func (self *Simstate) TypeID() uint8 {
	return 164
}

func (self *Simstate) TypeName() string {
	return "SIMSTATE"
}

func (self *Simstate) TypeSize() uint8 {
	return 44
}

func (self *Simstate) TypeCRCExtra() uint8 {
	return 12
}

// Status of key hardware
type Hwstatus struct {
	Vcc    uint16 // board voltage (mV)
	I2cerr uint8  // I2C error count
}

func (self *Hwstatus) TypeID() uint8 {
	return 165
}

func (self *Hwstatus) TypeName() string {
	return "HWSTATUS"
}

func (self *Hwstatus) TypeSize() uint8 {
	return 3
}

func (self *Hwstatus) TypeCRCExtra() uint8 {
	return 21
}

// Status generated by radio
type Radio struct {
	Fixed    uint16 // count of error corrected packets
	Rxerrors uint16 // receive errors
	Remnoise uint8  // remote background noise level
	Noise    uint8  // background noise level
	Txbuf    uint8  // how full the tx buffer is as a percentage
	Remrssi  uint8  // remote signal strength
	Rssi     uint8  // local signal strength
}

func (self *Radio) TypeID() uint8 {
	return 166
}

func (self *Radio) TypeName() string {
	return "RADIO"
}

func (self *Radio) TypeSize() uint8 {
	return 9
}

func (self *Radio) TypeCRCExtra() uint8 {
	return 91
}

// Status of AP_Limits. Sent in extended
// 	    status stream when AP_Limits is enabled
type LimitsStatus struct {
	LastClear     uint32 // time of last all-clear in milliseconds since boot
	LastRecovery  uint32 // time of last successful recovery in milliseconds since boot
	LastAction    uint32 // time of last recovery action in milliseconds since boot
	LastTrigger   uint32 // time of last breach in milliseconds since boot
	BreachCount   uint16 // number of fence breaches
	ModsTriggered uint8  // AP_Limit_Module bitfield of triggered modules, (see enum moduleid or LIMIT_MODULE)
	ModsRequired  uint8  // AP_Limit_Module bitfield of required modules, (see enum moduleid or LIMIT_MODULE)
	ModsEnabled   uint8  // AP_Limit_Module bitfield of enabled modules, (see enum moduleid or LIMIT_MODULE)
	LimitsState   uint8  // state of AP_Limits, (see enum LimitState, LIMITS_STATE)
}

func (self *LimitsStatus) TypeID() uint8 {
	return 167
}

func (self *LimitsStatus) TypeName() string {
	return "LIMITS_STATUS"
}

func (self *LimitsStatus) TypeSize() uint8 {
	return 22
}

func (self *LimitsStatus) TypeCRCExtra() uint8 {
	return 36
}

// Wind estimation
type Wind struct {
	SpeedZ    float32 // vertical wind speed (m/s)
	Speed     float32 // wind speed in ground plane (m/s)
	Direction float32 // wind direction that wind is coming from (degrees)
}

func (self *Wind) TypeID() uint8 {
	return 168
}

func (self *Wind) TypeName() string {
	return "WIND"
}

func (self *Wind) TypeSize() uint8 {
	return 12
}

func (self *Wind) TypeCRCExtra() uint8 {
	return 244
}

// Data packet, size 16
type Data16 struct {
	Data [16]uint8 // raw data
	Len  uint8     // data length
	Type uint8     // data type
}

func (self *Data16) TypeID() uint8 {
	return 169
}

func (self *Data16) TypeName() string {
	return "DATA16"
}

func (self *Data16) TypeSize() uint8 {
	return 3
}

func (self *Data16) TypeCRCExtra() uint8 {
	return 139
}

// Data packet, size 32
type Data32 struct {
	Data [32]uint8 // raw data
	Len  uint8     // data length
	Type uint8     // data type
}

func (self *Data32) TypeID() uint8 {
	return 170
}

func (self *Data32) TypeName() string {
	return "DATA32"
}

func (self *Data32) TypeSize() uint8 {
	return 3
}

func (self *Data32) TypeCRCExtra() uint8 {
	return 165
}

// Data packet, size 64
type Data64 struct {
	Data [64]uint8 // raw data
	Len  uint8     // data length
	Type uint8     // data type
}

func (self *Data64) TypeID() uint8 {
	return 171
}

func (self *Data64) TypeName() string {
	return "DATA64"
}

func (self *Data64) TypeSize() uint8 {
	return 3
}

func (self *Data64) TypeCRCExtra() uint8 {
	return 29
}

// Data packet, size 96
type Data96 struct {
	Data [96]uint8 // raw data
	Len  uint8     // data length
	Type uint8     // data type
}

func (self *Data96) TypeID() uint8 {
	return 172
}

func (self *Data96) TypeName() string {
	return "DATA96"
}

func (self *Data96) TypeSize() uint8 {
	return 3
}

func (self *Data96) TypeCRCExtra() uint8 {
	return 67
}

// Rangefinder reporting
type Rangefinder struct {
	Voltage  float32 // raw voltage if available, zero otherwise
	Distance float32 // distance in meters
}

func (self *Rangefinder) TypeID() uint8 {
	return 173
}

func (self *Rangefinder) TypeName() string {
	return "RANGEFINDER"
}

func (self *Rangefinder) TypeSize() uint8 {
	return 8
}

func (self *Rangefinder) TypeCRCExtra() uint8 {
	return 114
}

// Airspeed auto-calibration
type AirspeedAutocal struct {
	Pcz          float32 // EKF Pcz
	Pby          float32 // EKF Pby
	Pax          float32 // EKF Pax
	StateZ       float32 // EKF state z
	StateY       float32 // EKF state y
	StateX       float32 // EKF state x
	Ratio        float32 // Airspeed ratio
	Eas2tas      float32 // Estimated to true airspeed ratio
	DiffPressure float32 // Differential pressure pascals
	Vz           float32 // GPS velocity down m/s
	Vy           float32 // GPS velocity east m/s
	Vx           float32 // GPS velocity north m/s
}

func (self *AirspeedAutocal) TypeID() uint8 {
	return 174
}

func (self *AirspeedAutocal) TypeName() string {
	return "AIRSPEED_AUTOCAL"
}

func (self *AirspeedAutocal) TypeSize() uint8 {
	return 48
}

func (self *AirspeedAutocal) TypeCRCExtra() uint8 {
	return 251
}

// A rally point. Used to set a point when from GCS -> MAV. Also used to return a point from MAV -> GCS
type RallyPoint struct {
	Lng             int32  // Longitude of point in degrees * 1E7
	Lat             int32  // Latitude of point in degrees * 1E7
	LandDir         uint16 // Heading to aim for when landing. In centi-degrees.
	BreakAlt        int16  // Break altitude in meters relative to home
	Alt             int16  // Transit / loiter altitude in meters relative to home
	Flags           uint8  // See RALLY_FLAGS enum for definition of the bitmask.
	Count           uint8  // total number of points (for sanity checking)
	Idx             uint8  // point index (first point is 0)
	TargetComponent uint8  // Component ID
	TargetSystem    uint8  // System ID
}

func (self *RallyPoint) TypeID() uint8 {
	return 175
}

func (self *RallyPoint) TypeName() string {
	return "RALLY_POINT"
}

func (self *RallyPoint) TypeSize() uint8 {
	return 19
}

func (self *RallyPoint) TypeCRCExtra() uint8 {
	return 193
}

// Request a current rally point from MAV. MAV should respond with a RALLY_POINT message. MAV should not respond if the request is invalid.
type RallyFetchPoint struct {
	Idx             uint8 // point index (first point is 0)
	TargetComponent uint8 // Component ID
	TargetSystem    uint8 // System ID
}

func (self *RallyFetchPoint) TypeID() uint8 {
	return 176
}

func (self *RallyFetchPoint) TypeName() string {
	return "RALLY_FETCH_POINT"
}

func (self *RallyFetchPoint) TypeSize() uint8 {
	return 3
}

func (self *RallyFetchPoint) TypeCRCExtra() uint8 {
	return 171
}

// Status of compassmot calibration
type CompassmotStatus struct {
	Compensationz float32 // Motor Compensation Z
	Compensationy float32 // Motor Compensation Y
	Compensationx float32 // Motor Compensation X
	Current       float32 // current (amps)
	Interference  uint16  // interference (percent)
	Throttle      uint16  // throttle (percent*10)
}

func (self *CompassmotStatus) TypeID() uint8 {
	return 177
}

func (self *CompassmotStatus) TypeName() string {
	return "COMPASSMOT_STATUS"
}

func (self *CompassmotStatus) TypeSize() uint8 {
	return 20
}

func (self *CompassmotStatus) TypeCRCExtra() uint8 {
	return 90
}

// Status of secondary AHRS filter if available
type Ahrs2 struct {
	Lng      int32   // Longitude in degrees * 1E7
	Lat      int32   // Latitude in degrees * 1E7
	Altitude float32 // Altitude (MSL)
	Yaw      float32 // Yaw angle (rad)
	Pitch    float32 // Pitch angle (rad)
	Roll     float32 // Roll angle (rad)
}

func (self *Ahrs2) TypeID() uint8 {
	return 178
}

func (self *Ahrs2) TypeName() string {
	return "AHRS2"
}

func (self *Ahrs2) TypeSize() uint8 {
	return 24
}

func (self *Ahrs2) TypeCRCExtra() uint8 {
	return 79
}

// Camera Event
type CameraStatus struct {
	TimeUsec     uint64  // Image timestamp (microseconds since UNIX epoch, according to camera clock)
	P4           float32 // Parameter 4 (meaning depends on event, see CAMERA_STATUS_TYPES enum)
	P3           float32 // Parameter 3 (meaning depends on event, see CAMERA_STATUS_TYPES enum)
	P2           float32 // Parameter 2 (meaning depends on event, see CAMERA_STATUS_TYPES enum)
	P1           float32 // Parameter 1 (meaning depends on event, see CAMERA_STATUS_TYPES enum)
	ImgIdx       uint16  // Image index
	EventId      uint8   // See CAMERA_STATUS_TYPES enum for definition of the bitmask
	CamIdx       uint8   // Camera ID
	TargetSystem uint8   // System ID
}

func (self *CameraStatus) TypeID() uint8 {
	return 179
}

func (self *CameraStatus) TypeName() string {
	return "CAMERA_STATUS"
}

func (self *CameraStatus) TypeSize() uint8 {
	return 29
}

func (self *CameraStatus) TypeCRCExtra() uint8 {
	return 9
}

// Camera Capture Feedback
type CameraFeedback struct {
	TimeUsec     uint64  // Image timestamp (microseconds since UNIX epoch), as passed in by CAMERA_STATUS message (or autopilot if no CCB)
	FocLen       float32 // Focal Length (mm)
	Yaw          float32 // Camera Yaw (earth frame, degrees, 0-360, true)
	Pitch        float32 // Camera Pitch angle (earth frame, degrees, +-180)
	Roll         float32 // Camera Roll angle (earth frame, degrees, +-180)
	AltRel       float32 // Altitude Relative (meters above HOME location)
	AltMsl       float32 // Altitude Absolute (meters AMSL)
	Lng          int32   // Longitude in (deg * 1E7)
	Lat          int32   // Latitude in (deg * 1E7)
	ImgIdx       uint16  // Image index
	Flags        uint8   // See CAMERA_FEEDBACK_FLAGS enum for definition of the bitmask
	CamIdx       uint8   // Camera ID
	TargetSystem uint8   // System ID
}

func (self *CameraFeedback) TypeID() uint8 {
	return 180
}

func (self *CameraFeedback) TypeName() string {
	return "CAMERA_FEEDBACK"
}

func (self *CameraFeedback) TypeSize() uint8 {
	return 45
}

func (self *CameraFeedback) TypeCRCExtra() uint8 {
	return 55
}

// 2nd Battery status
type Battery2 struct {
	CurrentBattery int16  // Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
	Voltage        uint16 // voltage in millivolts
}

func (self *Battery2) TypeID() uint8 {
	return 181
}

func (self *Battery2) TypeName() string {
	return "BATTERY2"
}

func (self *Battery2) TypeSize() uint8 {
	return 4
}

func (self *Battery2) TypeCRCExtra() uint8 {
	return 115
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

type Char16 [16]byte

func (chars *Char16) String() string {
	return string(truncate(chars[:]))
}

type Char32 [32]byte

func (chars *Char32) String() string {
	return string(truncate(chars[:]))
}

type Char64 [64]byte

func (chars *Char64) String() string {
	return string(truncate(chars[:]))
}

type Char96 [96]byte

func (chars *Char96) String() string {
	return string(truncate(chars[:]))
}
