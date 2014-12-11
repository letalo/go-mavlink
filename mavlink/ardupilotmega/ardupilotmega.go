package ardupilotmega

import (
	"fmt"

	"github.com/SpaceLeap/go-mavlink/mavlink"
	"github.com/SpaceLeap/go-mavlink/mavlink/common"
)

const (
	PROTOCOL_NAME    = "ardupilotmega"
	PROTOCOL_VERSION = 0
	PROTOCOL_INCLUDE = common.PROTOCOL_NAME
)

func Init() {
	common.Init()

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
	mavlink.MessageFactory[182] = func() mavlink.Message { return new(Ahrs3) }
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
	MagDeclination float32 // magnetic declination (radians)
	RawPress       int32   // raw pressure from barometer
	RawTemp        int32   // raw temperature from barometer
	GyroCalX       float32 // gyro X calibration
	GyroCalY       float32 // gyro Y calibration
	GyroCalZ       float32 // gyro Z calibration
	AccelCalX      float32 // accel X calibration
	AccelCalY      float32 // accel Y calibration
	AccelCalZ      float32 // accel Z calibration
	MagOfsX        int16   // magnetometer X offset
	MagOfsY        int16   // magnetometer Y offset
	MagOfsZ        int16   // magnetometer Z offset
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
	return 134
}

func (self *SensorOffsets) FieldsString() string {
	return fmt.Sprintf("MagDeclination=%d RawPress=%d RawTemp=%d GyroCalX=%d GyroCalY=%d GyroCalZ=%d AccelCalX=%d AccelCalY=%d AccelCalZ=%d MagOfsX=%d MagOfsY=%d MagOfsZ=%d", self.MagDeclination, self.RawPress, self.RawTemp, self.GyroCalX, self.GyroCalY, self.GyroCalZ, self.AccelCalX, self.AccelCalY, self.AccelCalZ, self.MagOfsX, self.MagOfsY, self.MagOfsZ)
}

func (self *SensorOffsets) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Deprecated. Use MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS instead. Set the magnetometer offsets
type SetMagOffsets struct {
	MagOfsX         int16 // magnetometer X offset
	MagOfsY         int16 // magnetometer Y offset
	MagOfsZ         int16 // magnetometer Z offset
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
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
	return 219
}

func (self *SetMagOffsets) FieldsString() string {
	return fmt.Sprintf("MagOfsX=%d MagOfsY=%d MagOfsZ=%d TargetSystem=%d TargetComponent=%d", self.MagOfsX, self.MagOfsY, self.MagOfsZ, self.TargetSystem, self.TargetComponent)
}

func (self *SetMagOffsets) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// state of APM memory
type Meminfo struct {
	Brkval  uint16 // heap top
	Freemem uint16 // free memory
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
	return 208
}

func (self *Meminfo) FieldsString() string {
	return fmt.Sprintf("Brkval=%d Freemem=%d", self.Brkval, self.Freemem)
}

func (self *Meminfo) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// raw ADC output
type ApAdc struct {
	Adc1 uint16 // ADC output 1
	Adc2 uint16 // ADC output 2
	Adc3 uint16 // ADC output 3
	Adc4 uint16 // ADC output 4
	Adc5 uint16 // ADC output 5
	Adc6 uint16 // ADC output 6
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
	return 188
}

func (self *ApAdc) FieldsString() string {
	return fmt.Sprintf("Adc1=%d Adc2=%d Adc3=%d Adc4=%d Adc5=%d Adc6=%d", self.Adc1, self.Adc2, self.Adc3, self.Adc4, self.Adc5, self.Adc6)
}

func (self *ApAdc) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Configure on-board Camera Control System.
type DigicamConfigure struct {
	ExtraValue      float32 // Correspondent value to given extra_param
	ShutterSpeed    uint16  // Divisor number //e.g. 1000 means 1/1000 (0 means ignore)
	TargetSystem    uint8   // System ID
	TargetComponent uint8   // Component ID
	Mode            uint8   // Mode enumeration from 1 to N //P, TV, AV, M, Etc (0 means ignore)
	Aperture        uint8   // F stop number x 10 //e.g. 28 means 2.8 (0 means ignore)
	Iso             uint8   // ISO enumeration from 1 to N //e.g. 80, 100, 200, Etc (0 means ignore)
	ExposureType    uint8   // Exposure type enumeration from 1 to N (0 means ignore)
	CommandId       uint8   // Command Identity (incremental loop: 0 to 255)//A command sent multiple times will be executed or pooled just once
	EngineCutOff    uint8   // Main engine cut-off time before camera trigger in seconds/10 (0 means no cut-off)
	ExtraParam      uint8   // Extra parameters enumeration (0 means ignore)
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
	return 84
}

func (self *DigicamConfigure) FieldsString() string {
	return fmt.Sprintf("ExtraValue=%d ShutterSpeed=%d TargetSystem=%d TargetComponent=%d Mode=%d Aperture=%d Iso=%d ExposureType=%d CommandId=%d EngineCutOff=%d ExtraParam=%d", self.ExtraValue, self.ShutterSpeed, self.TargetSystem, self.TargetComponent, self.Mode, self.Aperture, self.Iso, self.ExposureType, self.CommandId, self.EngineCutOff, self.ExtraParam)
}

func (self *DigicamConfigure) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Control on-board Camera Control System to take shots.
type DigicamControl struct {
	ExtraValue      float32 // Correspondent value to given extra_param
	TargetSystem    uint8   // System ID
	TargetComponent uint8   // Component ID
	Session         uint8   // 0: stop, 1: start or keep it up //Session control e.g. show/hide lens
	ZoomPos         uint8   // 1 to N //Zoom's absolute position (0 means ignore)
	ZoomStep        int8    // -100 to 100 //Zooming step value to offset zoom from the current position
	FocusLock       uint8   // 0: unlock focus or keep unlocked, 1: lock focus or keep locked, 3: re-lock focus
	Shot            uint8   // 0: ignore, 1: shot or start filming
	CommandId       uint8   // Command Identity (incremental loop: 0 to 255)//A command sent multiple times will be executed or pooled just once
	ExtraParam      uint8   // Extra parameters enumeration (0 means ignore)
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
	return 22
}

func (self *DigicamControl) FieldsString() string {
	return fmt.Sprintf("ExtraValue=%d TargetSystem=%d TargetComponent=%d Session=%d ZoomPos=%d ZoomStep=%d FocusLock=%d Shot=%d CommandId=%d ExtraParam=%d", self.ExtraValue, self.TargetSystem, self.TargetComponent, self.Session, self.ZoomPos, self.ZoomStep, self.FocusLock, self.Shot, self.CommandId, self.ExtraParam)
}

func (self *DigicamControl) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Message to configure a camera mount, directional antenna, etc.
type MountConfigure struct {
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
	MountMode       uint8 // mount operating mode (see MAV_MOUNT_MODE enum)
	StabRoll        uint8 // (1 = yes, 0 = no)
	StabPitch       uint8 // (1 = yes, 0 = no)
	StabYaw         uint8 // (1 = yes, 0 = no)
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
	return 19
}

func (self *MountConfigure) FieldsString() string {
	return fmt.Sprintf("TargetSystem=%d TargetComponent=%d MountMode=%d StabRoll=%d StabPitch=%d StabYaw=%d", self.TargetSystem, self.TargetComponent, self.MountMode, self.StabRoll, self.StabPitch, self.StabYaw)
}

func (self *MountConfigure) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Message to control a camera mount, directional antenna, etc.
type MountControl struct {
	InputA          int32 // pitch(deg*100) or lat, depending on mount mode
	InputB          int32 // roll(deg*100) or lon depending on mount mode
	InputC          int32 // yaw(deg*100) or alt (in cm) depending on mount mode
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
	SavePosition    uint8 // if "1" it will save current trimmed position on EEPROM (just valid for NEUTRAL and LANDING)
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
	return 21
}

func (self *MountControl) FieldsString() string {
	return fmt.Sprintf("InputA=%d InputB=%d InputC=%d TargetSystem=%d TargetComponent=%d SavePosition=%d", self.InputA, self.InputB, self.InputC, self.TargetSystem, self.TargetComponent, self.SavePosition)
}

func (self *MountControl) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Message with some status from APM to GCS about camera or antenna mount
type MountStatus struct {
	PointingA       int32 // pitch(deg*100)
	PointingB       int32 // roll(deg*100)
	PointingC       int32 // yaw(deg*100)
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
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
	return 134
}

func (self *MountStatus) FieldsString() string {
	return fmt.Sprintf("PointingA=%d PointingB=%d PointingC=%d TargetSystem=%d TargetComponent=%d", self.PointingA, self.PointingB, self.PointingC, self.TargetSystem, self.TargetComponent)
}

func (self *MountStatus) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// A fence point. Used to set a point when from
// 	      GCS -> MAV. Also used to return a point from MAV -> GCS
type FencePoint struct {
	Lat             float32 // Latitude of point
	Lng             float32 // Longitude of point
	TargetSystem    uint8   // System ID
	TargetComponent uint8   // Component ID
	Idx             uint8   // point index (first point is 1, 0 is for return point)
	Count           uint8   // total number of points (for sanity checking)
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
	return 78
}

func (self *FencePoint) FieldsString() string {
	return fmt.Sprintf("Lat=%d Lng=%d TargetSystem=%d TargetComponent=%d Idx=%d Count=%d", self.Lat, self.Lng, self.TargetSystem, self.TargetComponent, self.Idx, self.Count)
}

func (self *FencePoint) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Request a current fence point from MAV
type FenceFetchPoint struct {
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
	Idx             uint8 // point index (first point is 1, 0 is for return point)
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
	return 68
}

func (self *FenceFetchPoint) FieldsString() string {
	return fmt.Sprintf("TargetSystem=%d TargetComponent=%d Idx=%d", self.TargetSystem, self.TargetComponent, self.Idx)
}

func (self *FenceFetchPoint) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Status of geo-fencing. Sent in extended
// 	    status stream when fencing enabled
type FenceStatus struct {
	BreachTime   uint32 // time of last breach in milliseconds since boot
	BreachCount  uint16 // number of fence breaches
	BreachStatus uint8  // 0 if currently inside fence, 1 if outside
	BreachType   uint8  // last breach type (see FENCE_BREACH_* enum)
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
	return 189
}

func (self *FenceStatus) FieldsString() string {
	return fmt.Sprintf("BreachTime=%d BreachCount=%d BreachStatus=%d BreachType=%d", self.BreachTime, self.BreachCount, self.BreachStatus, self.BreachType)
}

func (self *FenceStatus) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Status of DCM attitude estimator
type Ahrs struct {
	Omegaix     float32 // X gyro drift estimate rad/s
	Omegaiy     float32 // Y gyro drift estimate rad/s
	Omegaiz     float32 // Z gyro drift estimate rad/s
	AccelWeight float32 // average accel_weight
	RenormVal   float32 // average renormalisation value
	ErrorRp     float32 // average error_roll_pitch value
	ErrorYaw    float32 // average error_yaw value
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
	return 127
}

func (self *Ahrs) FieldsString() string {
	return fmt.Sprintf("Omegaix=%d Omegaiy=%d Omegaiz=%d AccelWeight=%d RenormVal=%d ErrorRp=%d ErrorYaw=%d", self.Omegaix, self.Omegaiy, self.Omegaiz, self.AccelWeight, self.RenormVal, self.ErrorRp, self.ErrorYaw)
}

func (self *Ahrs) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Status of simulation environment, if used
type Simstate struct {
	Roll  float32 // Roll angle (rad)
	Pitch float32 // Pitch angle (rad)
	Yaw   float32 // Yaw angle (rad)
	Xacc  float32 // X acceleration m/s/s
	Yacc  float32 // Y acceleration m/s/s
	Zacc  float32 // Z acceleration m/s/s
	Xgyro float32 // Angular speed around X axis rad/s
	Ygyro float32 // Angular speed around Y axis rad/s
	Zgyro float32 // Angular speed around Z axis rad/s
	Lat   int32   // Latitude in degrees * 1E7
	Lng   int32   // Longitude in degrees * 1E7
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
	return 154
}

func (self *Simstate) FieldsString() string {
	return fmt.Sprintf("Roll=%d Pitch=%d Yaw=%d Xacc=%d Yacc=%d Zacc=%d Xgyro=%d Ygyro=%d Zgyro=%d Lat=%d Lng=%d", self.Roll, self.Pitch, self.Yaw, self.Xacc, self.Yacc, self.Zacc, self.Xgyro, self.Ygyro, self.Zgyro, self.Lat, self.Lng)
}

func (self *Simstate) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
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

func (self *Hwstatus) FieldsString() string {
	return fmt.Sprintf("Vcc=%d I2cerr=%d", self.Vcc, self.I2cerr)
}

func (self *Hwstatus) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Status generated by radio
type Radio struct {
	Rxerrors uint16 // receive errors
	Fixed    uint16 // count of error corrected packets
	Rssi     uint8  // local signal strength
	Remrssi  uint8  // remote signal strength
	Txbuf    uint8  // how full the tx buffer is as a percentage
	Noise    uint8  // background noise level
	Remnoise uint8  // remote background noise level
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
	return 21
}

func (self *Radio) FieldsString() string {
	return fmt.Sprintf("Rxerrors=%d Fixed=%d Rssi=%d Remrssi=%d Txbuf=%d Noise=%d Remnoise=%d", self.Rxerrors, self.Fixed, self.Rssi, self.Remrssi, self.Txbuf, self.Noise, self.Remnoise)
}

func (self *Radio) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Status of AP_Limits. Sent in extended
// 	    status stream when AP_Limits is enabled
type LimitsStatus struct {
	LastTrigger   uint32 // time of last breach in milliseconds since boot
	LastAction    uint32 // time of last recovery action in milliseconds since boot
	LastRecovery  uint32 // time of last successful recovery in milliseconds since boot
	LastClear     uint32 // time of last all-clear in milliseconds since boot
	BreachCount   uint16 // number of fence breaches
	LimitsState   uint8  // state of AP_Limits, (see enum LimitState, LIMITS_STATE)
	ModsEnabled   uint8  // AP_Limit_Module bitfield of enabled modules, (see enum moduleid or LIMIT_MODULE)
	ModsRequired  uint8  // AP_Limit_Module bitfield of required modules, (see enum moduleid or LIMIT_MODULE)
	ModsTriggered uint8  // AP_Limit_Module bitfield of triggered modules, (see enum moduleid or LIMIT_MODULE)
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
	return 144
}

func (self *LimitsStatus) FieldsString() string {
	return fmt.Sprintf("LastTrigger=%d LastAction=%d LastRecovery=%d LastClear=%d BreachCount=%d LimitsState=%d ModsEnabled=%d ModsRequired=%d ModsTriggered=%d", self.LastTrigger, self.LastAction, self.LastRecovery, self.LastClear, self.BreachCount, self.LimitsState, self.ModsEnabled, self.ModsRequired, self.ModsTriggered)
}

func (self *LimitsStatus) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Wind estimation
type Wind struct {
	Direction float32 // wind direction that wind is coming from (degrees)
	Speed     float32 // wind speed in ground plane (m/s)
	SpeedZ    float32 // vertical wind speed (m/s)
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
	return 1
}

func (self *Wind) FieldsString() string {
	return fmt.Sprintf("Direction=%d Speed=%d SpeedZ=%d", self.Direction, self.Speed, self.SpeedZ)
}

func (self *Wind) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Data packet, size 16
type Data16 struct {
	Type uint8     // data type
	Len  uint8     // data length
	Data [16]uint8 // raw data
}

func (self *Data16) TypeID() uint8 {
	return 169
}

func (self *Data16) TypeName() string {
	return "DATA16"
}

func (self *Data16) TypeSize() uint8 {
	return 18
}

func (self *Data16) TypeCRCExtra() uint8 {
	return 141
}

func (self *Data16) FieldsString() string {
	return fmt.Sprintf("Type=%d Len=%d Data=%v", self.Type, self.Len, self.Data)
}

func (self *Data16) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Data packet, size 32
type Data32 struct {
	Type uint8     // data type
	Len  uint8     // data length
	Data [32]uint8 // raw data
}

func (self *Data32) TypeID() uint8 {
	return 170
}

func (self *Data32) TypeName() string {
	return "DATA32"
}

func (self *Data32) TypeSize() uint8 {
	return 34
}

func (self *Data32) TypeCRCExtra() uint8 {
	return 12
}

func (self *Data32) FieldsString() string {
	return fmt.Sprintf("Type=%d Len=%d Data=%v", self.Type, self.Len, self.Data)
}

func (self *Data32) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Data packet, size 64
type Data64 struct {
	Type uint8     // data type
	Len  uint8     // data length
	Data [64]uint8 // raw data
}

func (self *Data64) TypeID() uint8 {
	return 171
}

func (self *Data64) TypeName() string {
	return "DATA64"
}

func (self *Data64) TypeSize() uint8 {
	return 66
}

func (self *Data64) TypeCRCExtra() uint8 {
	return 165
}

func (self *Data64) FieldsString() string {
	return fmt.Sprintf("Type=%d Len=%d Data=%v", self.Type, self.Len, self.Data)
}

func (self *Data64) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Data packet, size 96
type Data96 struct {
	Type uint8     // data type
	Len  uint8     // data length
	Data [96]uint8 // raw data
}

func (self *Data96) TypeID() uint8 {
	return 172
}

func (self *Data96) TypeName() string {
	return "DATA96"
}

func (self *Data96) TypeSize() uint8 {
	return 98
}

func (self *Data96) TypeCRCExtra() uint8 {
	return 22
}

func (self *Data96) FieldsString() string {
	return fmt.Sprintf("Type=%d Len=%d Data=%v", self.Type, self.Len, self.Data)
}

func (self *Data96) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Rangefinder reporting
type Rangefinder struct {
	Distance float32 // distance in meters
	Voltage  float32 // raw voltage if available, zero otherwise
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
	return 83
}

func (self *Rangefinder) FieldsString() string {
	return fmt.Sprintf("Distance=%d Voltage=%d", self.Distance, self.Voltage)
}

func (self *Rangefinder) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Airspeed auto-calibration
type AirspeedAutocal struct {
	Vx           float32 // GPS velocity north m/s
	Vy           float32 // GPS velocity east m/s
	Vz           float32 // GPS velocity down m/s
	DiffPressure float32 // Differential pressure pascals
	Eas2tas      float32 // Estimated to true airspeed ratio
	Ratio        float32 // Airspeed ratio
	StateX       float32 // EKF state x
	StateY       float32 // EKF state y
	StateZ       float32 // EKF state z
	Pax          float32 // EKF Pax
	Pby          float32 // EKF Pby
	Pcz          float32 // EKF Pcz
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
	return 167
}

func (self *AirspeedAutocal) FieldsString() string {
	return fmt.Sprintf("Vx=%d Vy=%d Vz=%d DiffPressure=%d Eas2tas=%d Ratio=%d StateX=%d StateY=%d StateZ=%d Pax=%d Pby=%d Pcz=%d", self.Vx, self.Vy, self.Vz, self.DiffPressure, self.Eas2tas, self.Ratio, self.StateX, self.StateY, self.StateZ, self.Pax, self.Pby, self.Pcz)
}

func (self *AirspeedAutocal) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// A rally point. Used to set a point when from GCS -> MAV. Also used to return a point from MAV -> GCS
type RallyPoint struct {
	Lat             int32  // Latitude of point in degrees * 1E7
	Lng             int32  // Longitude of point in degrees * 1E7
	Alt             int16  // Transit / loiter altitude in meters relative to home
	BreakAlt        int16  // Break altitude in meters relative to home
	LandDir         uint16 // Heading to aim for when landing. In centi-degrees.
	TargetSystem    uint8  // System ID
	TargetComponent uint8  // Component ID
	Idx             uint8  // point index (first point is 0)
	Count           uint8  // total number of points (for sanity checking)
	Flags           uint8  // See RALLY_FLAGS enum for definition of the bitmask.
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
	return 138
}

func (self *RallyPoint) FieldsString() string {
	return fmt.Sprintf("Lat=%d Lng=%d Alt=%d BreakAlt=%d LandDir=%d TargetSystem=%d TargetComponent=%d Idx=%d Count=%d Flags=%d", self.Lat, self.Lng, self.Alt, self.BreakAlt, self.LandDir, self.TargetSystem, self.TargetComponent, self.Idx, self.Count, self.Flags)
}

func (self *RallyPoint) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Request a current rally point from MAV. MAV should respond with a RALLY_POINT message. MAV should not respond if the request is invalid.
type RallyFetchPoint struct {
	TargetSystem    uint8 // System ID
	TargetComponent uint8 // Component ID
	Idx             uint8 // point index (first point is 0)
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
	return 234
}

func (self *RallyFetchPoint) FieldsString() string {
	return fmt.Sprintf("TargetSystem=%d TargetComponent=%d Idx=%d", self.TargetSystem, self.TargetComponent, self.Idx)
}

func (self *RallyFetchPoint) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Status of compassmot calibration
type CompassmotStatus struct {
	Current       float32 // current (amps)
	Compensationx float32 // Motor Compensation X
	Compensationy float32 // Motor Compensation Y
	Compensationz float32 // Motor Compensation Z
	Throttle      uint16  // throttle (percent*10)
	Interference  uint16  // interference (percent)
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
	return 240
}

func (self *CompassmotStatus) FieldsString() string {
	return fmt.Sprintf("Current=%d Compensationx=%d Compensationy=%d Compensationz=%d Throttle=%d Interference=%d", self.Current, self.Compensationx, self.Compensationy, self.Compensationz, self.Throttle, self.Interference)
}

func (self *CompassmotStatus) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Status of secondary AHRS filter if available
type Ahrs2 struct {
	Roll     float32 // Roll angle (rad)
	Pitch    float32 // Pitch angle (rad)
	Yaw      float32 // Yaw angle (rad)
	Altitude float32 // Altitude (MSL)
	Lat      int32   // Latitude in degrees * 1E7
	Lng      int32   // Longitude in degrees * 1E7
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
	return 47
}

func (self *Ahrs2) FieldsString() string {
	return fmt.Sprintf("Roll=%d Pitch=%d Yaw=%d Altitude=%d Lat=%d Lng=%d", self.Roll, self.Pitch, self.Yaw, self.Altitude, self.Lat, self.Lng)
}

func (self *Ahrs2) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Camera Event
type CameraStatus struct {
	TimeUsec     uint64  // Image timestamp (microseconds since UNIX epoch, according to camera clock)
	P1           float32 // Parameter 1 (meaning depends on event, see CAMERA_STATUS_TYPES enum)
	P2           float32 // Parameter 2 (meaning depends on event, see CAMERA_STATUS_TYPES enum)
	P3           float32 // Parameter 3 (meaning depends on event, see CAMERA_STATUS_TYPES enum)
	P4           float32 // Parameter 4 (meaning depends on event, see CAMERA_STATUS_TYPES enum)
	ImgIdx       uint16  // Image index
	TargetSystem uint8   // System ID
	CamIdx       uint8   // Camera ID
	EventId      uint8   // See CAMERA_STATUS_TYPES enum for definition of the bitmask
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
	return 189
}

func (self *CameraStatus) FieldsString() string {
	return fmt.Sprintf("TimeUsec=%d P1=%d P2=%d P3=%d P4=%d ImgIdx=%d TargetSystem=%d CamIdx=%d EventId=%d", self.TimeUsec, self.P1, self.P2, self.P3, self.P4, self.ImgIdx, self.TargetSystem, self.CamIdx, self.EventId)
}

func (self *CameraStatus) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Camera Capture Feedback
type CameraFeedback struct {
	TimeUsec     uint64  // Image timestamp (microseconds since UNIX epoch), as passed in by CAMERA_STATUS message (or autopilot if no CCB)
	Lat          int32   // Latitude in (deg * 1E7)
	Lng          int32   // Longitude in (deg * 1E7)
	AltMsl       float32 // Altitude Absolute (meters AMSL)
	AltRel       float32 // Altitude Relative (meters above HOME location)
	Roll         float32 // Camera Roll angle (earth frame, degrees, +-180)
	Pitch        float32 // Camera Pitch angle (earth frame, degrees, +-180)
	Yaw          float32 // Camera Yaw (earth frame, degrees, 0-360, true)
	FocLen       float32 // Focal Length (mm)
	ImgIdx       uint16  // Image index
	TargetSystem uint8   // System ID
	CamIdx       uint8   // Camera ID
	Flags        uint8   // See CAMERA_FEEDBACK_FLAGS enum for definition of the bitmask
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
	return 52
}

func (self *CameraFeedback) FieldsString() string {
	return fmt.Sprintf("TimeUsec=%d Lat=%d Lng=%d AltMsl=%d AltRel=%d Roll=%d Pitch=%d Yaw=%d FocLen=%d ImgIdx=%d TargetSystem=%d CamIdx=%d Flags=%d", self.TimeUsec, self.Lat, self.Lng, self.AltMsl, self.AltRel, self.Roll, self.Pitch, self.Yaw, self.FocLen, self.ImgIdx, self.TargetSystem, self.CamIdx, self.Flags)
}

func (self *CameraFeedback) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// 2nd Battery status
type Battery2 struct {
	Voltage        uint16 // voltage in millivolts
	CurrentBattery int16  // Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
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
	return 174
}

func (self *Battery2) FieldsString() string {
	return fmt.Sprintf("Voltage=%d CurrentBattery=%d", self.Voltage, self.CurrentBattery)
}

func (self *Battery2) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Status of third AHRS filter if available. This is for ANU research group (Ali and Sean)
type Ahrs3 struct {
	Roll     float32 // Roll angle (rad)
	Pitch    float32 // Pitch angle (rad)
	Yaw      float32 // Yaw angle (rad)
	Altitude float32 // Altitude (MSL)
	Lat      int32   // Latitude in degrees * 1E7
	Lng      int32   // Longitude in degrees * 1E7
	V1       float32 // test variable1
	V2       float32 // test variable2
	V3       float32 // test variable3
	V4       float32 // test variable4
}

func (self *Ahrs3) TypeID() uint8 {
	return 182
}

func (self *Ahrs3) TypeName() string {
	return "AHRS3"
}

func (self *Ahrs3) TypeSize() uint8 {
	return 40
}

func (self *Ahrs3) TypeCRCExtra() uint8 {
	return 229
}

func (self *Ahrs3) FieldsString() string {
	return fmt.Sprintf("Roll=%d Pitch=%d Yaw=%d Altitude=%d Lat=%d Lng=%d V1=%d V2=%d V3=%d V4=%d", self.Roll, self.Pitch, self.Yaw, self.Altitude, self.Lat, self.Lng, self.V1, self.V2, self.V3, self.V4)
}

func (self *Ahrs3) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

////////////////////////////////////////////////////////////////////////////////
// String Helpers
////////////////////////////////////////////////////////////////////////////////

type Char16 [16]byte

func (chars *Char16) String() string {
	return mavlink.FixString(chars[:])
}

type Char32 [32]byte

func (chars *Char32) String() string {
	return mavlink.FixString(chars[:])
}

type Char64 [64]byte

func (chars *Char64) String() string {
	return mavlink.FixString(chars[:])
}

type Char96 [96]byte

func (chars *Char96) String() string {
	return mavlink.FixString(chars[:])
}
