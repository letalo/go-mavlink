package ASLUAV

import (
	"fmt"

	"github.com/SpaceLeap/go-mavlink/mavlink"
	"github.com/SpaceLeap/go-mavlink/mavlink/common"
)

const (
	PROTOCOL_NAME = "ASLUAV"

	PROTOCOL_INCLUDE = common.PROTOCOL_NAME
)

// Init initializes mavlink.ProtocolName, mavlink.ProtocolVersion, and mavlink.MessageFactory.
func Init() {
	common.Init()

	mavlink.ProtocolName = PROTOCOL_NAME

	mavlink.MessageFactory[201] = func() mavlink.Message { return new(SensPower) }
	mavlink.MessageFactory[202] = func() mavlink.Message { return new(SensMppt) }
	mavlink.MessageFactory[203] = func() mavlink.Message { return new(AslctrlData) }
	mavlink.MessageFactory[204] = func() mavlink.Message { return new(AslctrlDebug) }
	mavlink.MessageFactory[205] = func() mavlink.Message { return new(AsluavStatus) }
	mavlink.MessageFactory[206] = func() mavlink.Message { return new(EkfExt) }
	mavlink.MessageFactory[207] = func() mavlink.Message { return new(AslObctrl) }
	mavlink.MessageFactory[208] = func() mavlink.Message { return new(SensAtmos) }
}

// MessageNameIDMap returns a map from message name to message ID.
func MessageNameIDMap() map[string]int {
	return map[string]int{
		"SENS_POWER":    201,
		"SENS_MPPT":     202,
		"ASLCTRL_DATA":  203,
		"ASLCTRL_DEBUG": 204,
		"ASLUAV_STATUS": 205,
		"EKF_EXT":       206,
		"ASL_OBCTRL":    207,
		"SENS_ATMOS":    208,
	}
}

////////////////////////////////////////////////////////////////////////////////
// Enums
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Messages
////////////////////////////////////////////////////////////////////////////////

// Voltage and current sensor data
type SensPower struct {
	Adc121VspbVolt float32 //  Power board voltage sensor reading in volts
	Adc121CspbAmp  float32 //  Power board current sensor reading in amps
	Adc121Cs1Amp   float32 //  Board current sensor 1 reading in amps
	Adc121Cs2Amp   float32 //  Board current sensor 2 reading in amps
}

func (self *SensPower) TypeID() uint8 {
	return 201
}

func (self *SensPower) TypeName() string {
	return "SENS_POWER"
}

func (self *SensPower) TypeSize() uint8 {
	return 16
}

func (self *SensPower) TypeCRCExtra() uint8 {
	return 218
}

func (self *SensPower) FieldsString() string {
	return fmt.Sprintf("Adc121VspbVolt=%f Adc121CspbAmp=%f Adc121Cs1Amp=%f Adc121Cs2Amp=%f", self.Adc121VspbVolt, self.Adc121CspbAmp, self.Adc121Cs1Amp, self.Adc121Cs2Amp)
}

func (self *SensPower) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Maximum Power Point Tracker (MPPT) sensor data for solar module power performance tracking
type SensMppt struct {
	MpptTimestamp uint64  //  MPPT last timestamp
	Mppt1Volt     float32 //  MPPT1 voltage
	Mppt1Amp      float32 //  MPPT1 current
	Mppt2Volt     float32 //  MPPT2 voltage
	Mppt2Amp      float32 //  MPPT2 current
	Mppt3Volt     float32 //  MPPT3 voltage
	Mppt3Amp      float32 //  MPPT3 current
	Mppt1Pwm      uint16  //  MPPT1 pwm
	Mppt2Pwm      uint16  //  MPPT2 pwm
	Mppt3Pwm      uint16  //  MPPT3 pwm
	Mppt1Status   uint8   //  MPPT1 status
	Mppt2Status   uint8   //  MPPT2 status
	Mppt3Status   uint8   //  MPPT3 status
}

func (self *SensMppt) TypeID() uint8 {
	return 202
}

func (self *SensMppt) TypeName() string {
	return "SENS_MPPT"
}

func (self *SensMppt) TypeSize() uint8 {
	return 41
}

func (self *SensMppt) TypeCRCExtra() uint8 {
	return 231
}

func (self *SensMppt) FieldsString() string {
	return fmt.Sprintf("MpptTimestamp=%d Mppt1Volt=%f Mppt1Amp=%f Mppt2Volt=%f Mppt2Amp=%f Mppt3Volt=%f Mppt3Amp=%f Mppt1Pwm=%d Mppt2Pwm=%d Mppt3Pwm=%d Mppt1Status=%d Mppt2Status=%d Mppt3Status=%d", self.MpptTimestamp, self.Mppt1Volt, self.Mppt1Amp, self.Mppt2Volt, self.Mppt2Amp, self.Mppt3Volt, self.Mppt3Amp, self.Mppt1Pwm, self.Mppt2Pwm, self.Mppt3Pwm, self.Mppt1Status, self.Mppt2Status, self.Mppt3Status)
}

func (self *SensMppt) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// ASL-fixed-wing controller data
type AslctrlData struct {
	Timestamp       uint64  //  Timestamp
	H               float32 //  See sourcecode for a description of these values...
	Href            float32 //
	HrefT           float32 //
	Pitchangle      float32 // Pitch angle [deg]
	Pitchangleref   float32 // Pitch angle reference[deg]
	Q               float32 //
	Qref            float32 //
	Uelev           float32 //
	Uthrot          float32 //
	Uthrot2         float32 //
	Az              float32 //
	Airspeedref     float32 // Airspeed reference [m/s]
	Yawangle        float32 // Yaw angle [deg]
	Yawangleref     float32 // Yaw angle reference[deg]
	Rollangle       float32 // Roll angle [deg]
	Rollangleref    float32 // Roll angle reference[deg]
	P               float32 //
	Pref            float32 //
	R               float32 //
	Rref            float32 //
	Uail            float32 //
	Urud            float32 //
	AslctrlMode     uint8   //  ASLCTRL control-mode (manual, stabilized, auto, etc...)
	Spoilersengaged uint8   //
}

func (self *AslctrlData) TypeID() uint8 {
	return 203
}

func (self *AslctrlData) TypeName() string {
	return "ASLCTRL_DATA"
}

func (self *AslctrlData) TypeSize() uint8 {
	return 98
}

func (self *AslctrlData) TypeCRCExtra() uint8 {
	return 0
}

func (self *AslctrlData) FieldsString() string {
	return fmt.Sprintf("Timestamp=%d H=%f Href=%f HrefT=%f Pitchangle=%f Pitchangleref=%f Q=%f Qref=%f Uelev=%f Uthrot=%f Uthrot2=%f Az=%f Airspeedref=%f Yawangle=%f Yawangleref=%f Rollangle=%f Rollangleref=%f P=%f Pref=%f R=%f Rref=%f Uail=%f Urud=%f AslctrlMode=%d Spoilersengaged=%d", self.Timestamp, self.H, self.Href, self.HrefT, self.Pitchangle, self.Pitchangleref, self.Q, self.Qref, self.Uelev, self.Uthrot, self.Uthrot2, self.Az, self.Airspeedref, self.Yawangle, self.Yawangleref, self.Rollangle, self.Rollangleref, self.P, self.Pref, self.R, self.Rref, self.Uail, self.Urud, self.AslctrlMode, self.Spoilersengaged)
}

func (self *AslctrlData) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// ASL-fixed-wing controller debug data
type AslctrlDebug struct {
	I321 uint32  //  Debug data
	F1   float32 //  Debug data
	F2   float32 //  Debug data
	F3   float32 //  Debug data
	F4   float32 //  Debug data
	F5   float32 //  Debug data
	F6   float32 //  Debug data
	F7   float32 //  Debug data
	F8   float32 //  Debug data
	I81  uint8   //  Debug data
	I82  uint8   //  Debug data
}

func (self *AslctrlDebug) TypeID() uint8 {
	return 204
}

func (self *AslctrlDebug) TypeName() string {
	return "ASLCTRL_DEBUG"
}

func (self *AslctrlDebug) TypeSize() uint8 {
	return 38
}

func (self *AslctrlDebug) TypeCRCExtra() uint8 {
	return 251
}

func (self *AslctrlDebug) FieldsString() string {
	return fmt.Sprintf("I321=%d F1=%f F2=%f F3=%f F4=%f F5=%f F6=%f F7=%f F8=%f I81=%d I82=%d", self.I321, self.F1, self.F2, self.F3, self.F4, self.F5, self.F6, self.F7, self.F8, self.I81, self.I82)
}

func (self *AslctrlDebug) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Extended state information for ASLUAVs
type AsluavStatus struct {
	MotorRpm     float32  //  Motor RPM
	LedStatus    uint8    //  Status of the position-indicator LEDs
	SatcomStatus uint8    //  Status of the IRIDIUM satellite communication system
	ServoStatus  [8]uint8 //  Status vector for up to 8 servos
}

func (self *AsluavStatus) TypeID() uint8 {
	return 205
}

func (self *AsluavStatus) TypeName() string {
	return "ASLUAV_STATUS"
}

func (self *AsluavStatus) TypeSize() uint8 {
	return 14
}

func (self *AsluavStatus) TypeCRCExtra() uint8 {
	return 165
}

func (self *AsluavStatus) FieldsString() string {
	return fmt.Sprintf("MotorRpm=%f LedStatus=%d SatcomStatus=%d ServoStatus=%v", self.MotorRpm, self.LedStatus, self.SatcomStatus, self.ServoStatus)
}

func (self *AsluavStatus) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Extended EKF state estimates for ASLUAVs
type EkfExt struct {
	Timestamp uint64  //  Time since system start [us]
	Windspeed float32 //  Magnitude of wind velocity (in lateral inertial plane) [m/s]
	Winddir   float32 //  Wind heading angle from North [rad]
	Windz     float32 //  Z (Down) component of inertial wind velocity [m/s]
	Airspeed  float32 //  Magnitude of air velocity [m/s]
	Beta      float32 //  Sideslip angle [rad]
	Alpha     float32 //  Angle of attack [rad]
}

func (self *EkfExt) TypeID() uint8 {
	return 206
}

func (self *EkfExt) TypeName() string {
	return "EKF_EXT"
}

func (self *EkfExt) TypeSize() uint8 {
	return 32
}

func (self *EkfExt) TypeCRCExtra() uint8 {
	return 64
}

func (self *EkfExt) FieldsString() string {
	return fmt.Sprintf("Timestamp=%d Windspeed=%f Winddir=%f Windz=%f Airspeed=%f Beta=%f Alpha=%f", self.Timestamp, self.Windspeed, self.Winddir, self.Windz, self.Airspeed, self.Beta, self.Alpha)
}

func (self *EkfExt) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Off-board controls/commands for ASLUAVs
type AslObctrl struct {
	Timestamp    uint64  //  Time since system start [us]
	Uelev        float32 //  Elevator command [~]
	Uthrot       float32 //  Throttle command [~]
	Uthrot2      float32 //  Throttle 2 command [~]
	Uaill        float32 //  Left aileron command [~]
	Uailr        float32 //  Right aileron command [~]
	Urud         float32 //  Rudder command [~]
	ObctrlStatus uint8   //  Off-board computer status
}

func (self *AslObctrl) TypeID() uint8 {
	return 207
}

func (self *AslObctrl) TypeName() string {
	return "ASL_OBCTRL"
}

func (self *AslObctrl) TypeSize() uint8 {
	return 33
}

func (self *AslObctrl) TypeCRCExtra() uint8 {
	return 234
}

func (self *AslObctrl) FieldsString() string {
	return fmt.Sprintf("Timestamp=%d Uelev=%f Uthrot=%f Uthrot2=%f Uaill=%f Uailr=%f Urud=%f ObctrlStatus=%d", self.Timestamp, self.Uelev, self.Uthrot, self.Uthrot2, self.Uaill, self.Uailr, self.Urud, self.ObctrlStatus)
}

func (self *AslObctrl) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Atmospheric sensors (temperature, humidity, ...)
type SensAtmos struct {
	Tempambient float32 //  Ambient temperature [degrees Celsius]
	Humidity    float32 //  Relative humidity [%]
}

func (self *SensAtmos) TypeID() uint8 {
	return 208
}

func (self *SensAtmos) TypeName() string {
	return "SENS_ATMOS"
}

func (self *SensAtmos) TypeSize() uint8 {
	return 8
}

func (self *SensAtmos) TypeCRCExtra() uint8 {
	return 175
}

func (self *SensAtmos) FieldsString() string {
	return fmt.Sprintf("Tempambient=%f Humidity=%f", self.Tempambient, self.Humidity)
}

func (self *SensAtmos) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

////////////////////////////////////////////////////////////////////////////////
// String Helpers
////////////////////////////////////////////////////////////////////////////////

type Char8 [8]byte

func (chars *Char8) String() string {
	return mavlink.FixString(chars[:])
}
