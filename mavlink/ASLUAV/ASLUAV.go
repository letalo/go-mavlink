package ASLUAV

import (
	"fmt"

	"github.com/SpaceLeap/go-mavlink/mavlink"
	"github.com/SpaceLeap/go-mavlink/mavlink/common"
)

const (
	PROTOCOL_NAME    = "ASLUAV"
	PROTOCOL_VERSION = 0
	PROTOCOL_INCLUDE = common.PROTOCOL_NAME
)

func init() {
	mavlink.ProtocolName = PROTOCOL_NAME
	mavlink.ProtocolVersion = PROTOCOL_VERSION

	mavlink.MessageFactory[201] = func() mavlink.Message { return new(SensPower) }
	mavlink.MessageFactory[202] = func() mavlink.Message { return new(SensMppt) }
	mavlink.MessageFactory[203] = func() mavlink.Message { return new(AslctrlData) }
	mavlink.MessageFactory[204] = func() mavlink.Message { return new(AslctrlDebug) }
	mavlink.MessageFactory[205] = func() mavlink.Message { return new(AsluavStatus) }
}

////////////////////////////////////////////////////////////////////////////////
// Enums
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Messages
////////////////////////////////////////////////////////////////////////////////

// Voltage and current sensor data
type SensPower struct {
	Adc121Cs2Amp   float32 //  Board current sensor 2 reading in amps
	Adc121Cs1Amp   float32 //  Board current sensor 1 reading in amps
	Adc121CspbAmp  float32 //  Power board current sensor reading in amps
	Adc121VspbVolt float32 //  Power board voltage sensor reading in volts
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
	return 145
}

func (self *SensPower) FieldsString() string {
	return fmt.Sprintf("Adc121Cs2Amp=%d Adc121Cs1Amp=%d Adc121CspbAmp=%d Adc121VspbVolt=%d", self.Adc121Cs2Amp, self.Adc121Cs1Amp, self.Adc121CspbAmp, self.Adc121VspbVolt)
}

func (self *SensPower) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Maximum Power Point Tracker (MPPT) sensor data for solar module power performance tracking
type SensMppt struct {
	MpptTimestamp uint64  //  MPPT last timestamp
	Mppt3Amp      float32 //  MPPT3 current
	Mppt3Volt     float32 //  MPPT3 voltage
	Mppt2Amp      float32 //  MPPT2 current
	Mppt2Volt     float32 //  MPPT2 voltage
	Mppt1Amp      float32 //  MPPT1 current
	Mppt1Volt     float32 //  MPPT1 voltage
	Mppt3Pwm      uint16  //  MPPT3 pwm
	Mppt2Pwm      uint16  //  MPPT2 pwm
	Mppt1Pwm      uint16  //  MPPT1 pwm
	Mppt3Status   uint8   //  MPPT3 status
	Mppt2Status   uint8   //  MPPT2 status
	Mppt1Status   uint8   //  MPPT1 status
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
	return 57
}

func (self *SensMppt) FieldsString() string {
	return fmt.Sprintf("MpptTimestamp=%d Mppt3Amp=%d Mppt3Volt=%d Mppt2Amp=%d Mppt2Volt=%d Mppt1Amp=%d Mppt1Volt=%d Mppt3Pwm=%d Mppt2Pwm=%d Mppt1Pwm=%d Mppt3Status=%d Mppt2Status=%d Mppt1Status=%d", self.MpptTimestamp, self.Mppt3Amp, self.Mppt3Volt, self.Mppt2Amp, self.Mppt2Volt, self.Mppt1Amp, self.Mppt1Volt, self.Mppt3Pwm, self.Mppt2Pwm, self.Mppt1Pwm, self.Mppt3Status, self.Mppt2Status, self.Mppt1Status)
}

func (self *SensMppt) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// ASL-fixed-wing controller data
type AslctrlData struct {
	Timestamp       uint64  //  Timestamp
	Urud            float32 //
	Uail            float32 //
	Rref            float32 //
	R               float32 //
	Pref            float32 //
	P               float32 //
	Rollangleref    float32 // Roll angle reference[deg]
	Rollangle       float32 // Roll angle [deg]
	Yawangleref     float32 // Yaw angle reference[deg]
	Yawangle        float32 // Yaw angle [deg]
	Airspeedref     float32 // Airspeed reference [m/s]
	Az              float32 //
	Uthrot2         float32 //
	Uthrot          float32 //
	Uelev           float32 //
	Qref            float32 //
	Q               float32 //
	Pitchangleref   float32 // Pitch angle reference[deg]
	Pitchangle      float32 // Pitch angle [deg]
	HrefT           float32 //
	Href            float32 //
	H               float32 //  See sourcecode for a description of these values...
	Spoilersengaged uint8   //
	AslctrlMode     uint8   //  ASLCTRL control-mode (manual, stabilized, auto, etc...)
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
	return 172
}

func (self *AslctrlData) FieldsString() string {
	return fmt.Sprintf("Timestamp=%d Urud=%d Uail=%d Rref=%d R=%d Pref=%d P=%d Rollangleref=%d Rollangle=%d Yawangleref=%d Yawangle=%d Airspeedref=%d Az=%d Uthrot2=%d Uthrot=%d Uelev=%d Qref=%d Q=%d Pitchangleref=%d Pitchangle=%d HrefT=%d Href=%d H=%d Spoilersengaged=%d AslctrlMode=%d", self.Timestamp, self.Urud, self.Uail, self.Rref, self.R, self.Pref, self.P, self.Rollangleref, self.Rollangle, self.Yawangleref, self.Yawangle, self.Airspeedref, self.Az, self.Uthrot2, self.Uthrot, self.Uelev, self.Qref, self.Q, self.Pitchangleref, self.Pitchangle, self.HrefT, self.Href, self.H, self.Spoilersengaged, self.AslctrlMode)
}

func (self *AslctrlData) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// ASL-fixed-wing controller debug data
type AslctrlDebug struct {
	F8   float32 //  Debug data
	F7   float32 //  Debug data
	F6   float32 //  Debug data
	F5   float32 //  Debug data
	F4   float32 //  Debug data
	F3   float32 //  Debug data
	F2   float32 //  Debug data
	F1   float32 //  Debug data
	I321 uint32  //  Debug data
	I82  uint8   //  Debug data
	I81  uint8   //  Debug data
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
	return 182
}

func (self *AslctrlDebug) FieldsString() string {
	return fmt.Sprintf("F8=%d F7=%d F6=%d F5=%d F4=%d F3=%d F2=%d F1=%d I321=%d I82=%d I81=%d", self.F8, self.F7, self.F6, self.F5, self.F4, self.F3, self.F2, self.F1, self.I321, self.I82, self.I81)
}

func (self *AslctrlDebug) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Extended state information for ASLUAVs
type AsluavStatus struct {
	MotorRpm     float32  //  Motor RPM
	ServoStatus  [8]uint8 //  Status vector for up to 8 servos
	SatcomStatus uint8    //  Status of the IRIDIUM satellite communication system
	LedStatus    uint8    //  Status of the position-indicator LEDs
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
	return 58
}

func (self *AsluavStatus) FieldsString() string {
	return fmt.Sprintf("MotorRpm=%d ServoStatus=%v SatcomStatus=%d LedStatus=%d", self.MotorRpm, self.ServoStatus, self.SatcomStatus, self.LedStatus)
}

func (self *AsluavStatus) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

////////////////////////////////////////////////////////////////////////////////
// String Helpers
////////////////////////////////////////////////////////////////////////////////

func truncateZeroTerminator(chars []byte) []byte {
	for i, c := range chars {
		if c == 0 {
			return chars[:i]
		}
	}
	return chars
}

type Char8 [8]byte

func (chars *Char8) String() string {
	return string(truncateZeroTerminator(chars[:]))
}
