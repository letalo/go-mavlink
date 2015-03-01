package slugs

import (
	"fmt"

	"github.com/SpaceLeap/go-mavlink/mavlink"
	"github.com/SpaceLeap/go-mavlink/mavlink/common"
)

const (
	PROTOCOL_NAME = "slugs"

	PROTOCOL_INCLUDE = common.PROTOCOL_NAME
)

// Init initializes mavlink.ProtocolName, mavlink.ProtocolVersion, and mavlink.MessageFactory.
func Init() {
	common.Init()

	mavlink.ProtocolName = PROTOCOL_NAME

	mavlink.MessageFactory[170] = func() mavlink.Message { return new(CpuLoad) }
	mavlink.MessageFactory[172] = func() mavlink.Message { return new(SensorBias) }
	mavlink.MessageFactory[173] = func() mavlink.Message { return new(Diagnostic) }
	mavlink.MessageFactory[176] = func() mavlink.Message { return new(SlugsNavigation) }
	mavlink.MessageFactory[177] = func() mavlink.Message { return new(DataLog) }
	mavlink.MessageFactory[179] = func() mavlink.Message { return new(GpsDateTime) }
	mavlink.MessageFactory[180] = func() mavlink.Message { return new(MidLvlCmds) }
	mavlink.MessageFactory[181] = func() mavlink.Message { return new(CtrlSrfcPt) }
	mavlink.MessageFactory[184] = func() mavlink.Message { return new(SlugsCameraOrder) }
	mavlink.MessageFactory[185] = func() mavlink.Message { return new(ControlSurface) }
	mavlink.MessageFactory[186] = func() mavlink.Message { return new(SlugsMobileLocation) }
	mavlink.MessageFactory[188] = func() mavlink.Message { return new(SlugsConfigurationCamera) }
	mavlink.MessageFactory[189] = func() mavlink.Message { return new(IsrLocation) }
	mavlink.MessageFactory[191] = func() mavlink.Message { return new(VoltSensor) }
	mavlink.MessageFactory[192] = func() mavlink.Message { return new(PtzStatus) }
	mavlink.MessageFactory[193] = func() mavlink.Message { return new(UavStatus) }
	mavlink.MessageFactory[194] = func() mavlink.Message { return new(StatusGps) }
	mavlink.MessageFactory[195] = func() mavlink.Message { return new(NovatelDiag) }
	mavlink.MessageFactory[196] = func() mavlink.Message { return new(SensorDiag) }
	mavlink.MessageFactory[197] = func() mavlink.Message { return new(Boot) }
}

// MessageNameIDMap returns a map from message name to message ID.
func MessageNameIDMap() map[string]int {
	return map[string]int{
		"CPU_LOAD":                   170,
		"SENSOR_BIAS":                172,
		"DIAGNOSTIC":                 173,
		"SLUGS_NAVIGATION":           176,
		"DATA_LOG":                   177,
		"GPS_DATE_TIME":              179,
		"MID_LVL_CMDS":               180,
		"CTRL_SRFC_PT":               181,
		"SLUGS_CAMERA_ORDER":         184,
		"CONTROL_SURFACE":            185,
		"SLUGS_MOBILE_LOCATION":      186,
		"SLUGS_CONFIGURATION_CAMERA": 188,
		"ISR_LOCATION":               189,
		"VOLT_SENSOR":                191,
		"PTZ_STATUS":                 192,
		"UAV_STATUS":                 193,
		"STATUS_GPS":                 194,
		"NOVATEL_DIAG":               195,
		"SENSOR_DIAG":                196,
		"BOOT":                       197,
	}
}

////////////////////////////////////////////////////////////////////////////////
// Enums
////////////////////////////////////////////////////////////////////////////////

// MAV_CMD:
const (
	MAV_CMD_DO_NOTHING             = 0 // Does nothing.
	MAV_CMD_RETURN_TO_BASE         = 0 // Return vehicle to base.
	MAV_CMD_STOP_RETURN_TO_BASE    = 0 // Stops the vehicle from returning to base and resumes flight.
	MAV_CMD_TURN_LIGHT             = 0 // Turns the vehicle's visible or infrared lights on or off.
	MAV_CMD_GET_MID_LEVEL_COMMANDS = 0 // Requests vehicle to send current mid-level commands to ground station.
	MAV_CMD_MIDLEVEL_STORAGE       = 0 // Requests storage of mid-level commands.
)

// SLUGS_MODE: Slugs-specific navigation modes.
const (
	SLUGS_MODE_NONE                  = 0  // No change to SLUGS mode.
	SLUGS_MODE_LIFTOFF               = 1  // Vehicle is in liftoff mode.
	SLUGS_MODE_PASSTHROUGH           = 2  // Vehicle is in passthrough mode, being controlled by a pilot.
	SLUGS_MODE_WAYPOINT              = 3  // Vehicle is in waypoint mode, navigating to waypoints.
	SLUGS_MODE_MID_LEVEL             = 4  // Vehicle is executing mid-level commands.
	SLUGS_MODE_RETURNING             = 5  // Vehicle is returning to the home location.
	SLUGS_MODE_LANDING               = 6  // Vehicle is landing.
	SLUGS_MODE_LOST                  = 7  // Lost connection with vehicle.
	SLUGS_MODE_SELECTIVE_PASSTHROUGH = 8  // Vehicle is in selective passthrough mode, where selected surfaces are being manually controlled.
	SLUGS_MODE_ISR                   = 9  // Vehicle is in ISR mode, performing reconaissance at a point specified by ISR_LOCATION message.
	SLUGS_MODE_LINE_PATROL           = 10 // Vehicle is patrolling along lines between waypoints.
	SLUGS_MODE_GROUNDED              = 11 // Vehicle is grounded or an error has occurred.
)

// CONTROL_SURFACE_FLAG: These flags encode the control surfaces for selective passthrough mode. If a bit is set then the pilot console
//             has control of the surface, and if not then the autopilot has control of the surface.
const (
	CONTROL_SURFACE_FLAG_THROTTLE       = 128 // 0b10000000 Throttle control passes through to pilot console.
	CONTROL_SURFACE_FLAG_LEFT_AILERON   = 64  // 0b01000000 Left aileron control passes through to pilot console.
	CONTROL_SURFACE_FLAG_RIGHT_AILERON  = 32  // 0b00100000 Right aileron control passes through to pilot console.
	CONTROL_SURFACE_FLAG_RUDDER         = 16  // 0b00010000 Rudder control passes through to pilot console.
	CONTROL_SURFACE_FLAG_LEFT_ELEVATOR  = 8   // 0b00001000 Left elevator control passes through to pilot console.
	CONTROL_SURFACE_FLAG_RIGHT_ELEVATOR = 4   // 0b00000100 Right elevator control passes through to pilot console.
	CONTROL_SURFACE_FLAG_LEFT_FLAP      = 2   // 0b00000010 Left flap control passes through to pilot console.
	CONTROL_SURFACE_FLAG_RIGHT_FLAP     = 1   // 0b00000001 Right flap control passes through to pilot console.
)

////////////////////////////////////////////////////////////////////////////////
// Messages
////////////////////////////////////////////////////////////////////////////////

// Sensor and DSC control loads.
type CpuLoad struct {
	Batvolt  uint16 // Battery Voltage in millivolts
	Sensload uint8  // Sensor DSC Load
	Ctrlload uint8  // Control DSC Load
}

func (self *CpuLoad) TypeID() uint8 {
	return 170
}

func (self *CpuLoad) TypeName() string {
	return "CPU_LOAD"
}

func (self *CpuLoad) TypeSize() uint8 {
	return 4
}

func (self *CpuLoad) TypeCRCExtra() uint8 {
	return 75
}

func (self *CpuLoad) FieldsString() string {
	return fmt.Sprintf("Batvolt=%d Sensload=%d Ctrlload=%d", self.Batvolt, self.Sensload, self.Ctrlload)
}

func (self *CpuLoad) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Accelerometer and gyro biases.
type SensorBias struct {
	Axbias float32 // Accelerometer X bias (m/s)
	Aybias float32 // Accelerometer Y bias (m/s)
	Azbias float32 // Accelerometer Z bias (m/s)
	Gxbias float32 // Gyro X bias (rad/s)
	Gybias float32 // Gyro Y bias (rad/s)
	Gzbias float32 // Gyro Z bias (rad/s)
}

func (self *SensorBias) TypeID() uint8 {
	return 172
}

func (self *SensorBias) TypeName() string {
	return "SENSOR_BIAS"
}

func (self *SensorBias) TypeSize() uint8 {
	return 24
}

func (self *SensorBias) TypeCRCExtra() uint8 {
	return 168
}

func (self *SensorBias) FieldsString() string {
	return fmt.Sprintf("Axbias=%f Aybias=%f Azbias=%f Gxbias=%f Gybias=%f Gzbias=%f", self.Axbias, self.Aybias, self.Azbias, self.Gxbias, self.Gybias, self.Gzbias)
}

func (self *SensorBias) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Configurable diagnostic messages.
type Diagnostic struct {
	Diagfl1 float32 // Diagnostic float 1
	Diagfl2 float32 // Diagnostic float 2
	Diagfl3 float32 // Diagnostic float 3
	Diagsh1 int16   // Diagnostic short 1
	Diagsh2 int16   // Diagnostic short 2
	Diagsh3 int16   // Diagnostic short 3
}

func (self *Diagnostic) TypeID() uint8 {
	return 173
}

func (self *Diagnostic) TypeName() string {
	return "DIAGNOSTIC"
}

func (self *Diagnostic) TypeSize() uint8 {
	return 18
}

func (self *Diagnostic) TypeCRCExtra() uint8 {
	return 2
}

func (self *Diagnostic) FieldsString() string {
	return fmt.Sprintf("Diagfl1=%f Diagfl2=%f Diagfl3=%f Diagsh1=%d Diagsh2=%d Diagsh3=%d", self.Diagfl1, self.Diagfl2, self.Diagfl3, self.Diagsh1, self.Diagsh2, self.Diagsh3)
}

func (self *Diagnostic) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Data used in the navigation algorithm.
type SlugsNavigation struct {
	UM        float32 // Measured Airspeed prior to the nav filter in m/s
	PhiC      float32 // Commanded Roll
	ThetaC    float32 // Commanded Pitch
	PsidotC   float32 // Commanded Turn rate
	AyBody    float32 // Y component of the body acceleration
	Totaldist float32 // Total Distance to Run on this leg of Navigation
	Dist2go   float32 // Remaining distance to Run on this leg of Navigation
	HC        uint16  // Commanded altitude in 0.1 m
	Fromwp    uint8   // Origin WP
	Towp      uint8   // Destination WP
}

func (self *SlugsNavigation) TypeID() uint8 {
	return 176
}

func (self *SlugsNavigation) TypeName() string {
	return "SLUGS_NAVIGATION"
}

func (self *SlugsNavigation) TypeSize() uint8 {
	return 32
}

func (self *SlugsNavigation) TypeCRCExtra() uint8 {
	return 228
}

func (self *SlugsNavigation) FieldsString() string {
	return fmt.Sprintf("UM=%f PhiC=%f ThetaC=%f PsidotC=%f AyBody=%f Totaldist=%f Dist2go=%f HC=%d Fromwp=%d Towp=%d", self.UM, self.PhiC, self.ThetaC, self.PsidotC, self.AyBody, self.Totaldist, self.Dist2go, self.HC, self.Fromwp, self.Towp)
}

func (self *SlugsNavigation) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Configurable data log probes to be used inside Simulink
type DataLog struct {
	Fl1 float32 // Log value 1
	Fl2 float32 // Log value 2
	Fl3 float32 // Log value 3
	Fl4 float32 // Log value 4
	Fl5 float32 // Log value 5
	Fl6 float32 // Log value 6
}

func (self *DataLog) TypeID() uint8 {
	return 177
}

func (self *DataLog) TypeName() string {
	return "DATA_LOG"
}

func (self *DataLog) TypeSize() uint8 {
	return 24
}

func (self *DataLog) TypeCRCExtra() uint8 {
	return 167
}

func (self *DataLog) FieldsString() string {
	return fmt.Sprintf("Fl1=%f Fl2=%f Fl3=%f Fl4=%f Fl5=%f Fl6=%f", self.Fl1, self.Fl2, self.Fl3, self.Fl4, self.Fl5, self.Fl6)
}

func (self *DataLog) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Pilot console PWM messges.
type GpsDateTime struct {
	Year        uint8 // Year reported by Gps
	Month       uint8 // Month reported by Gps
	Day         uint8 // Day reported by Gps
	Hour        uint8 // Hour reported by Gps
	Min         uint8 // Min reported by Gps
	Sec         uint8 // Sec reported by Gps
	Clockstat   uint8 // Clock Status. See table 47 page 211 OEMStar Manual
	Vissat      uint8 // Visible satellites reported by Gps
	Usesat      uint8 // Used satellites in Solution
	Gppgl       uint8 // GPS+GLONASS satellites in Solution
	Sigusedmask uint8 // GPS and GLONASS usage mask (bit 0 GPS_used? bit_4 GLONASS_used?)
	Percentused uint8 // Percent used GPS
}

func (self *GpsDateTime) TypeID() uint8 {
	return 179
}

func (self *GpsDateTime) TypeName() string {
	return "GPS_DATE_TIME"
}

func (self *GpsDateTime) TypeSize() uint8 {
	return 12
}

func (self *GpsDateTime) TypeCRCExtra() uint8 {
	return 132
}

func (self *GpsDateTime) FieldsString() string {
	return fmt.Sprintf("Year=%d Month=%d Day=%d Hour=%d Min=%d Sec=%d Clockstat=%d Vissat=%d Usesat=%d Gppgl=%d Sigusedmask=%d Percentused=%d", self.Year, self.Month, self.Day, self.Hour, self.Min, self.Sec, self.Clockstat, self.Vissat, self.Usesat, self.Gppgl, self.Sigusedmask, self.Percentused)
}

func (self *GpsDateTime) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Mid Level commands sent from the GS to the autopilot. These are only sent when being operated in mid-level commands mode from the ground.
type MidLvlCmds struct {
	Hcommand float32 // Commanded Altitude in meters
	Ucommand float32 // Commanded Airspeed in m/s
	Rcommand float32 // Commanded Turnrate in rad/s
	Target   uint8   // The system setting the commands
}

func (self *MidLvlCmds) TypeID() uint8 {
	return 180
}

func (self *MidLvlCmds) TypeName() string {
	return "MID_LVL_CMDS"
}

func (self *MidLvlCmds) TypeSize() uint8 {
	return 13
}

func (self *MidLvlCmds) TypeCRCExtra() uint8 {
	return 146
}

func (self *MidLvlCmds) FieldsString() string {
	return fmt.Sprintf("Hcommand=%f Ucommand=%f Rcommand=%f Target=%d", self.Hcommand, self.Ucommand, self.Rcommand, self.Target)
}

func (self *MidLvlCmds) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// This message sets the control surfaces for selective passthrough mode.
type CtrlSrfcPt struct {
	Bitfieldpt uint16 // Bitfield containing the passthrough configuration, see CONTROL_SURFACE_FLAG ENUM.
	Target     uint8  // The system setting the commands
}

func (self *CtrlSrfcPt) TypeID() uint8 {
	return 181
}

func (self *CtrlSrfcPt) TypeName() string {
	return "CTRL_SRFC_PT"
}

func (self *CtrlSrfcPt) TypeSize() uint8 {
	return 3
}

func (self *CtrlSrfcPt) TypeCRCExtra() uint8 {
	return 104
}

func (self *CtrlSrfcPt) FieldsString() string {
	return fmt.Sprintf("Bitfieldpt=%d Target=%d", self.Bitfieldpt, self.Target)
}

func (self *CtrlSrfcPt) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Orders generated to the SLUGS camera mount.
type SlugsCameraOrder struct {
	Target   uint8 // The system reporting the action
	Pan      int8  // Order the mount to pan: -1 left, 0 No pan motion, +1 right
	Tilt     int8  // Order the mount to tilt: -1 down, 0 No tilt motion, +1 up
	Zoom     int8  // Order the zoom values 0 to 10
	Movehome int8  // Orders the camera mount to move home. The other fields are ignored when this field is set. 1: move home, 0 ignored
}

func (self *SlugsCameraOrder) TypeID() uint8 {
	return 184
}

func (self *SlugsCameraOrder) TypeName() string {
	return "SLUGS_CAMERA_ORDER"
}

func (self *SlugsCameraOrder) TypeSize() uint8 {
	return 5
}

func (self *SlugsCameraOrder) TypeCRCExtra() uint8 {
	return 45
}

func (self *SlugsCameraOrder) FieldsString() string {
	return fmt.Sprintf("Target=%d Pan=%d Tilt=%d Zoom=%d Movehome=%d", self.Target, self.Pan, self.Tilt, self.Zoom, self.Movehome)
}

func (self *SlugsCameraOrder) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Control for surface; pending and order to origin.
type ControlSurface struct {
	Mcontrol  float32 // Pending
	Bcontrol  float32 // Order to origin
	Target    uint8   // The system setting the commands
	Idsurface uint8   // ID control surface send 0: throttle 1: aileron 2: elevator 3: rudder
}

func (self *ControlSurface) TypeID() uint8 {
	return 185
}

func (self *ControlSurface) TypeName() string {
	return "CONTROL_SURFACE"
}

func (self *ControlSurface) TypeSize() uint8 {
	return 10
}

func (self *ControlSurface) TypeCRCExtra() uint8 {
	return 113
}

func (self *ControlSurface) FieldsString() string {
	return fmt.Sprintf("Mcontrol=%f Bcontrol=%f Target=%d Idsurface=%d", self.Mcontrol, self.Bcontrol, self.Target, self.Idsurface)
}

func (self *ControlSurface) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Transmits the last known position of the mobile GS to the UAV. Very relevant when Track Mobile is enabled
type SlugsMobileLocation struct {
	Latitude  float32 // Mobile Latitude
	Longitude float32 // Mobile Longitude
	Target    uint8   // The system reporting the action
}

func (self *SlugsMobileLocation) TypeID() uint8 {
	return 186
}

func (self *SlugsMobileLocation) TypeName() string {
	return "SLUGS_MOBILE_LOCATION"
}

func (self *SlugsMobileLocation) TypeSize() uint8 {
	return 9
}

func (self *SlugsMobileLocation) TypeCRCExtra() uint8 {
	return 101
}

func (self *SlugsMobileLocation) FieldsString() string {
	return fmt.Sprintf("Latitude=%f Longitude=%f Target=%d", self.Latitude, self.Longitude, self.Target)
}

func (self *SlugsMobileLocation) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Control for camara.
type SlugsConfigurationCamera struct {
	Target  uint8 // The system setting the commands
	Idorder uint8 // ID 0: brightness 1: aperture 2: iris 3: ICR 4: backlight
	Order   uint8 //  1: up/on 2: down/off 3: auto/reset/no action
}

func (self *SlugsConfigurationCamera) TypeID() uint8 {
	return 188
}

func (self *SlugsConfigurationCamera) TypeName() string {
	return "SLUGS_CONFIGURATION_CAMERA"
}

func (self *SlugsConfigurationCamera) TypeSize() uint8 {
	return 3
}

func (self *SlugsConfigurationCamera) TypeCRCExtra() uint8 {
	return 5
}

func (self *SlugsConfigurationCamera) FieldsString() string {
	return fmt.Sprintf("Target=%d Idorder=%d Order=%d", self.Target, self.Idorder, self.Order)
}

func (self *SlugsConfigurationCamera) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Transmits the position of watch
type IsrLocation struct {
	Latitude  float32 // ISR Latitude
	Longitude float32 // ISR Longitude
	Height    float32 // ISR Height
	Target    uint8   // The system reporting the action
	Option1   uint8   // Option 1
	Option2   uint8   // Option 2
	Option3   uint8   // Option 3
}

func (self *IsrLocation) TypeID() uint8 {
	return 189
}

func (self *IsrLocation) TypeName() string {
	return "ISR_LOCATION"
}

func (self *IsrLocation) TypeSize() uint8 {
	return 16
}

func (self *IsrLocation) TypeCRCExtra() uint8 {
	return 246
}

func (self *IsrLocation) FieldsString() string {
	return fmt.Sprintf("Latitude=%f Longitude=%f Height=%f Target=%d Option1=%d Option2=%d Option3=%d", self.Latitude, self.Longitude, self.Height, self.Target, self.Option1, self.Option2, self.Option3)
}

func (self *IsrLocation) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Transmits the readings from the voltage and current sensors
type VoltSensor struct {
	Voltage  uint16 // Voltage in uS of PWM. 0 uS = 0V, 20 uS = 21.5V
	Reading2 uint16 // Depends on the value of r2Type (0) Current consumption in uS of PWM, 20 uS = 90Amp (1) Distance in cm (2) Distance in cm (3) Absolute value
	R2type   uint8  // It is the value of reading 2: 0 - Current, 1 - Foreward Sonar, 2 - Back Sonar, 3 - RPM
}

func (self *VoltSensor) TypeID() uint8 {
	return 191
}

func (self *VoltSensor) TypeName() string {
	return "VOLT_SENSOR"
}

func (self *VoltSensor) TypeSize() uint8 {
	return 5
}

func (self *VoltSensor) TypeCRCExtra() uint8 {
	return 17
}

func (self *VoltSensor) FieldsString() string {
	return fmt.Sprintf("Voltage=%d Reading2=%d R2type=%d", self.Voltage, self.Reading2, self.R2type)
}

func (self *VoltSensor) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Transmits the actual Pan, Tilt and Zoom values of the camera unit
type PtzStatus struct {
	Pan  int16 // The Pan value in 10ths of degree
	Tilt int16 // The Tilt value in 10ths of degree
	Zoom uint8 // The actual Zoom Value
}

func (self *PtzStatus) TypeID() uint8 {
	return 192
}

func (self *PtzStatus) TypeName() string {
	return "PTZ_STATUS"
}

func (self *PtzStatus) TypeSize() uint8 {
	return 5
}

func (self *PtzStatus) TypeCRCExtra() uint8 {
	return 187
}

func (self *PtzStatus) FieldsString() string {
	return fmt.Sprintf("Pan=%d Tilt=%d Zoom=%d", self.Pan, self.Tilt, self.Zoom)
}

func (self *PtzStatus) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Transmits the actual status values UAV in flight
type UavStatus struct {
	Latitude  float32 // Latitude UAV
	Longitude float32 // Longitude UAV
	Altitude  float32 // Altitude UAV
	Speed     float32 // Speed UAV
	Course    float32 // Course UAV
	Target    uint8   // The ID system reporting the action
}

func (self *UavStatus) TypeID() uint8 {
	return 193
}

func (self *UavStatus) TypeName() string {
	return "UAV_STATUS"
}

func (self *UavStatus) TypeSize() uint8 {
	return 21
}

func (self *UavStatus) TypeCRCExtra() uint8 {
	return 160
}

func (self *UavStatus) FieldsString() string {
	return fmt.Sprintf("Latitude=%f Longitude=%f Altitude=%f Speed=%f Course=%f Target=%d", self.Latitude, self.Longitude, self.Altitude, self.Speed, self.Course, self.Target)
}

func (self *UavStatus) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// This contains the status of the GPS readings
type StatusGps struct {
	Magvar     float32 // Magnetic variation, degrees
	Csfails    uint16  // Number of times checksum has failed
	Gpsquality uint8   // The quality indicator, 0=fix not available or invalid, 1=GPS fix, 2=C/A differential GPS, 6=Dead reckoning mode, 7=Manual input mode (fixed position), 8=Simulator mode, 9= WAAS a
	Msgstype   uint8   //  Indicates if GN, GL or GP messages are being received
	Posstatus  uint8   //  A = data valid, V = data invalid
	Magdir     int8    //  Magnetic variation direction E/W. Easterly variation (E) subtracts from True course and Westerly variation (W) adds to True course
	Modeind    uint8   //  Positioning system mode indicator. A - Autonomous;D-Differential; E-Estimated (dead reckoning) mode;M-Manual input; N-Data not valid
}

func (self *StatusGps) TypeID() uint8 {
	return 194
}

func (self *StatusGps) TypeName() string {
	return "STATUS_GPS"
}

func (self *StatusGps) TypeSize() uint8 {
	return 11
}

func (self *StatusGps) TypeCRCExtra() uint8 {
	return 51
}

func (self *StatusGps) FieldsString() string {
	return fmt.Sprintf("Magvar=%f Csfails=%d Gpsquality=%d Msgstype=%d Posstatus=%d Magdir=%d Modeind=%d", self.Magvar, self.Csfails, self.Gpsquality, self.Msgstype, self.Posstatus, self.Magdir, self.Modeind)
}

func (self *StatusGps) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Transmits the diagnostics data from the Novatel OEMStar GPS
type NovatelDiag struct {
	Receiverstatus uint32  // Status Bitfield. See table 69 page 350 Novatel OEMstar Manual
	Possolage      float32 // Age of the position solution in seconds
	Csfails        uint16  // Times the CRC has failed since boot
	Timestatus     uint8   // The Time Status. See Table 8 page 27 Novatel OEMStar Manual
	Solstatus      uint8   // solution Status. See table 44 page 197
	Postype        uint8   // position type. See table 43 page 196
	Veltype        uint8   // velocity type. See table 43 page 196
}

func (self *NovatelDiag) TypeID() uint8 {
	return 195
}

func (self *NovatelDiag) TypeName() string {
	return "NOVATEL_DIAG"
}

func (self *NovatelDiag) TypeSize() uint8 {
	return 14
}

func (self *NovatelDiag) TypeCRCExtra() uint8 {
	return 59
}

func (self *NovatelDiag) FieldsString() string {
	return fmt.Sprintf("Receiverstatus=%d Possolage=%f Csfails=%d Timestatus=%d Solstatus=%d Postype=%d Veltype=%d", self.Receiverstatus, self.Possolage, self.Csfails, self.Timestatus, self.Solstatus, self.Postype, self.Veltype)
}

func (self *NovatelDiag) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Diagnostic data Sensor MCU
type SensorDiag struct {
	Float1 float32 // Float field 1
	Float2 float32 // Float field 2
	Int1   int16   // Int 16 field 1
	Char1  int8    // Int 8 field 1
}

func (self *SensorDiag) TypeID() uint8 {
	return 196
}

func (self *SensorDiag) TypeName() string {
	return "SENSOR_DIAG"
}

func (self *SensorDiag) TypeSize() uint8 {
	return 11
}

func (self *SensorDiag) TypeCRCExtra() uint8 {
	return 129
}

func (self *SensorDiag) FieldsString() string {
	return fmt.Sprintf("Float1=%f Float2=%f Int1=%d Char1=%d", self.Float1, self.Float2, self.Int1, self.Char1)
}

func (self *SensorDiag) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// The boot message indicates that a system is starting. The onboard software version allows to keep track of onboard soft/firmware revisions. This message allows the sensor and control MCUs to communicate version numbers on startup.
type Boot struct {
	Version uint32 // The onboard software version
}

func (self *Boot) TypeID() uint8 {
	return 197
}

func (self *Boot) TypeName() string {
	return "BOOT"
}

func (self *Boot) TypeSize() uint8 {
	return 4
}

func (self *Boot) TypeCRCExtra() uint8 {
	return 39
}

func (self *Boot) FieldsString() string {
	return fmt.Sprintf("Version=%d", self.Version)
}

func (self *Boot) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}
