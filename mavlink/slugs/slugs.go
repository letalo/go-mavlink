package slugs

import (
	"github.com/SpaceLeap/go-mavlink/mavlink"
	"github.com/SpaceLeap/go-mavlink/mavlink/common"
)

const (
	PROTOCOL_NAME    = "slugs"
	PROTOCOL_VERSION = 0
	PROTOCOL_INCLUDE = common.PROTOCOL_NAME
)

func init() {
	mavlink.ProtocolName = PROTOCOL_NAME
	mavlink.ProtocolVersion = PROTOCOL_VERSION

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
	Ctrlload uint8  // Control DSC Load
	Sensload uint8  // Sensor DSC Load
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
	return 183
}

// Accelerometer and gyro biases.
type SensorBias struct {
	Gzbias float32 // Gyro Z bias (rad/s)
	Gybias float32 // Gyro Y bias (rad/s)
	Gxbias float32 // Gyro X bias (rad/s)
	Azbias float32 // Accelerometer Z bias (m/s)
	Aybias float32 // Accelerometer Y bias (m/s)
	Axbias float32 // Accelerometer X bias (m/s)
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
	return 213
}

// Configurable diagnostic messages.
type Diagnostic struct {
	Diagfl3 float32 // Diagnostic float 3
	Diagfl2 float32 // Diagnostic float 2
	Diagfl1 float32 // Diagnostic float 1
	Diagsh3 int16   // Diagnostic short 3
	Diagsh2 int16   // Diagnostic short 2
	Diagsh1 int16   // Diagnostic short 1
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
	return 236
}

// Data used in the navigation algorithm.
type SlugsNavigation struct {
	Dist2go   float32 // Remaining distance to Run on this leg of Navigation
	Totaldist float32 // Total Distance to Run on this leg of Navigation
	AyBody    float32 // Y component of the body acceleration
	PsidotC   float32 // Commanded Turn rate
	ThetaC    float32 // Commanded Pitch
	PhiC      float32 // Commanded Roll
	UM        float32 // Measured Airspeed prior to the nav filter in m/s
	HC        uint16  // Commanded altitude in 0.1 m
	Towp      uint8   // Destination WP
	Fromwp    uint8   // Origin WP
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
	return 249
}

// Configurable data log probes to be used inside Simulink
type DataLog struct {
	Fl6 float32 // Log value 6
	Fl5 float32 // Log value 5
	Fl4 float32 // Log value 4
	Fl3 float32 // Log value 3
	Fl2 float32 // Log value 2
	Fl1 float32 // Log value 1
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
	return 224
}

// Pilot console PWM messges.
type GpsDateTime struct {
	Percentused uint8 // Percent used GPS
	Sigusedmask uint8 // GPS and GLONASS usage mask (bit 0 GPS_used? bit_4 GLONASS_used?)
	Gppgl       uint8 // GPS+GLONASS satellites in Solution
	Usesat      uint8 // Used satellites in Solution
	Vissat      uint8 // Visible satellites reported by Gps
	Clockstat   uint8 // Clock Status. See table 47 page 211 OEMStar Manual
	Sec         uint8 // Sec reported by Gps
	Min         uint8 // Min reported by Gps
	Hour        uint8 // Hour reported by Gps
	Day         uint8 // Day reported by Gps
	Month       uint8 // Month reported by Gps
	Year        uint8 // Year reported by Gps
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
	return 33
}

// Mid Level commands sent from the GS to the autopilot. These are only sent when being operated in mid-level commands mode from the ground.
type MidLvlCmds struct {
	Rcommand float32 // Commanded Turnrate in rad/s
	Ucommand float32 // Commanded Airspeed in m/s
	Hcommand float32 // Commanded Altitude in meters
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
	return 117
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

// Orders generated to the SLUGS camera mount.
type SlugsCameraOrder struct {
	Movehome int8  // Orders the camera mount to move home. The other fields are ignored when this field is set. 1: move home, 0 ignored
	Zoom     int8  // Order the zoom values 0 to 10
	Tilt     int8  // Order the mount to tilt: -1 down, 0 No tilt motion, +1 up
	Pan      int8  // Order the mount to pan: -1 left, 0 No pan motion, +1 right
	Target   uint8 // The system reporting the action
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
	return 201
}

// Control for surface; pending and order to origin.
type ControlSurface struct {
	Bcontrol  float32 // Order to origin
	Mcontrol  float32 // Pending
	Idsurface uint8   // ID control surface send 0: throttle 1: aileron 2: elevator 3: rudder
	Target    uint8   // The system setting the commands
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
	return 43
}

// Transmits the last known position of the mobile GS to the UAV. Very relevant when Track Mobile is enabled
type SlugsMobileLocation struct {
	Longitude float32 // Mobile Longitude
	Latitude  float32 // Mobile Latitude
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
	return 20
}

// Control for camara.
type SlugsConfigurationCamera struct {
	Order   uint8 //  1: up/on 2: down/off 3: auto/reset/no action
	Idorder uint8 // ID 0: brightness 1: aperture 2: iris 3: ICR 4: backlight
	Target  uint8 // The system setting the commands
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
	return 72
}

// Transmits the position of watch
type IsrLocation struct {
	Height    float32 // ISR Height
	Longitude float32 // ISR Longitude
	Latitude  float32 // ISR Latitude
	Option3   uint8   // Option 3
	Option2   uint8   // Option 2
	Option1   uint8   // Option 1
	Target    uint8   // The system reporting the action
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
	return 166
}

// Transmits the readings from the voltage and current sensors
type VoltSensor struct {
	Reading2 uint16 // Depends on the value of r2Type (0) Current consumption in uS of PWM, 20 uS = 90Amp (1) Distance in cm (2) Distance in cm (3) Absolute value
	Voltage  uint16 // Voltage in uS of PWM. 0 uS = 0V, 20 uS = 21.5V
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
	return 240
}

// Transmits the actual Pan, Tilt and Zoom values of the camera unit
type PtzStatus struct {
	Tilt int16 // The Tilt value in 10ths of degree
	Pan  int16 // The Pan value in 10ths of degree
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
	return 48
}

// Transmits the actual status values UAV in flight
type UavStatus struct {
	Course    float32 // Course UAV
	Speed     float32 // Speed UAV
	Altitude  float32 // Altitude UAV
	Longitude float32 // Longitude UAV
	Latitude  float32 // Latitude UAV
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
	return 144
}

// This contains the status of the GPS readings
type StatusGps struct {
	Magvar     float32 // Magnetic variation, degrees
	Csfails    uint16  // Number of times checksum has failed
	Modeind    uint8   //  Positioning system mode indicator. A - Autonomous;D-Differential; E-Estimated (dead reckoning) mode;M-Manual input; N-Data not valid
	Magdir     int8    //  Magnetic variation direction E/W. Easterly variation (E) subtracts from True course and Westerly variation (W) adds to True course
	Posstatus  uint8   //  A = data valid, V = data invalid
	Msgstype   uint8   //  Indicates if GN, GL or GP messages are being received
	Gpsquality uint8   // The quality indicator, 0=fix not available or invalid, 1=GPS fix, 2=C/A differential GPS, 6=Dead reckoning mode, 7=Manual input mode (fixed position), 8=Simulator mode, 9= WAAS a
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
	return 63
}

// Transmits the diagnostics data from the Novatel OEMStar GPS
type NovatelDiag struct {
	Possolage      float32 // Age of the position solution in seconds
	Receiverstatus uint32  // Status Bitfield. See table 69 page 350 Novatel OEMstar Manual
	Csfails        uint16  // Times the CRC has failed since boot
	Veltype        uint8   // velocity type. See table 43 page 196
	Postype        uint8   // position type. See table 43 page 196
	Solstatus      uint8   // solution Status. See table 44 page 197
	Timestatus     uint8   // The Time Status. See Table 8 page 27 Novatel OEMStar Manual
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
	return 93
}

// Diagnostic data Sensor MCU
type SensorDiag struct {
	Float2 float32 // Float field 2
	Float1 float32 // Float field 1
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
	return 43
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
