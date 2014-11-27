package pixhawk

import (
	"github.com/SpaceLeap/go-mavlink/mavlink"
	"github.com/SpaceLeap/go-mavlink/mavlink/common"
)

const (
	PROTOCOL_NAME    = "pixhawk"
	PROTOCOL_VERSION = ""
	PROTOCOL_INCLUDE = common.PROTOCOL_NAME
)

func init() {
	mavlink.ProtocolName = PROTOCOL_NAME
	mavlink.ProtocolVersion = PROTOCOL_VERSION
	// mavlink.MessageCRSs = nil

	mavlink.NewMessage[151] = func() mavlink.Message { return new(SetCamShutter) }
	mavlink.NewMessage[152] = func() mavlink.Message { return new(ImageTriggered) }
	mavlink.NewMessage[153] = func() mavlink.Message { return new(ImageTriggerControl) }
	mavlink.NewMessage[154] = func() mavlink.Message { return new(ImageAvailable) }
	mavlink.NewMessage[160] = func() mavlink.Message { return new(SetPositionControlOffset) }
	mavlink.NewMessage[170] = func() mavlink.Message { return new(PositionControlSetpoint) }
	mavlink.NewMessage[171] = func() mavlink.Message { return new(Marker) }
	mavlink.NewMessage[172] = func() mavlink.Message { return new(RawAux) }
	mavlink.NewMessage[180] = func() mavlink.Message { return new(WatchdogHeartbeat) }
	mavlink.NewMessage[181] = func() mavlink.Message { return new(WatchdogProcessInfo) }
	mavlink.NewMessage[182] = func() mavlink.Message { return new(WatchdogProcessStatus) }
	mavlink.NewMessage[183] = func() mavlink.Message { return new(WatchdogCommand) }
	mavlink.NewMessage[190] = func() mavlink.Message { return new(PatternDetected) }
	mavlink.NewMessage[191] = func() mavlink.Message { return new(PointOfInterest) }
	mavlink.NewMessage[192] = func() mavlink.Message { return new(PointOfInterestConnection) }
	mavlink.NewMessage[195] = func() mavlink.Message { return new(BriefFeature) }
	mavlink.NewMessage[200] = func() mavlink.Message { return new(AttitudeControl) }
	mavlink.NewMessage[205] = func() mavlink.Message { return new(DetectionStats) }
	mavlink.NewMessage[206] = func() mavlink.Message { return new(OnboardHealth) }
}

////////////////////////////////////////////////////////////////////////////////
// Enums
////////////////////////////////////////////////////////////////////////////////

// DATA_TYPES: Content Types for data transmission handshake
const (
	DATA_TYPE_JPEG_IMAGE = 1 //
	DATA_TYPE_RAW_IMAGE  = 2 //
	DATA_TYPE_KINECT     = 3 //
)

// MAV_CMD:
const (
	MAV_CMD_DO_START_SEARCH  = 0 // Starts a search
	MAV_CMD_DO_FINISH_SEARCH = 0 // Starts a search
	MAV_CMD_NAV_SWEEP        = 0 // Starts a search
)

////////////////////////////////////////////////////////////////////////////////
// Messages
////////////////////////////////////////////////////////////////////////////////

//
type SetCamShutter struct {
	Gain       float32 // Camera gain
	Exposure   uint16  // Exposure time, in microseconds
	Interval   uint16  // Shutter interval, in microseconds
	TriggerPin uint8   // Trigger pin, 0-3 for PtGrey FireFly
	CamMode    uint8   // Camera mode: 0 = auto, 1 = manual
	CamNo      uint8   // Camera id
}

func (self *SetCamShutter) MsgID() uint8 {
	return 151
}

func (self *SetCamShutter) MsgName() string {
	return "SetCamShutter"
}

func (self *SetCamShutter) MsgNameUpper() string {
	return "SET_CAM_SHUTTER"
}

func (self *SetCamShutter) MsgSize() uint8 {
	return 0
}

//
type ImageTriggered struct {
	Timestamp uint64  // Timestamp
	GroundZ   float32 // Ground truth Z
	GroundY   float32 // Ground truth Y
	GroundX   float32 // Ground truth X
	Alt       float32 // Global frame altitude
	Lon       float32 // GPS Y coordinate
	Lat       float32 // GPS X coordinate
	LocalZ    float32 // Local frame Z coordinate (height over ground)
	Yaw       float32 // Yaw angle in rad
	Pitch     float32 // Pitch angle in rad
	Roll      float32 // Roll angle in rad
	Seq       uint32  // IMU seq
}

func (self *ImageTriggered) MsgID() uint8 {
	return 152
}

func (self *ImageTriggered) MsgName() string {
	return "ImageTriggered"
}

func (self *ImageTriggered) MsgNameUpper() string {
	return "IMAGE_TRIGGERED"
}

func (self *ImageTriggered) MsgSize() uint8 {
	return 0
}

//
type ImageTriggerControl struct {
	Enable uint8 // 0 to disable, 1 to enable
}

func (self *ImageTriggerControl) MsgID() uint8 {
	return 153
}

func (self *ImageTriggerControl) MsgName() string {
	return "ImageTriggerControl"
}

func (self *ImageTriggerControl) MsgNameUpper() string {
	return "IMAGE_TRIGGER_CONTROL"
}

func (self *ImageTriggerControl) MsgSize() uint8 {
	return 0
}

//
type ImageAvailable struct {
	ValidUntil  uint64  // Until which timestamp this buffer will stay valid
	Timestamp   uint64  // Timestamp
	CamId       uint64  // Camera id
	GroundZ     float32 // Ground truth Z
	GroundY     float32 // Ground truth Y
	GroundX     float32 // Ground truth X
	Alt         float32 // Global frame altitude
	Lon         float32 // GPS Y coordinate
	Lat         float32 // GPS X coordinate
	LocalZ      float32 // Local frame Z coordinate (height over ground)
	Yaw         float32 // Yaw angle in rad
	Pitch       float32 // Pitch angle in rad
	Roll        float32 // Roll angle in rad
	Gain        float32 // Camera gain
	Exposure    uint32  // Exposure time, in microseconds
	Key         uint32  // Shared memory area key
	ImgBufIndex uint32  // Position of the image in the buffer, starts with 0
	ImgSeq      uint32  // The image sequence number
	Depth       uint16  // Image depth
	Height      uint16  // Image height
	Width       uint16  // Image width
	Channels    uint8   // Image channels
	CamNo       uint8   // Camera # (starts with 0)
}

func (self *ImageAvailable) MsgID() uint8 {
	return 154
}

func (self *ImageAvailable) MsgName() string {
	return "ImageAvailable"
}

func (self *ImageAvailable) MsgNameUpper() string {
	return "IMAGE_AVAILABLE"
}

func (self *ImageAvailable) MsgSize() uint8 {
	return 0
}

// Message sent to the MAV to set a new offset from the currently controlled position
type SetPositionControlOffset struct {
	Yaw             float32 // yaw orientation offset in radians, 0 = NORTH
	Z               float32 // z position offset
	Y               float32 // y position offset
	X               float32 // x position offset
	TargetComponent uint8   // Component ID
	TargetSystem    uint8   // System ID
}

func (self *SetPositionControlOffset) MsgID() uint8 {
	return 160
}

func (self *SetPositionControlOffset) MsgName() string {
	return "SetPositionControlOffset"
}

func (self *SetPositionControlOffset) MsgNameUpper() string {
	return "SET_POSITION_CONTROL_OFFSET"
}

func (self *SetPositionControlOffset) MsgSize() uint8 {
	return 0
}

//
type PositionControlSetpoint struct {
	Yaw float32 // yaw orientation in radians, 0 = NORTH
	Z   float32 // z position
	Y   float32 // y position
	X   float32 // x position
	Id  uint16  // ID of waypoint, 0 for plain position
}

func (self *PositionControlSetpoint) MsgID() uint8 {
	return 170
}

func (self *PositionControlSetpoint) MsgName() string {
	return "PositionControlSetpoint"
}

func (self *PositionControlSetpoint) MsgNameUpper() string {
	return "POSITION_CONTROL_SETPOINT"
}

func (self *PositionControlSetpoint) MsgSize() uint8 {
	return 0
}

//
type Marker struct {
	Yaw   float32 // yaw orientation
	Pitch float32 // pitch orientation
	Roll  float32 // roll orientation
	Z     float32 // z position
	Y     float32 // y position
	X     float32 // x position
	Id    uint16  // ID
}

func (self *Marker) MsgID() uint8 {
	return 171
}

func (self *Marker) MsgName() string {
	return "Marker"
}

func (self *Marker) MsgNameUpper() string {
	return "MARKER"
}

func (self *Marker) MsgSize() uint8 {
	return 0
}

//
type RawAux struct {
	Baro int32  // Barometric pressure (hecto Pascal)
	Temp int16  // Temperature (degrees celcius)
	Vbat uint16 // Battery voltage
	Adc4 uint16 // ADC4 (J405 ADC7, LPC2148 AD1.3)
	Adc3 uint16 // ADC3 (J405 ADC6, LPC2148 AD0.1)
	Adc2 uint16 // ADC2 (J405 ADC5, LPC2148 AD0.2)
	Adc1 uint16 // ADC1 (J405 ADC3, LPC2148 AD0.6)
}

func (self *RawAux) MsgID() uint8 {
	return 172
}

func (self *RawAux) MsgName() string {
	return "RawAux"
}

func (self *RawAux) MsgNameUpper() string {
	return "RAW_AUX"
}

func (self *RawAux) MsgSize() uint8 {
	return 0
}

//
type WatchdogHeartbeat struct {
	ProcessCount uint16 // Number of processes
	WatchdogId   uint16 // Watchdog ID
}

func (self *WatchdogHeartbeat) MsgID() uint8 {
	return 180
}

func (self *WatchdogHeartbeat) MsgName() string {
	return "WatchdogHeartbeat"
}

func (self *WatchdogHeartbeat) MsgNameUpper() string {
	return "WATCHDOG_HEARTBEAT"
}

func (self *WatchdogHeartbeat) MsgSize() uint8 {
	return 0
}

//
type WatchdogProcessInfo struct {
	Timeout    int32   // Timeout (seconds)
	ProcessId  uint16  // Process ID
	WatchdogId uint16  // Watchdog ID
	Arguments  Char147 // Process arguments
	Name       Char100 // Process name
}

func (self *WatchdogProcessInfo) MsgID() uint8 {
	return 181
}

func (self *WatchdogProcessInfo) MsgName() string {
	return "WatchdogProcessInfo"
}

func (self *WatchdogProcessInfo) MsgNameUpper() string {
	return "WATCHDOG_PROCESS_INFO"
}

func (self *WatchdogProcessInfo) MsgSize() uint8 {
	return 0
}

//
type WatchdogProcessStatus struct {
	Pid        int32  // PID
	Crashes    uint16 // Number of crashes
	ProcessId  uint16 // Process ID
	WatchdogId uint16 // Watchdog ID
	Muted      uint8  // Is muted
	State      uint8  // Is running / finished / suspended / crashed
}

func (self *WatchdogProcessStatus) MsgID() uint8 {
	return 182
}

func (self *WatchdogProcessStatus) MsgName() string {
	return "WatchdogProcessStatus"
}

func (self *WatchdogProcessStatus) MsgNameUpper() string {
	return "WATCHDOG_PROCESS_STATUS"
}

func (self *WatchdogProcessStatus) MsgSize() uint8 {
	return 0
}

//
type WatchdogCommand struct {
	ProcessId      uint16 // Process ID
	WatchdogId     uint16 // Watchdog ID
	CommandId      uint8  // Command ID
	TargetSystemId uint8  // Target system ID
}

func (self *WatchdogCommand) MsgID() uint8 {
	return 183
}

func (self *WatchdogCommand) MsgName() string {
	return "WatchdogCommand"
}

func (self *WatchdogCommand) MsgNameUpper() string {
	return "WATCHDOG_COMMAND"
}

func (self *WatchdogCommand) MsgSize() uint8 {
	return 0
}

//
type PatternDetected struct {
	Confidence float32 // Confidence of detection
	Detected   uint8   // Accepted as true detection, 0 no, 1 yes
	File       Char100 // Pattern file name
	Type       uint8   // 0: Pattern, 1: Letter
}

func (self *PatternDetected) MsgID() uint8 {
	return 190
}

func (self *PatternDetected) MsgName() string {
	return "PatternDetected"
}

func (self *PatternDetected) MsgNameUpper() string {
	return "PATTERN_DETECTED"
}

func (self *PatternDetected) MsgSize() uint8 {
	return 0
}

// Notifies the operator about a point of interest (POI). This can be anything detected by the
//                 system. This generic message is intented to help interfacing to generic visualizations and to display
//                 the POI on a map.
//
type PointOfInterest struct {
	Z                float32 // Z Position
	Y                float32 // Y Position
	X                float32 // X Position
	Timeout          uint16  // 0: no timeout, >1: timeout in seconds
	Name             Char26  // POI name
	CoordinateSystem uint8   // 0: global, 1:local
	Color            uint8   // 0: blue, 1: yellow, 2: red, 3: orange, 4: green, 5: magenta
	Type             uint8   // 0: Notice, 1: Warning, 2: Critical, 3: Emergency, 4: Debug
}

func (self *PointOfInterest) MsgID() uint8 {
	return 191
}

func (self *PointOfInterest) MsgName() string {
	return "PointOfInterest"
}

func (self *PointOfInterest) MsgNameUpper() string {
	return "POINT_OF_INTEREST"
}

func (self *PointOfInterest) MsgSize() uint8 {
	return 0
}

// Notifies the operator about the connection of two point of interests (POI). This can be anything detected by the
//                 system. This generic message is intented to help interfacing to generic visualizations and to display
//                 the POI on a map.
//
type PointOfInterestConnection struct {
	Zp2              float32 // Z2 Position
	Yp2              float32 // Y2 Position
	Xp2              float32 // X2 Position
	Zp1              float32 // Z1 Position
	Yp1              float32 // Y1 Position
	Xp1              float32 // X1 Position
	Timeout          uint16  // 0: no timeout, >1: timeout in seconds
	Name             Char26  // POI connection name
	CoordinateSystem uint8   // 0: global, 1:local
	Color            uint8   // 0: blue, 1: yellow, 2: red, 3: orange, 4: green, 5: magenta
	Type             uint8   // 0: Notice, 1: Warning, 2: Critical, 3: Emergency, 4: Debug
}

func (self *PointOfInterestConnection) MsgID() uint8 {
	return 192
}

func (self *PointOfInterestConnection) MsgName() string {
	return "PointOfInterestConnection"
}

func (self *PointOfInterestConnection) MsgNameUpper() string {
	return "POINT_OF_INTEREST_CONNECTION"
}

func (self *PointOfInterestConnection) MsgSize() uint8 {
	return 0
}

//
type BriefFeature struct {
	Response              float32   // Harris operator response at this location
	Z                     float32   // z position in m
	Y                     float32   // y position in m
	X                     float32   // x position in m
	Orientation           uint16    // Orientation
	Size                  uint16    // Size in pixels
	Descriptor            [32]uint8 // Descriptor
	OrientationAssignment uint8     // Orientation assignment 0: false, 1:true
}

func (self *BriefFeature) MsgID() uint8 {
	return 195
}

func (self *BriefFeature) MsgName() string {
	return "BriefFeature"
}

func (self *BriefFeature) MsgNameUpper() string {
	return "BRIEF_FEATURE"
}

func (self *BriefFeature) MsgSize() uint8 {
	return 0
}

//
type AttitudeControl struct {
	Thrust       float32 // thrust
	Yaw          float32 // yaw
	Pitch        float32 // pitch
	Roll         float32 // roll
	ThrustManual uint8   // thrust auto:0, manual:1
	YawManual    uint8   // yaw auto:0, manual:1
	PitchManual  uint8   // pitch auto:0, manual:1
	RollManual   uint8   // roll control enabled auto:0, manual:1
	Target       uint8   // The system to be controlled
}

func (self *AttitudeControl) MsgID() uint8 {
	return 200
}

func (self *AttitudeControl) MsgName() string {
	return "AttitudeControl"
}

func (self *AttitudeControl) MsgNameUpper() string {
	return "ATTITUDE_CONTROL"
}

func (self *AttitudeControl) MsgSize() uint8 {
	return 0
}

//
type DetectionStats struct {
	Fps               float32 // Average images per seconds processed
	ImagesTodo        uint32  // Number of images still to process
	ImagesDone        uint32  // Number of images already processed
	BestClusterIterId uint32  // Best cluster ID
	BestClusterId     uint32  // Best cluster ID
	BestDetectionId   uint32  // Best detection ID
	BestAlt           int32   // Altitude of the best detection * 1E3
	BestLon           int32   // Longitude of the best detection * 1E7
	BestLat           int32   // Latitude of the best detection * 1E7
	BestScore         float32 // Best score
	ClusterIters      uint32  // Number of cluster iterations
	Detections        uint32  // Number of detections
}

func (self *DetectionStats) MsgID() uint8 {
	return 205
}

func (self *DetectionStats) MsgName() string {
	return "DetectionStats"
}

func (self *DetectionStats) MsgNameUpper() string {
	return "DETECTION_STATS"
}

func (self *DetectionStats) MsgSize() uint8 {
	return 0
}

//
type OnboardHealth struct {
	NetworkLoadOut float32 // Network load outbound in KiB/s
	NetworkLoadIn  float32 // Network load inbound KiB/s
	Voltage        float32 // Supply voltage V
	Temp           float32 // Temperature
	DiskTotal      float32 // Disk total in GiB
	SwapTotal      float32 // Swap size in GiB
	RamTotal       float32 // RAM size in GiB
	Uptime         uint32  // Uptime of system
	CpuFreq        uint16  // CPU frequency
	DiskUsage      uint8   // Disk usage in percent
	DiskHealth     int8    // Disk health (-1: N/A, 0: ERR, 1: RO, 2: RW)
	SwapUsage      uint8   // Swap usage in percent
	RamUsage       uint8   // RAM usage in percent
	CpuLoad        uint8   // CPU load in percent
}

func (self *OnboardHealth) MsgID() uint8 {
	return 206
}

func (self *OnboardHealth) MsgName() string {
	return "OnboardHealth"
}

func (self *OnboardHealth) MsgNameUpper() string {
	return "ONBOARD_HEALTH"
}

func (self *OnboardHealth) MsgSize() uint8 {
	return 0
}

////////////////////////////////////////////////////////////////////////////////
// String Helpers
////////////////////////////////////////////////////////////////////////////////

type Char26 [26]byte

func (chars *Char26) String() string {
	i := 0
	for chars[i] != 0 && i < len(chars) {
		i++
	}
	return string(chars[:i])
}

type Char100 [100]byte

func (chars *Char100) String() string {
	i := 0
	for chars[i] != 0 && i < len(chars) {
		i++
	}
	return string(chars[:i])
}

type Char147 [147]byte

func (chars *Char147) String() string {
	i := 0
	for chars[i] != 0 && i < len(chars) {
		i++
	}
	return string(chars[:i])
}
