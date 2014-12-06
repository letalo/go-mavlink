package pixhawk

import (
	"fmt"

	"github.com/SpaceLeap/go-mavlink/mavlink"
	"github.com/SpaceLeap/go-mavlink/mavlink/common"
)

const (
	PROTOCOL_NAME    = "pixhawk"
	PROTOCOL_VERSION = 0
	PROTOCOL_INCLUDE = common.PROTOCOL_NAME
)

func init() {
	mavlink.ProtocolName = PROTOCOL_NAME
	mavlink.ProtocolVersion = PROTOCOL_VERSION

	mavlink.MessageFactory[151] = func() mavlink.Message { return new(SetCamShutter) }
	mavlink.MessageFactory[152] = func() mavlink.Message { return new(ImageTriggered) }
	mavlink.MessageFactory[153] = func() mavlink.Message { return new(ImageTriggerControl) }
	mavlink.MessageFactory[154] = func() mavlink.Message { return new(ImageAvailable) }
	mavlink.MessageFactory[160] = func() mavlink.Message { return new(SetPositionControlOffset) }
	mavlink.MessageFactory[170] = func() mavlink.Message { return new(PositionControlSetpoint) }
	mavlink.MessageFactory[171] = func() mavlink.Message { return new(Marker) }
	mavlink.MessageFactory[172] = func() mavlink.Message { return new(RawAux) }
	mavlink.MessageFactory[180] = func() mavlink.Message { return new(WatchdogHeartbeat) }
	mavlink.MessageFactory[181] = func() mavlink.Message { return new(WatchdogProcessInfo) }
	mavlink.MessageFactory[182] = func() mavlink.Message { return new(WatchdogProcessStatus) }
	mavlink.MessageFactory[183] = func() mavlink.Message { return new(WatchdogCommand) }
	mavlink.MessageFactory[190] = func() mavlink.Message { return new(PatternDetected) }
	mavlink.MessageFactory[191] = func() mavlink.Message { return new(PointOfInterest) }
	mavlink.MessageFactory[192] = func() mavlink.Message { return new(PointOfInterestConnection) }
	mavlink.MessageFactory[195] = func() mavlink.Message { return new(BriefFeature) }
	mavlink.MessageFactory[200] = func() mavlink.Message { return new(AttitudeControl) }
	mavlink.MessageFactory[205] = func() mavlink.Message { return new(DetectionStats) }
	mavlink.MessageFactory[206] = func() mavlink.Message { return new(OnboardHealth) }
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

func (self *SetCamShutter) TypeID() uint8 {
	return 151
}

func (self *SetCamShutter) TypeName() string {
	return "SET_CAM_SHUTTER"
}

func (self *SetCamShutter) TypeSize() uint8 {
	return 11
}

func (self *SetCamShutter) TypeCRCExtra() uint8 {
	return 235
}

func (self *SetCamShutter) FieldsString() string {
	return fmt.Sprintf("Gain=%d Exposure=%d Interval=%d TriggerPin=%d CamMode=%d CamNo=%d", self.Gain, self.Exposure, self.Interval, self.TriggerPin, self.CamMode, self.CamNo)
}

func (self *SetCamShutter) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
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

func (self *ImageTriggered) TypeID() uint8 {
	return 152
}

func (self *ImageTriggered) TypeName() string {
	return "IMAGE_TRIGGERED"
}

func (self *ImageTriggered) TypeSize() uint8 {
	return 52
}

func (self *ImageTriggered) TypeCRCExtra() uint8 {
	return 114
}

func (self *ImageTriggered) FieldsString() string {
	return fmt.Sprintf("Timestamp=%d GroundZ=%d GroundY=%d GroundX=%d Alt=%d Lon=%d Lat=%d LocalZ=%d Yaw=%d Pitch=%d Roll=%d Seq=%d", self.Timestamp, self.GroundZ, self.GroundY, self.GroundX, self.Alt, self.Lon, self.Lat, self.LocalZ, self.Yaw, self.Pitch, self.Roll, self.Seq)
}

func (self *ImageTriggered) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

//
type ImageTriggerControl struct {
	Enable uint8 // 0 to disable, 1 to enable
}

func (self *ImageTriggerControl) TypeID() uint8 {
	return 153
}

func (self *ImageTriggerControl) TypeName() string {
	return "IMAGE_TRIGGER_CONTROL"
}

func (self *ImageTriggerControl) TypeSize() uint8 {
	return 1
}

func (self *ImageTriggerControl) TypeCRCExtra() uint8 {
	return 95
}

func (self *ImageTriggerControl) FieldsString() string {
	return fmt.Sprintf("Enable=%d", self.Enable)
}

func (self *ImageTriggerControl) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
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

func (self *ImageAvailable) TypeID() uint8 {
	return 154
}

func (self *ImageAvailable) TypeName() string {
	return "IMAGE_AVAILABLE"
}

func (self *ImageAvailable) TypeSize() uint8 {
	return 92
}

func (self *ImageAvailable) TypeCRCExtra() uint8 {
	return 16
}

func (self *ImageAvailable) FieldsString() string {
	return fmt.Sprintf("ValidUntil=%d Timestamp=%d CamId=%d GroundZ=%d GroundY=%d GroundX=%d Alt=%d Lon=%d Lat=%d LocalZ=%d Yaw=%d Pitch=%d Roll=%d Gain=%d Exposure=%d Key=%d ImgBufIndex=%d ImgSeq=%d Depth=%d Height=%d Width=%d Channels=%d CamNo=%d", self.ValidUntil, self.Timestamp, self.CamId, self.GroundZ, self.GroundY, self.GroundX, self.Alt, self.Lon, self.Lat, self.LocalZ, self.Yaw, self.Pitch, self.Roll, self.Gain, self.Exposure, self.Key, self.ImgBufIndex, self.ImgSeq, self.Depth, self.Height, self.Width, self.Channels, self.CamNo)
}

func (self *ImageAvailable) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
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

func (self *SetPositionControlOffset) TypeID() uint8 {
	return 160
}

func (self *SetPositionControlOffset) TypeName() string {
	return "SET_POSITION_CONTROL_OFFSET"
}

func (self *SetPositionControlOffset) TypeSize() uint8 {
	return 18
}

func (self *SetPositionControlOffset) TypeCRCExtra() uint8 {
	return 203
}

func (self *SetPositionControlOffset) FieldsString() string {
	return fmt.Sprintf("Yaw=%d Z=%d Y=%d X=%d TargetComponent=%d TargetSystem=%d", self.Yaw, self.Z, self.Y, self.X, self.TargetComponent, self.TargetSystem)
}

func (self *SetPositionControlOffset) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

//
type PositionControlSetpoint struct {
	Yaw float32 // yaw orientation in radians, 0 = NORTH
	Z   float32 // z position
	Y   float32 // y position
	X   float32 // x position
	Id  uint16  // ID of waypoint, 0 for plain position
}

func (self *PositionControlSetpoint) TypeID() uint8 {
	return 170
}

func (self *PositionControlSetpoint) TypeName() string {
	return "POSITION_CONTROL_SETPOINT"
}

func (self *PositionControlSetpoint) TypeSize() uint8 {
	return 18
}

func (self *PositionControlSetpoint) TypeCRCExtra() uint8 {
	return 88
}

func (self *PositionControlSetpoint) FieldsString() string {
	return fmt.Sprintf("Yaw=%d Z=%d Y=%d X=%d Id=%d", self.Yaw, self.Z, self.Y, self.X, self.Id)
}

func (self *PositionControlSetpoint) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
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

func (self *Marker) TypeID() uint8 {
	return 171
}

func (self *Marker) TypeName() string {
	return "MARKER"
}

func (self *Marker) TypeSize() uint8 {
	return 26
}

func (self *Marker) TypeCRCExtra() uint8 {
	return 72
}

func (self *Marker) FieldsString() string {
	return fmt.Sprintf("Yaw=%d Pitch=%d Roll=%d Z=%d Y=%d X=%d Id=%d", self.Yaw, self.Pitch, self.Roll, self.Z, self.Y, self.X, self.Id)
}

func (self *Marker) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
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

func (self *RawAux) TypeID() uint8 {
	return 172
}

func (self *RawAux) TypeName() string {
	return "RAW_AUX"
}

func (self *RawAux) TypeSize() uint8 {
	return 16
}

func (self *RawAux) TypeCRCExtra() uint8 {
	return 115
}

func (self *RawAux) FieldsString() string {
	return fmt.Sprintf("Baro=%d Temp=%d Vbat=%d Adc4=%d Adc3=%d Adc2=%d Adc1=%d", self.Baro, self.Temp, self.Vbat, self.Adc4, self.Adc3, self.Adc2, self.Adc1)
}

func (self *RawAux) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

//
type WatchdogHeartbeat struct {
	ProcessCount uint16 // Number of processes
	WatchdogId   uint16 // Watchdog ID
}

func (self *WatchdogHeartbeat) TypeID() uint8 {
	return 180
}

func (self *WatchdogHeartbeat) TypeName() string {
	return "WATCHDOG_HEARTBEAT"
}

func (self *WatchdogHeartbeat) TypeSize() uint8 {
	return 4
}

func (self *WatchdogHeartbeat) TypeCRCExtra() uint8 {
	return 123
}

func (self *WatchdogHeartbeat) FieldsString() string {
	return fmt.Sprintf("ProcessCount=%d WatchdogId=%d", self.ProcessCount, self.WatchdogId)
}

func (self *WatchdogHeartbeat) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

//
type WatchdogProcessInfo struct {
	Timeout    int32   // Timeout (seconds)
	ProcessId  uint16  // Process ID
	WatchdogId uint16  // Watchdog ID
	Arguments  Char147 // Process arguments
	Name       Char100 // Process name
}

func (self *WatchdogProcessInfo) TypeID() uint8 {
	return 181
}

func (self *WatchdogProcessInfo) TypeName() string {
	return "WATCHDOG_PROCESS_INFO"
}

func (self *WatchdogProcessInfo) TypeSize() uint8 {
	return 255
}

func (self *WatchdogProcessInfo) TypeCRCExtra() uint8 {
	return 133
}

func (self *WatchdogProcessInfo) FieldsString() string {
	return fmt.Sprintf("Timeout=%d ProcessId=%d WatchdogId=%d Arguments=\"%s\" Name=\"%s\"", self.Timeout, self.ProcessId, self.WatchdogId, self.Arguments, self.Name)
}

func (self *WatchdogProcessInfo) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
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

func (self *WatchdogProcessStatus) TypeID() uint8 {
	return 182
}

func (self *WatchdogProcessStatus) TypeName() string {
	return "WATCHDOG_PROCESS_STATUS"
}

func (self *WatchdogProcessStatus) TypeSize() uint8 {
	return 12
}

func (self *WatchdogProcessStatus) TypeCRCExtra() uint8 {
	return 132
}

func (self *WatchdogProcessStatus) FieldsString() string {
	return fmt.Sprintf("Pid=%d Crashes=%d ProcessId=%d WatchdogId=%d Muted=%d State=%d", self.Pid, self.Crashes, self.ProcessId, self.WatchdogId, self.Muted, self.State)
}

func (self *WatchdogProcessStatus) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

//
type WatchdogCommand struct {
	ProcessId      uint16 // Process ID
	WatchdogId     uint16 // Watchdog ID
	CommandId      uint8  // Command ID
	TargetSystemId uint8  // Target system ID
}

func (self *WatchdogCommand) TypeID() uint8 {
	return 183
}

func (self *WatchdogCommand) TypeName() string {
	return "WATCHDOG_COMMAND"
}

func (self *WatchdogCommand) TypeSize() uint8 {
	return 6
}

func (self *WatchdogCommand) TypeCRCExtra() uint8 {
	return 208
}

func (self *WatchdogCommand) FieldsString() string {
	return fmt.Sprintf("ProcessId=%d WatchdogId=%d CommandId=%d TargetSystemId=%d", self.ProcessId, self.WatchdogId, self.CommandId, self.TargetSystemId)
}

func (self *WatchdogCommand) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

//
type PatternDetected struct {
	Confidence float32 // Confidence of detection
	Detected   uint8   // Accepted as true detection, 0 no, 1 yes
	File       Char100 // Pattern file name
	Type       uint8   // 0: Pattern, 1: Letter
}

func (self *PatternDetected) TypeID() uint8 {
	return 190
}

func (self *PatternDetected) TypeName() string {
	return "PATTERN_DETECTED"
}

func (self *PatternDetected) TypeSize() uint8 {
	return 106
}

func (self *PatternDetected) TypeCRCExtra() uint8 {
	return 124
}

func (self *PatternDetected) FieldsString() string {
	return fmt.Sprintf("Confidence=%d Detected=%d File=\"%s\" Type=%d", self.Confidence, self.Detected, self.File, self.Type)
}

func (self *PatternDetected) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
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

func (self *PointOfInterest) TypeID() uint8 {
	return 191
}

func (self *PointOfInterest) TypeName() string {
	return "POINT_OF_INTEREST"
}

func (self *PointOfInterest) TypeSize() uint8 {
	return 43
}

func (self *PointOfInterest) TypeCRCExtra() uint8 {
	return 115
}

func (self *PointOfInterest) FieldsString() string {
	return fmt.Sprintf("Z=%d Y=%d X=%d Timeout=%d Name=\"%s\" CoordinateSystem=%d Color=%d Type=%d", self.Z, self.Y, self.X, self.Timeout, self.Name, self.CoordinateSystem, self.Color, self.Type)
}

func (self *PointOfInterest) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
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

func (self *PointOfInterestConnection) TypeID() uint8 {
	return 192
}

func (self *PointOfInterestConnection) TypeName() string {
	return "POINT_OF_INTEREST_CONNECTION"
}

func (self *PointOfInterestConnection) TypeSize() uint8 {
	return 55
}

func (self *PointOfInterestConnection) TypeCRCExtra() uint8 {
	return 230
}

func (self *PointOfInterestConnection) FieldsString() string {
	return fmt.Sprintf("Zp2=%d Yp2=%d Xp2=%d Zp1=%d Yp1=%d Xp1=%d Timeout=%d Name=\"%s\" CoordinateSystem=%d Color=%d Type=%d", self.Zp2, self.Yp2, self.Xp2, self.Zp1, self.Yp1, self.Xp1, self.Timeout, self.Name, self.CoordinateSystem, self.Color, self.Type)
}

func (self *PointOfInterestConnection) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
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

func (self *BriefFeature) TypeID() uint8 {
	return 195
}

func (self *BriefFeature) TypeName() string {
	return "BRIEF_FEATURE"
}

func (self *BriefFeature) TypeSize() uint8 {
	return 53
}

func (self *BriefFeature) TypeCRCExtra() uint8 {
	return 34
}

func (self *BriefFeature) FieldsString() string {
	return fmt.Sprintf("Response=%d Z=%d Y=%d X=%d Orientation=%d Size=%d Descriptor=%v OrientationAssignment=%d", self.Response, self.Z, self.Y, self.X, self.Orientation, self.Size, self.Descriptor, self.OrientationAssignment)
}

func (self *BriefFeature) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
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

func (self *AttitudeControl) TypeID() uint8 {
	return 200
}

func (self *AttitudeControl) TypeName() string {
	return "ATTITUDE_CONTROL"
}

func (self *AttitudeControl) TypeSize() uint8 {
	return 21
}

func (self *AttitudeControl) TypeCRCExtra() uint8 {
	return 64
}

func (self *AttitudeControl) FieldsString() string {
	return fmt.Sprintf("Thrust=%d Yaw=%d Pitch=%d Roll=%d ThrustManual=%d YawManual=%d PitchManual=%d RollManual=%d Target=%d", self.Thrust, self.Yaw, self.Pitch, self.Roll, self.ThrustManual, self.YawManual, self.PitchManual, self.RollManual, self.Target)
}

func (self *AttitudeControl) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
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

func (self *DetectionStats) TypeID() uint8 {
	return 205
}

func (self *DetectionStats) TypeName() string {
	return "DETECTION_STATS"
}

func (self *DetectionStats) TypeSize() uint8 {
	return 48
}

func (self *DetectionStats) TypeCRCExtra() uint8 {
	return 41
}

func (self *DetectionStats) FieldsString() string {
	return fmt.Sprintf("Fps=%d ImagesTodo=%d ImagesDone=%d BestClusterIterId=%d BestClusterId=%d BestDetectionId=%d BestAlt=%d BestLon=%d BestLat=%d BestScore=%d ClusterIters=%d Detections=%d", self.Fps, self.ImagesTodo, self.ImagesDone, self.BestClusterIterId, self.BestClusterId, self.BestDetectionId, self.BestAlt, self.BestLon, self.BestLat, self.BestScore, self.ClusterIters, self.Detections)
}

func (self *DetectionStats) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
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

func (self *OnboardHealth) TypeID() uint8 {
	return 206
}

func (self *OnboardHealth) TypeName() string {
	return "ONBOARD_HEALTH"
}

func (self *OnboardHealth) TypeSize() uint8 {
	return 39
}

func (self *OnboardHealth) TypeCRCExtra() uint8 {
	return 164
}

func (self *OnboardHealth) FieldsString() string {
	return fmt.Sprintf("NetworkLoadOut=%d NetworkLoadIn=%d Voltage=%d Temp=%d DiskTotal=%d SwapTotal=%d RamTotal=%d Uptime=%d CpuFreq=%d DiskUsage=%d DiskHealth=%d SwapUsage=%d RamUsage=%d CpuLoad=%d", self.NetworkLoadOut, self.NetworkLoadIn, self.Voltage, self.Temp, self.DiskTotal, self.SwapTotal, self.RamTotal, self.Uptime, self.CpuFreq, self.DiskUsage, self.DiskHealth, self.SwapUsage, self.RamUsage, self.CpuLoad)
}

func (self *OnboardHealth) String() string {
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

type Char26 [26]byte

func (chars *Char26) String() string {
	return string(truncateZeroTerminator(chars[:]))
}

type Char32 [32]byte

func (chars *Char32) String() string {
	return string(truncateZeroTerminator(chars[:]))
}

type Char100 [100]byte

func (chars *Char100) String() string {
	return string(truncateZeroTerminator(chars[:]))
}

type Char147 [147]byte

func (chars *Char147) String() string {
	return string(truncateZeroTerminator(chars[:]))
}
