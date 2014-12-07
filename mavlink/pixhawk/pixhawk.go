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
	Interval   uint16  // Shutter interval, in microseconds
	Exposure   uint16  // Exposure time, in microseconds
	CamNo      uint8   // Camera id
	CamMode    uint8   // Camera mode: 0 = auto, 1 = manual
	TriggerPin uint8   // Trigger pin, 0-3 for PtGrey FireFly
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
	return 108
}

func (self *SetCamShutter) FieldsString() string {
	return fmt.Sprintf("Gain=%d Interval=%d Exposure=%d CamNo=%d CamMode=%d TriggerPin=%d", self.Gain, self.Interval, self.Exposure, self.CamNo, self.CamMode, self.TriggerPin)
}

func (self *SetCamShutter) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

//
type ImageTriggered struct {
	Timestamp uint64  // Timestamp
	Seq       uint32  // IMU seq
	Roll      float32 // Roll angle in rad
	Pitch     float32 // Pitch angle in rad
	Yaw       float32 // Yaw angle in rad
	LocalZ    float32 // Local frame Z coordinate (height over ground)
	Lat       float32 // GPS X coordinate
	Lon       float32 // GPS Y coordinate
	Alt       float32 // Global frame altitude
	GroundX   float32 // Ground truth X
	GroundY   float32 // Ground truth Y
	GroundZ   float32 // Ground truth Z
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
	return 86
}

func (self *ImageTriggered) FieldsString() string {
	return fmt.Sprintf("Timestamp=%d Seq=%d Roll=%d Pitch=%d Yaw=%d LocalZ=%d Lat=%d Lon=%d Alt=%d GroundX=%d GroundY=%d GroundZ=%d", self.Timestamp, self.Seq, self.Roll, self.Pitch, self.Yaw, self.LocalZ, self.Lat, self.Lon, self.Alt, self.GroundX, self.GroundY, self.GroundZ)
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
	CamId       uint64  // Camera id
	Timestamp   uint64  // Timestamp
	ValidUntil  uint64  // Until which timestamp this buffer will stay valid
	ImgSeq      uint32  // The image sequence number
	ImgBufIndex uint32  // Position of the image in the buffer, starts with 0
	Key         uint32  // Shared memory area key
	Exposure    uint32  // Exposure time, in microseconds
	Gain        float32 // Camera gain
	Roll        float32 // Roll angle in rad
	Pitch       float32 // Pitch angle in rad
	Yaw         float32 // Yaw angle in rad
	LocalZ      float32 // Local frame Z coordinate (height over ground)
	Lat         float32 // GPS X coordinate
	Lon         float32 // GPS Y coordinate
	Alt         float32 // Global frame altitude
	GroundX     float32 // Ground truth X
	GroundY     float32 // Ground truth Y
	GroundZ     float32 // Ground truth Z
	Width       uint16  // Image width
	Height      uint16  // Image height
	Depth       uint16  // Image depth
	CamNo       uint8   // Camera # (starts with 0)
	Channels    uint8   // Image channels
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
	return 224
}

func (self *ImageAvailable) FieldsString() string {
	return fmt.Sprintf("CamId=%d Timestamp=%d ValidUntil=%d ImgSeq=%d ImgBufIndex=%d Key=%d Exposure=%d Gain=%d Roll=%d Pitch=%d Yaw=%d LocalZ=%d Lat=%d Lon=%d Alt=%d GroundX=%d GroundY=%d GroundZ=%d Width=%d Height=%d Depth=%d CamNo=%d Channels=%d", self.CamId, self.Timestamp, self.ValidUntil, self.ImgSeq, self.ImgBufIndex, self.Key, self.Exposure, self.Gain, self.Roll, self.Pitch, self.Yaw, self.LocalZ, self.Lat, self.Lon, self.Alt, self.GroundX, self.GroundY, self.GroundZ, self.Width, self.Height, self.Depth, self.CamNo, self.Channels)
}

func (self *ImageAvailable) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Message sent to the MAV to set a new offset from the currently controlled position
type SetPositionControlOffset struct {
	X               float32 // x position offset
	Y               float32 // y position offset
	Z               float32 // z position offset
	Yaw             float32 // yaw orientation offset in radians, 0 = NORTH
	TargetSystem    uint8   // System ID
	TargetComponent uint8   // Component ID
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
	return 22
}

func (self *SetPositionControlOffset) FieldsString() string {
	return fmt.Sprintf("X=%d Y=%d Z=%d Yaw=%d TargetSystem=%d TargetComponent=%d", self.X, self.Y, self.Z, self.Yaw, self.TargetSystem, self.TargetComponent)
}

func (self *SetPositionControlOffset) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

//
type PositionControlSetpoint struct {
	X   float32 // x position
	Y   float32 // y position
	Z   float32 // z position
	Yaw float32 // yaw orientation in radians, 0 = NORTH
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
	return 28
}

func (self *PositionControlSetpoint) FieldsString() string {
	return fmt.Sprintf("X=%d Y=%d Z=%d Yaw=%d Id=%d", self.X, self.Y, self.Z, self.Yaw, self.Id)
}

func (self *PositionControlSetpoint) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

//
type Marker struct {
	X     float32 // x position
	Y     float32 // y position
	Z     float32 // z position
	Roll  float32 // roll orientation
	Pitch float32 // pitch orientation
	Yaw   float32 // yaw orientation
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
	return 249
}

func (self *Marker) FieldsString() string {
	return fmt.Sprintf("X=%d Y=%d Z=%d Roll=%d Pitch=%d Yaw=%d Id=%d", self.X, self.Y, self.Z, self.Roll, self.Pitch, self.Yaw, self.Id)
}

func (self *Marker) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

//
type RawAux struct {
	Baro int32  // Barometric pressure (hecto Pascal)
	Adc1 uint16 // ADC1 (J405 ADC3, LPC2148 AD0.6)
	Adc2 uint16 // ADC2 (J405 ADC5, LPC2148 AD0.2)
	Adc3 uint16 // ADC3 (J405 ADC6, LPC2148 AD0.1)
	Adc4 uint16 // ADC4 (J405 ADC7, LPC2148 AD1.3)
	Vbat uint16 // Battery voltage
	Temp int16  // Temperature (degrees celcius)
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
	return 182
}

func (self *RawAux) FieldsString() string {
	return fmt.Sprintf("Baro=%d Adc1=%d Adc2=%d Adc3=%d Adc4=%d Vbat=%d Temp=%d", self.Baro, self.Adc1, self.Adc2, self.Adc3, self.Adc4, self.Vbat, self.Temp)
}

func (self *RawAux) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

//
type WatchdogHeartbeat struct {
	WatchdogId   uint16 // Watchdog ID
	ProcessCount uint16 // Number of processes
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
	return 153
}

func (self *WatchdogHeartbeat) FieldsString() string {
	return fmt.Sprintf("WatchdogId=%d ProcessCount=%d", self.WatchdogId, self.ProcessCount)
}

func (self *WatchdogHeartbeat) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

//
type WatchdogProcessInfo struct {
	Timeout    int32   // Timeout (seconds)
	WatchdogId uint16  // Watchdog ID
	ProcessId  uint16  // Process ID
	Name       Char100 // Process name
	Arguments  Char147 // Process arguments
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
	return 42
}

func (self *WatchdogProcessInfo) FieldsString() string {
	return fmt.Sprintf("Timeout=%d WatchdogId=%d ProcessId=%d Name=\"%s\" Arguments=\"%s\"", self.Timeout, self.WatchdogId, self.ProcessId, self.Name, self.Arguments)
}

func (self *WatchdogProcessInfo) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

//
type WatchdogProcessStatus struct {
	Pid        int32  // PID
	WatchdogId uint16 // Watchdog ID
	ProcessId  uint16 // Process ID
	Crashes    uint16 // Number of crashes
	State      uint8  // Is running / finished / suspended / crashed
	Muted      uint8  // Is muted
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
	return 29
}

func (self *WatchdogProcessStatus) FieldsString() string {
	return fmt.Sprintf("Pid=%d WatchdogId=%d ProcessId=%d Crashes=%d State=%d Muted=%d", self.Pid, self.WatchdogId, self.ProcessId, self.Crashes, self.State, self.Muted)
}

func (self *WatchdogProcessStatus) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

//
type WatchdogCommand struct {
	WatchdogId     uint16 // Watchdog ID
	ProcessId      uint16 // Process ID
	TargetSystemId uint8  // Target system ID
	CommandId      uint8  // Command ID
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
	return 162
}

func (self *WatchdogCommand) FieldsString() string {
	return fmt.Sprintf("WatchdogId=%d ProcessId=%d TargetSystemId=%d CommandId=%d", self.WatchdogId, self.ProcessId, self.TargetSystemId, self.CommandId)
}

func (self *WatchdogCommand) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

//
type PatternDetected struct {
	Confidence float32 // Confidence of detection
	Type       uint8   // 0: Pattern, 1: Letter
	File       Char100 // Pattern file name
	Detected   uint8   // Accepted as true detection, 0 no, 1 yes
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
	return 74
}

func (self *PatternDetected) FieldsString() string {
	return fmt.Sprintf("Confidence=%d Type=%d File=\"%s\" Detected=%d", self.Confidence, self.Type, self.File, self.Detected)
}

func (self *PatternDetected) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Notifies the operator about a point of interest (POI). This can be anything detected by the
//                 system. This generic message is intented to help interfacing to generic visualizations and to display
//                 the POI on a map.
//
type PointOfInterest struct {
	X                float32 // X Position
	Y                float32 // Y Position
	Z                float32 // Z Position
	Timeout          uint16  // 0: no timeout, >1: timeout in seconds
	Type             uint8   // 0: Notice, 1: Warning, 2: Critical, 3: Emergency, 4: Debug
	Color            uint8   // 0: blue, 1: yellow, 2: red, 3: orange, 4: green, 5: magenta
	CoordinateSystem uint8   // 0: global, 1:local
	Name             Char26  // POI name
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
	return 19
}

func (self *PointOfInterest) FieldsString() string {
	return fmt.Sprintf("X=%d Y=%d Z=%d Timeout=%d Type=%d Color=%d CoordinateSystem=%d Name=\"%s\"", self.X, self.Y, self.Z, self.Timeout, self.Type, self.Color, self.CoordinateSystem, self.Name)
}

func (self *PointOfInterest) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

// Notifies the operator about the connection of two point of interests (POI). This can be anything detected by the
//                 system. This generic message is intented to help interfacing to generic visualizations and to display
//                 the POI on a map.
//
type PointOfInterestConnection struct {
	Xp1              float32 // X1 Position
	Yp1              float32 // Y1 Position
	Zp1              float32 // Z1 Position
	Xp2              float32 // X2 Position
	Yp2              float32 // Y2 Position
	Zp2              float32 // Z2 Position
	Timeout          uint16  // 0: no timeout, >1: timeout in seconds
	Type             uint8   // 0: Notice, 1: Warning, 2: Critical, 3: Emergency, 4: Debug
	Color            uint8   // 0: blue, 1: yellow, 2: red, 3: orange, 4: green, 5: magenta
	CoordinateSystem uint8   // 0: global, 1:local
	Name             Char26  // POI connection name
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
	return 236
}

func (self *PointOfInterestConnection) FieldsString() string {
	return fmt.Sprintf("Xp1=%d Yp1=%d Zp1=%d Xp2=%d Yp2=%d Zp2=%d Timeout=%d Type=%d Color=%d CoordinateSystem=%d Name=\"%s\"", self.Xp1, self.Yp1, self.Zp1, self.Xp2, self.Yp2, self.Zp2, self.Timeout, self.Type, self.Color, self.CoordinateSystem, self.Name)
}

func (self *PointOfInterestConnection) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

//
type BriefFeature struct {
	X                     float32   // x position in m
	Y                     float32   // y position in m
	Z                     float32   // z position in m
	Response              float32   // Harris operator response at this location
	Size                  uint16    // Size in pixels
	Orientation           uint16    // Orientation
	OrientationAssignment uint8     // Orientation assignment 0: false, 1:true
	Descriptor            [32]uint8 // Descriptor
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
	return 232
}

func (self *BriefFeature) FieldsString() string {
	return fmt.Sprintf("X=%d Y=%d Z=%d Response=%d Size=%d Orientation=%d OrientationAssignment=%d Descriptor=%v", self.X, self.Y, self.Z, self.Response, self.Size, self.Orientation, self.OrientationAssignment, self.Descriptor)
}

func (self *BriefFeature) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

//
type AttitudeControl struct {
	Roll         float32 // roll
	Pitch        float32 // pitch
	Yaw          float32 // yaw
	Thrust       float32 // thrust
	Target       uint8   // The system to be controlled
	RollManual   uint8   // roll control enabled auto:0, manual:1
	PitchManual  uint8   // pitch auto:0, manual:1
	YawManual    uint8   // yaw auto:0, manual:1
	ThrustManual uint8   // thrust auto:0, manual:1
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
	return 254
}

func (self *AttitudeControl) FieldsString() string {
	return fmt.Sprintf("Roll=%d Pitch=%d Yaw=%d Thrust=%d Target=%d RollManual=%d PitchManual=%d YawManual=%d ThrustManual=%d", self.Roll, self.Pitch, self.Yaw, self.Thrust, self.Target, self.RollManual, self.PitchManual, self.YawManual, self.ThrustManual)
}

func (self *AttitudeControl) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

//
type DetectionStats struct {
	Detections        uint32  // Number of detections
	ClusterIters      uint32  // Number of cluster iterations
	BestScore         float32 // Best score
	BestLat           int32   // Latitude of the best detection * 1E7
	BestLon           int32   // Longitude of the best detection * 1E7
	BestAlt           int32   // Altitude of the best detection * 1E3
	BestDetectionId   uint32  // Best detection ID
	BestClusterId     uint32  // Best cluster ID
	BestClusterIterId uint32  // Best cluster ID
	ImagesDone        uint32  // Number of images already processed
	ImagesTodo        uint32  // Number of images still to process
	Fps               float32 // Average images per seconds processed
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
	return 87
}

func (self *DetectionStats) FieldsString() string {
	return fmt.Sprintf("Detections=%d ClusterIters=%d BestScore=%d BestLat=%d BestLon=%d BestAlt=%d BestDetectionId=%d BestClusterId=%d BestClusterIterId=%d ImagesDone=%d ImagesTodo=%d Fps=%d", self.Detections, self.ClusterIters, self.BestScore, self.BestLat, self.BestLon, self.BestAlt, self.BestDetectionId, self.BestClusterId, self.BestClusterIterId, self.ImagesDone, self.ImagesTodo, self.Fps)
}

func (self *DetectionStats) String() string {
	return mavlink.NameIDFromMessage(self) + "{" + self.FieldsString() + "}"
}

//
type OnboardHealth struct {
	Uptime         uint32  // Uptime of system
	RamTotal       float32 // RAM size in GiB
	SwapTotal      float32 // Swap size in GiB
	DiskTotal      float32 // Disk total in GiB
	Temp           float32 // Temperature
	Voltage        float32 // Supply voltage V
	NetworkLoadIn  float32 // Network load inbound KiB/s
	NetworkLoadOut float32 // Network load outbound in KiB/s
	CpuFreq        uint16  // CPU frequency
	CpuLoad        uint8   // CPU load in percent
	RamUsage       uint8   // RAM usage in percent
	SwapUsage      uint8   // Swap usage in percent
	DiskHealth     int8    // Disk health (-1: N/A, 0: ERR, 1: RO, 2: RW)
	DiskUsage      uint8   // Disk usage in percent
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
	return 19
}

func (self *OnboardHealth) FieldsString() string {
	return fmt.Sprintf("Uptime=%d RamTotal=%d SwapTotal=%d DiskTotal=%d Temp=%d Voltage=%d NetworkLoadIn=%d NetworkLoadOut=%d CpuFreq=%d CpuLoad=%d RamUsage=%d SwapUsage=%d DiskHealth=%d DiskUsage=%d", self.Uptime, self.RamTotal, self.SwapTotal, self.DiskTotal, self.Temp, self.Voltage, self.NetworkLoadIn, self.NetworkLoadOut, self.CpuFreq, self.CpuLoad, self.RamUsage, self.SwapUsage, self.DiskHealth, self.DiskUsage)
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
