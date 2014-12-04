package common

import (
	"github.com/SpaceLeap/go-mavlink/mavlink"
)

const (
	PROTOCOL_NAME    = "common"
	PROTOCOL_VERSION = 3
)

func init() {
	mavlink.ProtocolName = PROTOCOL_NAME
	mavlink.ProtocolVersion = PROTOCOL_VERSION

	mavlink.MessageFactory[0] = func() mavlink.Message { return new(Heartbeat) }
	mavlink.MessageFactory[1] = func() mavlink.Message { return new(SysStatus) }
	mavlink.MessageFactory[2] = func() mavlink.Message { return new(SystemTime) }
	mavlink.MessageFactory[4] = func() mavlink.Message { return new(Ping) }
	mavlink.MessageFactory[5] = func() mavlink.Message { return new(ChangeOperatorControl) }
	mavlink.MessageFactory[6] = func() mavlink.Message { return new(ChangeOperatorControlAck) }
	mavlink.MessageFactory[7] = func() mavlink.Message { return new(AuthKey) }
	mavlink.MessageFactory[11] = func() mavlink.Message { return new(SetMode) }
	mavlink.MessageFactory[20] = func() mavlink.Message { return new(ParamRequestRead) }
	mavlink.MessageFactory[21] = func() mavlink.Message { return new(ParamRequestList) }
	mavlink.MessageFactory[22] = func() mavlink.Message { return new(ParamValue) }
	mavlink.MessageFactory[23] = func() mavlink.Message { return new(ParamSet) }
	mavlink.MessageFactory[24] = func() mavlink.Message { return new(GpsRawInt) }
	mavlink.MessageFactory[25] = func() mavlink.Message { return new(GpsStatus) }
	mavlink.MessageFactory[26] = func() mavlink.Message { return new(ScaledImu) }
	mavlink.MessageFactory[27] = func() mavlink.Message { return new(RawImu) }
	mavlink.MessageFactory[28] = func() mavlink.Message { return new(RawPressure) }
	mavlink.MessageFactory[29] = func() mavlink.Message { return new(ScaledPressure) }
	mavlink.MessageFactory[30] = func() mavlink.Message { return new(Attitude) }
	mavlink.MessageFactory[31] = func() mavlink.Message { return new(AttitudeQuaternion) }
	mavlink.MessageFactory[32] = func() mavlink.Message { return new(LocalPositionNed) }
	mavlink.MessageFactory[33] = func() mavlink.Message { return new(GlobalPositionInt) }
	mavlink.MessageFactory[34] = func() mavlink.Message { return new(RcChannelsScaled) }
	mavlink.MessageFactory[35] = func() mavlink.Message { return new(RcChannelsRaw) }
	mavlink.MessageFactory[36] = func() mavlink.Message { return new(ServoOutputRaw) }
	mavlink.MessageFactory[37] = func() mavlink.Message { return new(MissionRequestPartialList) }
	mavlink.MessageFactory[38] = func() mavlink.Message { return new(MissionWritePartialList) }
	mavlink.MessageFactory[39] = func() mavlink.Message { return new(MissionItem) }
	mavlink.MessageFactory[40] = func() mavlink.Message { return new(MissionRequest) }
	mavlink.MessageFactory[41] = func() mavlink.Message { return new(MissionSetCurrent) }
	mavlink.MessageFactory[42] = func() mavlink.Message { return new(MissionCurrent) }
	mavlink.MessageFactory[43] = func() mavlink.Message { return new(MissionRequestList) }
	mavlink.MessageFactory[44] = func() mavlink.Message { return new(MissionCount) }
	mavlink.MessageFactory[45] = func() mavlink.Message { return new(MissionClearAll) }
	mavlink.MessageFactory[46] = func() mavlink.Message { return new(MissionItemReached) }
	mavlink.MessageFactory[47] = func() mavlink.Message { return new(MissionAck) }
	mavlink.MessageFactory[48] = func() mavlink.Message { return new(SetGpsGlobalOrigin) }
	mavlink.MessageFactory[49] = func() mavlink.Message { return new(GpsGlobalOrigin) }
	mavlink.MessageFactory[54] = func() mavlink.Message { return new(SafetySetAllowedArea) }
	mavlink.MessageFactory[55] = func() mavlink.Message { return new(SafetyAllowedArea) }
	mavlink.MessageFactory[61] = func() mavlink.Message { return new(AttitudeQuaternionCov) }
	mavlink.MessageFactory[62] = func() mavlink.Message { return new(NavControllerOutput) }
	mavlink.MessageFactory[63] = func() mavlink.Message { return new(GlobalPositionIntCov) }
	mavlink.MessageFactory[64] = func() mavlink.Message { return new(LocalPositionNedCov) }
	mavlink.MessageFactory[65] = func() mavlink.Message { return new(RcChannels) }
	mavlink.MessageFactory[66] = func() mavlink.Message { return new(RequestDataStream) }
	mavlink.MessageFactory[67] = func() mavlink.Message { return new(DataStream) }
	mavlink.MessageFactory[69] = func() mavlink.Message { return new(ManualControl) }
	mavlink.MessageFactory[70] = func() mavlink.Message { return new(RcChannelsOverride) }
	mavlink.MessageFactory[73] = func() mavlink.Message { return new(MissionItemInt) }
	mavlink.MessageFactory[74] = func() mavlink.Message { return new(VfrHud) }
	mavlink.MessageFactory[75] = func() mavlink.Message { return new(CommandInt) }
	mavlink.MessageFactory[76] = func() mavlink.Message { return new(CommandLong) }
	mavlink.MessageFactory[77] = func() mavlink.Message { return new(CommandAck) }
	mavlink.MessageFactory[81] = func() mavlink.Message { return new(ManualSetpoint) }
	mavlink.MessageFactory[82] = func() mavlink.Message { return new(SetAttitudeTarget) }
	mavlink.MessageFactory[83] = func() mavlink.Message { return new(AttitudeTarget) }
	mavlink.MessageFactory[84] = func() mavlink.Message { return new(SetPositionTargetLocalNed) }
	mavlink.MessageFactory[85] = func() mavlink.Message { return new(PositionTargetLocalNed) }
	mavlink.MessageFactory[86] = func() mavlink.Message { return new(SetPositionTargetGlobalInt) }
	mavlink.MessageFactory[87] = func() mavlink.Message { return new(PositionTargetGlobalInt) }
	mavlink.MessageFactory[89] = func() mavlink.Message { return new(LocalPositionNedSystemGlobalOffset) }
	mavlink.MessageFactory[90] = func() mavlink.Message { return new(HilState) }
	mavlink.MessageFactory[91] = func() mavlink.Message { return new(HilControls) }
	mavlink.MessageFactory[92] = func() mavlink.Message { return new(HilRcInputsRaw) }
	mavlink.MessageFactory[100] = func() mavlink.Message { return new(OpticalFlow) }
	mavlink.MessageFactory[101] = func() mavlink.Message { return new(GlobalVisionPositionEstimate) }
	mavlink.MessageFactory[102] = func() mavlink.Message { return new(VisionPositionEstimate) }
	mavlink.MessageFactory[103] = func() mavlink.Message { return new(VisionSpeedEstimate) }
	mavlink.MessageFactory[104] = func() mavlink.Message { return new(ViconPositionEstimate) }
	mavlink.MessageFactory[105] = func() mavlink.Message { return new(HighresImu) }
	mavlink.MessageFactory[106] = func() mavlink.Message { return new(OpticalFlowRad) }
	mavlink.MessageFactory[107] = func() mavlink.Message { return new(HilSensor) }
	mavlink.MessageFactory[108] = func() mavlink.Message { return new(SimState) }
	mavlink.MessageFactory[109] = func() mavlink.Message { return new(RadioStatus) }
	mavlink.MessageFactory[110] = func() mavlink.Message { return new(FileTransferProtocol) }
	mavlink.MessageFactory[111] = func() mavlink.Message { return new(Timesync) }
	mavlink.MessageFactory[113] = func() mavlink.Message { return new(HilGps) }
	mavlink.MessageFactory[114] = func() mavlink.Message { return new(HilOpticalFlow) }
	mavlink.MessageFactory[115] = func() mavlink.Message { return new(HilStateQuaternion) }
	mavlink.MessageFactory[116] = func() mavlink.Message { return new(ScaledImu2) }
	mavlink.MessageFactory[117] = func() mavlink.Message { return new(LogRequestList) }
	mavlink.MessageFactory[118] = func() mavlink.Message { return new(LogEntry) }
	mavlink.MessageFactory[119] = func() mavlink.Message { return new(LogRequestData) }
	mavlink.MessageFactory[120] = func() mavlink.Message { return new(LogData) }
	mavlink.MessageFactory[121] = func() mavlink.Message { return new(LogErase) }
	mavlink.MessageFactory[122] = func() mavlink.Message { return new(LogRequestEnd) }
	mavlink.MessageFactory[123] = func() mavlink.Message { return new(GpsInjectData) }
	mavlink.MessageFactory[124] = func() mavlink.Message { return new(Gps2Raw) }
	mavlink.MessageFactory[125] = func() mavlink.Message { return new(PowerStatus) }
	mavlink.MessageFactory[126] = func() mavlink.Message { return new(SerialControl) }
	mavlink.MessageFactory[127] = func() mavlink.Message { return new(GpsRtk) }
	mavlink.MessageFactory[128] = func() mavlink.Message { return new(Gps2Rtk) }
	mavlink.MessageFactory[130] = func() mavlink.Message { return new(DataTransmissionHandshake) }
	mavlink.MessageFactory[131] = func() mavlink.Message { return new(EncapsulatedData) }
	mavlink.MessageFactory[132] = func() mavlink.Message { return new(DistanceSensor) }
	mavlink.MessageFactory[133] = func() mavlink.Message { return new(TerrainRequest) }
	mavlink.MessageFactory[134] = func() mavlink.Message { return new(TerrainData) }
	mavlink.MessageFactory[135] = func() mavlink.Message { return new(TerrainCheck) }
	mavlink.MessageFactory[136] = func() mavlink.Message { return new(TerrainReport) }
	mavlink.MessageFactory[147] = func() mavlink.Message { return new(BatteryStatus) }
	mavlink.MessageFactory[148] = func() mavlink.Message { return new(AutopilotVersion) }
	mavlink.MessageFactory[248] = func() mavlink.Message { return new(V2Extension) }
	mavlink.MessageFactory[249] = func() mavlink.Message { return new(MemoryVect) }
	mavlink.MessageFactory[250] = func() mavlink.Message { return new(DebugVect) }
	mavlink.MessageFactory[251] = func() mavlink.Message { return new(NamedValueFloat) }
	mavlink.MessageFactory[252] = func() mavlink.Message { return new(NamedValueInt) }
	mavlink.MessageFactory[253] = func() mavlink.Message { return new(Statustext) }
	mavlink.MessageFactory[254] = func() mavlink.Message { return new(Debug) }
}

////////////////////////////////////////////////////////////////////////////////
// Enums
////////////////////////////////////////////////////////////////////////////////

// MAV_AUTOPILOT: Micro air vehicle / autopilot classes. This identifies the individual model.
const (
	MAV_AUTOPILOT_GENERIC                                      = 0  // Generic autopilot, full support for everything
	MAV_AUTOPILOT_PIXHAWK                                      = 1  // PIXHAWK autopilot, http://pixhawk.ethz.ch
	MAV_AUTOPILOT_SLUGS                                        = 2  // SLUGS autopilot, http://slugsuav.soe.ucsc.edu
	MAV_AUTOPILOT_ARDUPILOTMEGA                                = 3  // ArduPilotMega / ArduCopter, http://diydrones.com
	MAV_AUTOPILOT_OPENPILOT                                    = 4  // OpenPilot, http://openpilot.org
	MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY                       = 5  // Generic autopilot only supporting simple waypoints
	MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY = 6  // Generic autopilot supporting waypoints and other simple navigation commands
	MAV_AUTOPILOT_GENERIC_MISSION_FULL                         = 7  // Generic autopilot supporting the full mission command set
	MAV_AUTOPILOT_INVALID                                      = 8  // No valid autopilot, e.g. a GCS or other MAVLink component
	MAV_AUTOPILOT_PPZ                                          = 9  // PPZ UAV - http://nongnu.org/paparazzi
	MAV_AUTOPILOT_UDB                                          = 10 // UAV Dev Board
	MAV_AUTOPILOT_FP                                           = 11 // FlexiPilot
	MAV_AUTOPILOT_PX4                                          = 12 // PX4 Autopilot - http://pixhawk.ethz.ch/px4/
	MAV_AUTOPILOT_SMACCMPILOT                                  = 13 // SMACCMPilot - http://smaccmpilot.org
	MAV_AUTOPILOT_AUTOQUAD                                     = 14 // AutoQuad -- http://autoquad.org
	MAV_AUTOPILOT_ARMAZILA                                     = 15 // Armazila -- http://armazila.com
	MAV_AUTOPILOT_AEROB                                        = 16 // Aerob -- http://aerob.ru
	MAV_AUTOPILOT_ASLUAV                                       = 17 // ASLUAV autopilot -- http://www.asl.ethz.ch
)

// MAV_TYPE:
const (
	MAV_TYPE_GENERIC            = 0  // Generic micro air vehicle.
	MAV_TYPE_FIXED_WING         = 1  // Fixed wing aircraft.
	MAV_TYPE_QUADROTOR          = 2  // Quadrotor
	MAV_TYPE_COAXIAL            = 3  // Coaxial helicopter
	MAV_TYPE_HELICOPTER         = 4  // Normal helicopter with tail rotor.
	MAV_TYPE_ANTENNA_TRACKER    = 5  // Ground installation
	MAV_TYPE_GCS                = 6  // Operator control unit / ground control station
	MAV_TYPE_AIRSHIP            = 7  // Airship, controlled
	MAV_TYPE_FREE_BALLOON       = 8  // Free balloon, uncontrolled
	MAV_TYPE_ROCKET             = 9  // Rocket
	MAV_TYPE_GROUND_ROVER       = 10 // Ground rover
	MAV_TYPE_SURFACE_BOAT       = 11 // Surface vessel, boat, ship
	MAV_TYPE_SUBMARINE          = 12 // Submarine
	MAV_TYPE_HEXAROTOR          = 13 // Hexarotor
	MAV_TYPE_OCTOROTOR          = 14 // Octorotor
	MAV_TYPE_TRICOPTER          = 15 // Octorotor
	MAV_TYPE_FLAPPING_WING      = 16 // Flapping wing
	MAV_TYPE_KITE               = 17 // Flapping wing
	MAV_TYPE_ONBOARD_CONTROLLER = 18 // Onboard companion controller
	MAV_TYPE_VTOL_DUOROTOR      = 19 // Two-rotor VTOL using control surfaces in vertical operation in addition. Tailsitter.
	MAV_TYPE_VTOL_QUADROTOR     = 20 // Quad-rotor VTOL using a V-shaped quad config in vertical operation. Tailsitter.
)

// MAV_MODE_FLAG: These flags encode the MAV mode.
const (
	MAV_MODE_FLAG_SAFETY_ARMED         = 128 // 0b10000000 MAV safety set to armed. Motors are enabled / running / can start. Ready to fly.
	MAV_MODE_FLAG_MANUAL_INPUT_ENABLED = 64  // 0b01000000 remote control input is enabled.
	MAV_MODE_FLAG_HIL_ENABLED          = 32  // 0b00100000 hardware in the loop simulation. All motors / actuators are blocked, but internal software is full operational.
	MAV_MODE_FLAG_STABILIZE_ENABLED    = 16  // 0b00010000 system stabilizes electronically its attitude (and optionally position). It needs however further control inputs to move around.
	MAV_MODE_FLAG_GUIDED_ENABLED       = 8   // 0b00001000 guided mode enabled, system flies MISSIONs / mission items.
	MAV_MODE_FLAG_AUTO_ENABLED         = 4   // 0b00000100 autonomous mode enabled, system finds its own goal positions. Guided flag can be set or not, depends on the actual implementation.
	MAV_MODE_FLAG_TEST_ENABLED         = 2   // 0b00000010 system has a test mode enabled. This flag is intended for temporary system tests and should not be used for stable implementations.
	MAV_MODE_FLAG_CUSTOM_MODE_ENABLED  = 1   // 0b00000001 Reserved for future use.
)

// MAV_MODE_FLAG_DECODE_POSITION: These values encode the bit positions of the decode position. These values can be used to read the value of a flag bit by combining the base_mode variable with AND with the flag position value. The result will be either 0 or 1, depending on if the flag is set or not.
const (
	MAV_MODE_FLAG_DECODE_POSITION_SAFETY      = 128 // First bit:  10000000
	MAV_MODE_FLAG_DECODE_POSITION_MANUAL      = 64  // Second bit: 01000000
	MAV_MODE_FLAG_DECODE_POSITION_HIL         = 32  // Third bit:  00100000
	MAV_MODE_FLAG_DECODE_POSITION_STABILIZE   = 16  // Fourth bit: 00010000
	MAV_MODE_FLAG_DECODE_POSITION_GUIDED      = 8   // Fifth bit:  00001000
	MAV_MODE_FLAG_DECODE_POSITION_AUTO        = 4   // Sixt bit:   00000100
	MAV_MODE_FLAG_DECODE_POSITION_TEST        = 2   // Seventh bit: 00000010
	MAV_MODE_FLAG_DECODE_POSITION_CUSTOM_MODE = 1   // Eighth bit: 00000001
)

// MAV_GOTO: Override command, pauses current mission execution and moves immediately to a position
const (
	MAV_GOTO_DO_HOLD                    = 0 // Hold at the current position.
	MAV_GOTO_DO_CONTINUE                = 1 // Continue with the next item in mission execution.
	MAV_GOTO_HOLD_AT_CURRENT_POSITION   = 2 // Hold at the current position of the system
	MAV_GOTO_HOLD_AT_SPECIFIED_POSITION = 3 // Hold at the position specified in the parameters of the DO_HOLD action
)

// MAV_MODE: These defines are predefined OR-combined mode flags. There is no need to use values from this enum, but it
//                simplifies the use of the mode flags. Note that manual input is enabled in all modes as a safety override.
const (
	MAV_MODE_PREFLIGHT          = 0   // System is not ready to fly, booting, calibrating, etc. No flag is set.
	MAV_MODE_STABILIZE_DISARMED = 80  // System is allowed to be active, under assisted RC control.
	MAV_MODE_STABILIZE_ARMED    = 208 // System is allowed to be active, under assisted RC control.
	MAV_MODE_MANUAL_DISARMED    = 64  // System is allowed to be active, under manual (RC) control, no stabilization
	MAV_MODE_MANUAL_ARMED       = 192 // System is allowed to be active, under manual (RC) control, no stabilization
	MAV_MODE_GUIDED_DISARMED    = 88  // System is allowed to be active, under autonomous control, manual setpoint
	MAV_MODE_GUIDED_ARMED       = 216 // System is allowed to be active, under autonomous control, manual setpoint
	MAV_MODE_AUTO_DISARMED      = 92  // System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard and not pre-programmed by MISSIONs)
	MAV_MODE_AUTO_ARMED         = 220 // System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard and not pre-programmed by MISSIONs)
	MAV_MODE_TEST_DISARMED      = 66  // UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only.
	MAV_MODE_TEST_ARMED         = 194 // UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only.
)

// MAV_STATE:
const (
	MAV_STATE_UNINIT      = 0 // Uninitialized system, state is unknown.
	MAV_STATE_BOOT        = 1 // System is booting up.
	MAV_STATE_CALIBRATING = 2 // System is calibrating and not flight-ready.
	MAV_STATE_STANDBY     = 3 // System is grounded and on standby. It can be launched any time.
	MAV_STATE_ACTIVE      = 4 // System is active and might be already airborne. Motors are engaged.
	MAV_STATE_CRITICAL    = 5 // System is in a non-normal flight mode. It can however still navigate.
	MAV_STATE_EMERGENCY   = 6 // System is in a non-normal flight mode. It lost control over parts or over the whole airframe. It is in mayday and going down.
	MAV_STATE_POWEROFF    = 7 // System just initialized its power-down sequence, will shut down now.
)

// MAV_COMPONENT:
const (
	MAV_COMP_ID_ALL            = 0   //
	MAV_COMP_ID_GPS            = 220 //
	MAV_COMP_ID_MISSIONPLANNER = 190 //
	MAV_COMP_ID_PATHPLANNER    = 195 //
	MAV_COMP_ID_MAPPER         = 180 //
	MAV_COMP_ID_CAMERA         = 100 //
	MAV_COMP_ID_IMU            = 200 //
	MAV_COMP_ID_IMU_2          = 201 //
	MAV_COMP_ID_IMU_3          = 202 //
	MAV_COMP_ID_UDP_BRIDGE     = 240 //
	MAV_COMP_ID_UART_BRIDGE    = 241 //
	MAV_COMP_ID_SYSTEM_CONTROL = 250 //
	MAV_COMP_ID_SERVO1         = 140 //
	MAV_COMP_ID_SERVO2         = 141 //
	MAV_COMP_ID_SERVO3         = 142 //
	MAV_COMP_ID_SERVO4         = 143 //
	MAV_COMP_ID_SERVO5         = 144 //
	MAV_COMP_ID_SERVO6         = 145 //
	MAV_COMP_ID_SERVO7         = 146 //
	MAV_COMP_ID_SERVO8         = 147 //
	MAV_COMP_ID_SERVO9         = 148 //
	MAV_COMP_ID_SERVO10        = 149 //
	MAV_COMP_ID_SERVO11        = 150 //
	MAV_COMP_ID_SERVO12        = 151 //
	MAV_COMP_ID_SERVO13        = 152 //
	MAV_COMP_ID_SERVO14        = 153 //
)

// MAV_SYS_STATUS_SENSOR: These encode the sensors whose status is sent as part of the SYS_STATUS message.
const (
	MAV_SYS_STATUS_SENSOR_3D_GYRO                = 1   // 0x01 3D gyro
	MAV_SYS_STATUS_SENSOR_3D_ACCEL               = 2   // 0x02 3D accelerometer
	MAV_SYS_STATUS_SENSOR_3D_MAG                 = 4   // 0x04 3D magnetometer
	MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE      = 8   // 0x08 absolute pressure
	MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE  = 16  // 0x10 differential pressure
	MAV_SYS_STATUS_SENSOR_GPS                    = 32  // 0x20 GPS
	MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW           = 64  // 0x40 optical flow
	MAV_SYS_STATUS_SENSOR_VISION_POSITION        = 128 // 0x80 computer vision position
	MAV_SYS_STATUS_SENSOR_LASER_POSITION         = 0   // 0x100 laser based position
	MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH  = 0   // 0x200 external ground truth (Vicon or Leica)
	MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL   = 0   // 0x400 3D angular rate control
	MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION = 0   // 0x800 attitude stabilization
	MAV_SYS_STATUS_SENSOR_YAW_POSITION           = 0   // 0x1000 yaw position
	MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL     = 0   // 0x2000 z/altitude control
	MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL    = 0   // 0x4000 x/y position control
	MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS          = 0   // 0x8000 motor outputs / control
	MAV_SYS_STATUS_SENSOR_RC_RECEIVER            = 0   // 0x10000 rc receiver
	MAV_SYS_STATUS_SENSOR_3D_GYRO2               = 0   // 0x20000 2nd 3D gyro
	MAV_SYS_STATUS_SENSOR_3D_ACCEL2              = 0   // 0x40000 2nd 3D accelerometer
	MAV_SYS_STATUS_SENSOR_3D_MAG2                = 0   // 0x80000 2nd 3D magnetometer
	MAV_SYS_STATUS_GEOFENCE                      = 0   // 0x100000 geofence
	MAV_SYS_STATUS_AHRS                          = 0   // 0x200000 AHRS subsystem health
	MAV_SYS_STATUS_TERRAIN                       = 0   // 0x400000 Terrain subsystem health
)

// MAV_FRAME:
const (
	MAV_FRAME_GLOBAL              = 0  // Global coordinate frame, WGS84 coordinate system. First value / x: latitude, second value / y: longitude, third value / z: positive altitude over mean sea level (MSL)
	MAV_FRAME_LOCAL_NED           = 1  // Local coordinate frame, Z-up (x: north, y: east, z: down).
	MAV_FRAME_MISSION             = 2  // NOT a coordinate frame, indicates a mission command.
	MAV_FRAME_GLOBAL_RELATIVE_ALT = 3  // Global coordinate frame, WGS84 coordinate system, relative altitude over ground with respect to the home position. First value / x: latitude, second value / y: longitude, third value / z: positive altitude with 0 being at the altitude of the home location.
	MAV_FRAME_LOCAL_ENU           = 4  // Local coordinate frame, Z-down (x: east, y: north, z: up)
	MAV_FRAME_LOCAL_OFFSET_NED    = 7  // Offset to the current local frame. Anything expressed in this frame should be added to the current local frame position.
	MAV_FRAME_BODY_NED            = 8  // Setpoint in body NED frame. This makes sense if all position control is externalized - e.g. useful to command 2 m/s^2 acceleration to the right.
	MAV_FRAME_BODY_OFFSET_NED     = 9  // Offset in body NED frame. This makes sense if adding setpoints to the current flight path, to avoid an obstacle - e.g. useful to command 2 m/s^2 acceleration to the east.
	MAV_FRAME_GLOBAL_TERRAIN_ALT  = 10 // Global coordinate frame with above terrain level altitude. WGS84 coordinate system, relative altitude over terrain with respect to the waypoint coordinate. First value / x: latitude in degrees, second value / y: longitude in degrees, third value / z: positive altitude in meters with 0 being at ground level in terrain model.
)

// MAVLINK_DATA_STREAM_TYPE:
const (
	MAVLINK_DATA_STREAM_IMG_JPEG   = 0 //
	MAVLINK_DATA_STREAM_IMG_BMP    = 1 //
	MAVLINK_DATA_STREAM_IMG_RAW8U  = 2 //
	MAVLINK_DATA_STREAM_IMG_RAW32U = 3 //
	MAVLINK_DATA_STREAM_IMG_PGM    = 4 //
	MAVLINK_DATA_STREAM_IMG_PNG    = 5 //
)

// FENCE_ACTION:
const (
	FENCE_ACTION_NONE            = 0 // Disable fenced mode
	FENCE_ACTION_GUIDED          = 1 // Switched to guided mode to return point (fence point 0)
	FENCE_ACTION_REPORT          = 2 // Report fence breach, but don't take action
	FENCE_ACTION_GUIDED_THR_PASS = 3 // Switched to guided mode to return point (fence point 0) with manual throttle control
)

// FENCE_BREACH:
const (
	FENCE_BREACH_NONE     = 0 // No last fence breach
	FENCE_BREACH_MINALT   = 1 // Breached minimum altitude
	FENCE_BREACH_MAXALT   = 2 // Breached maximum altitude
	FENCE_BREACH_BOUNDARY = 3 // Breached fence boundary
)

// MAV_MOUNT_MODE: Enumeration of possible mount operation modes
const (
	MAV_MOUNT_MODE_RETRACT           = 0 // Load and keep safe position (Roll,Pitch,Yaw) from permant memory and stop stabilization
	MAV_MOUNT_MODE_NEUTRAL           = 1 // Load and keep neutral position (Roll,Pitch,Yaw) from permanent memory.
	MAV_MOUNT_MODE_MAVLINK_TARGETING = 2 // Load neutral position and start MAVLink Roll,Pitch,Yaw control with stabilization
	MAV_MOUNT_MODE_RC_TARGETING      = 3 // Load neutral position and start RC Roll,Pitch,Yaw control with stabilization
	MAV_MOUNT_MODE_GPS_POINT         = 4 // Load neutral position and start to point to Lat,Lon,Alt
)

// MAV_CMD: Commands to be executed by the MAV. They can be executed on user request, or as part of a mission script. If the action is used in a mission, the parameter mapping to the waypoint/mission message is as follows: Param 1, Param 2, Param 3, Param 4, X: Param 5, Y:Param 6, Z:Param 7. This command list is similar what ARINC 424 is for commercial aircraft: A data format how to interpret waypoint/mission data.
const (
	MAV_CMD_NAV_WAYPOINT                 = 16  // Navigate to MISSION.
	MAV_CMD_NAV_LOITER_UNLIM             = 17  // Loiter around this MISSION an unlimited amount of time
	MAV_CMD_NAV_LOITER_TURNS             = 18  // Loiter around this MISSION for X turns
	MAV_CMD_NAV_LOITER_TIME              = 19  // Loiter around this MISSION for X seconds
	MAV_CMD_NAV_RETURN_TO_LAUNCH         = 20  // Return to launch location
	MAV_CMD_NAV_LAND                     = 21  // Land at location
	MAV_CMD_NAV_TAKEOFF                  = 22  // Takeoff from ground / hand
	MAV_CMD_NAV_ROI                      = 80  // Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras.
	MAV_CMD_NAV_PATHPLANNING             = 81  // Control autonomous path planning on the MAV.
	MAV_CMD_NAV_SPLINE_WAYPOINT          = 82  // Navigate to MISSION using a spline path.
	MAV_CMD_NAV_GUIDED_ENABLE            = 92  // hand control over to an external controller
	MAV_CMD_NAV_LAST                     = 95  // NOP - This command is only used to mark the upper limit of the NAV/ACTION commands in the enumeration
	MAV_CMD_CONDITION_DELAY              = 112 // Delay mission state machine.
	MAV_CMD_CONDITION_CHANGE_ALT         = 113 // Ascend/descend at rate.  Delay mission state machine until desired altitude reached.
	MAV_CMD_CONDITION_DISTANCE           = 114 // Delay mission state machine until within desired distance of next NAV point.
	MAV_CMD_CONDITION_YAW                = 115 // Reach a certain target angle.
	MAV_CMD_CONDITION_LAST               = 159 // NOP - This command is only used to mark the upper limit of the CONDITION commands in the enumeration
	MAV_CMD_DO_SET_MODE                  = 176 // Set system mode.
	MAV_CMD_DO_JUMP                      = 177 // Jump to the desired command in the mission list.  Repeat this action only the specified number of times
	MAV_CMD_DO_CHANGE_SPEED              = 178 // Change speed and/or throttle set points.
	MAV_CMD_DO_SET_HOME                  = 179 // Changes the home location either to the current location or a specified location.
	MAV_CMD_DO_SET_PARAMETER             = 180 // Set a system parameter.  Caution!  Use of this command requires knowledge of the numeric enumeration value of the parameter.
	MAV_CMD_DO_SET_RELAY                 = 181 // Set a relay to a condition.
	MAV_CMD_DO_REPEAT_RELAY              = 182 // Cycle a relay on and off for a desired number of cyles with a desired period.
	MAV_CMD_DO_SET_SERVO                 = 183 // Set a servo to a desired PWM value.
	MAV_CMD_DO_REPEAT_SERVO              = 184 // Cycle a between its nominal setting and a desired PWM for a desired number of cycles with a desired period.
	MAV_CMD_DO_FLIGHTTERMINATION         = 185 // Terminate flight immediately
	MAV_CMD_DO_LAND_START                = 189 // Mission command to perform a landing. This is used as a marker in a mission to tell the autopilot where a sequence of mission items that represents a landing starts. It may also be sent via a COMMAND_LONG to trigger a landing, in which case the nearest (geographically) landing sequence in the mission will be used. The Latitude/Longitude is optional, and may be set to 0/0 if not needed. If specified then it will be used to help find the closest landing sequence.
	MAV_CMD_DO_RALLY_LAND                = 190 // Mission command to perform a landing from a rally point.
	MAV_CMD_DO_GO_AROUND                 = 191 // Mission command to safely abort an autonmous landing.
	MAV_CMD_DO_CONTROL_VIDEO             = 200 // Control onboard camera system.
	MAV_CMD_DO_SET_ROI                   = 201 // Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras.
	MAV_CMD_DO_DIGICAM_CONFIGURE         = 202 // Mission command to configure an on-board camera controller system.
	MAV_CMD_DO_DIGICAM_CONTROL           = 203 // Mission command to control an on-board camera controller system.
	MAV_CMD_DO_MOUNT_CONFIGURE           = 204 // Mission command to configure a camera or antenna mount
	MAV_CMD_DO_MOUNT_CONTROL             = 205 // Mission command to control a camera or antenna mount
	MAV_CMD_DO_SET_CAM_TRIGG_DIST        = 206 // Mission command to set CAM_TRIGG_DIST for this flight
	MAV_CMD_DO_FENCE_ENABLE              = 207 // Mission command to enable the geofence
	MAV_CMD_DO_PARACHUTE                 = 208 // Mission command to trigger a parachute
	MAV_CMD_DO_INVERTED_FLIGHT           = 210 // Change to/from inverted flight
	MAV_CMD_DO_MOUNT_CONTROL_QUAT        = 220 // Mission command to control a camera or antenna mount, using a quaternion as reference.
	MAV_CMD_DO_GUIDED_MASTER             = 221 // set id of master controller
	MAV_CMD_DO_GUIDED_LIMITS             = 222 // set limits for external control
	MAV_CMD_DO_LAST                      = 240 // NOP - This command is only used to mark the upper limit of the DO commands in the enumeration
	MAV_CMD_PREFLIGHT_CALIBRATION        = 241 // Trigger calibration. This command will be only accepted if in pre-flight mode.
	MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS = 242 // Set sensor offsets. This command will be only accepted if in pre-flight mode.
	MAV_CMD_PREFLIGHT_STORAGE            = 245 // Request storage of different parameter values and logs. This command will be only accepted if in pre-flight mode.
	MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN    = 246 // Request the reboot or shutdown of system components.
	MAV_CMD_OVERRIDE_GOTO                = 252 // Hold / continue the current action
	MAV_CMD_MISSION_START                = 0   // start running a mission
	MAV_CMD_COMPONENT_ARM_DISARM         = 0   // Arms / Disarms a component
	MAV_CMD_START_RX_PAIR                = 0   // Starts receiver pairing
	MAV_CMD_IMAGE_START_CAPTURE          = 0   // Start image capture sequence
	MAV_CMD_IMAGE_STOP_CAPTURE           = 0   // Stop image capture sequence
	MAV_CMD_VIDEO_START_CAPTURE          = 0   // Starts video capture
	MAV_CMD_VIDEO_STOP_CAPTURE           = 0   // Stop the current video capture
	MAV_CMD_PANORAMA_CREATE              = 0   // Create a panorama at the current position
	MAV_CMD_PAYLOAD_PREPARE_DEPLOY       = 0   // Deploy payload on a Lat / Lon / Alt position. This includes the navigation to reach the required release position and velocity.
	MAV_CMD_PAYLOAD_CONTROL_DEPLOY       = 0   // Control the payload deployment.
)

// MAV_DATA_STREAM: Data stream IDs. A data stream is not a fixed set of messages, but rather a
//      recommendation to the autopilot software. Individual autopilots may or may not obey
//      the recommended messages.
const (
	MAV_DATA_STREAM_ALL             = 0  // Enable all data streams
	MAV_DATA_STREAM_RAW_SENSORS     = 1  // Enable IMU_RAW, GPS_RAW, GPS_STATUS packets.
	MAV_DATA_STREAM_EXTENDED_STATUS = 2  // Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS
	MAV_DATA_STREAM_RC_CHANNELS     = 3  // Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW
	MAV_DATA_STREAM_RAW_CONTROLLER  = 4  // Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT.
	MAV_DATA_STREAM_POSITION        = 6  // Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages.
	MAV_DATA_STREAM_EXTRA1          = 10 // Dependent on the autopilot
	MAV_DATA_STREAM_EXTRA2          = 11 // Dependent on the autopilot
	MAV_DATA_STREAM_EXTRA3          = 12 // Dependent on the autopilot
)

// MAV_ROI:  The ROI (region of interest) for the vehicle. This can be
//                 be used by the vehicle for camera/vehicle attitude alignment (see
//                 MAV_CMD_NAV_ROI).
const (
	MAV_ROI_NONE     = 0 // No region of interest.
	MAV_ROI_WPNEXT   = 1 // Point toward next MISSION.
	MAV_ROI_WPINDEX  = 2 // Point toward given MISSION.
	MAV_ROI_LOCATION = 3 // Point toward fixed location.
	MAV_ROI_TARGET   = 4 // Point toward of given id.
)

// MAV_CMD_ACK: ACK / NACK / ERROR values as a result of MAV_CMDs and for mission item transmission.
const (
	MAV_CMD_ACK_OK                                 = 0 // Command / mission item is ok.
	MAV_CMD_ACK_ERR_FAIL                           = 1 // Generic error message if none of the other reasons fails or if no detailed error reporting is implemented.
	MAV_CMD_ACK_ERR_ACCESS_DENIED                  = 2 // The system is refusing to accept this command from this source / communication partner.
	MAV_CMD_ACK_ERR_NOT_SUPPORTED                  = 3 // Command or mission item is not supported, other commands would be accepted.
	MAV_CMD_ACK_ERR_COORDINATE_FRAME_NOT_SUPPORTED = 4 // The coordinate frame of this command / mission item is not supported.
	MAV_CMD_ACK_ERR_COORDINATES_OUT_OF_RANGE       = 5 // The coordinate frame of this command is ok, but he coordinate values exceed the safety limits of this system. This is a generic error, please use the more specific error messages below if possible.
	MAV_CMD_ACK_ERR_X_LAT_OUT_OF_RANGE             = 6 // The X or latitude value is out of range.
	MAV_CMD_ACK_ERR_Y_LON_OUT_OF_RANGE             = 7 // The Y or longitude value is out of range.
	MAV_CMD_ACK_ERR_Z_ALT_OUT_OF_RANGE             = 8 // The Z or altitude value is out of range.
)

// MAV_PARAM_TYPE: Specifies the datatype of a MAVLink parameter.
const (
	MAV_PARAM_TYPE_UINT8  = 1  // 8-bit unsigned integer
	MAV_PARAM_TYPE_INT8   = 2  // 8-bit signed integer
	MAV_PARAM_TYPE_UINT16 = 3  // 16-bit unsigned integer
	MAV_PARAM_TYPE_INT16  = 4  // 16-bit signed integer
	MAV_PARAM_TYPE_UINT32 = 5  // 32-bit unsigned integer
	MAV_PARAM_TYPE_INT32  = 6  // 32-bit signed integer
	MAV_PARAM_TYPE_UINT64 = 7  // 64-bit unsigned integer
	MAV_PARAM_TYPE_INT64  = 8  // 64-bit signed integer
	MAV_PARAM_TYPE_REAL32 = 9  // 32-bit floating-point
	MAV_PARAM_TYPE_REAL64 = 10 // 64-bit floating-point
)

// MAV_RESULT: result from a mavlink command
const (
	MAV_RESULT_ACCEPTED             = 0 // Command ACCEPTED and EXECUTED
	MAV_RESULT_TEMPORARILY_REJECTED = 1 // Command TEMPORARY REJECTED/DENIED
	MAV_RESULT_DENIED               = 2 // Command PERMANENTLY DENIED
	MAV_RESULT_UNSUPPORTED          = 3 // Command UNKNOWN/UNSUPPORTED
	MAV_RESULT_FAILED               = 4 // Command executed, but failed
)

// MAV_MISSION_RESULT: result in a mavlink mission ack
const (
	MAV_MISSION_ACCEPTED          = 0  // mission accepted OK
	MAV_MISSION_ERROR             = 1  // generic error / not accepting mission commands at all right now
	MAV_MISSION_UNSUPPORTED_FRAME = 2  // coordinate frame is not supported
	MAV_MISSION_UNSUPPORTED       = 3  // command is not supported
	MAV_MISSION_NO_SPACE          = 4  // mission item exceeds storage space
	MAV_MISSION_INVALID           = 5  // one of the parameters has an invalid value
	MAV_MISSION_INVALID_PARAM1    = 6  // param1 has an invalid value
	MAV_MISSION_INVALID_PARAM2    = 7  // param2 has an invalid value
	MAV_MISSION_INVALID_PARAM3    = 8  // param3 has an invalid value
	MAV_MISSION_INVALID_PARAM4    = 9  // param4 has an invalid value
	MAV_MISSION_INVALID_PARAM5_X  = 10 // x/param5 has an invalid value
	MAV_MISSION_INVALID_PARAM6_Y  = 11 // y/param6 has an invalid value
	MAV_MISSION_INVALID_PARAM7    = 12 // param7 has an invalid value
	MAV_MISSION_INVALID_SEQUENCE  = 13 // received waypoint out of sequence
	MAV_MISSION_DENIED            = 14 // not accepting any mission commands from this communication partner
)

// MAV_SEVERITY: Indicates the severity level, generally used for status messages to indicate their relative urgency. Based on RFC-5424 using expanded definitions at: http://www.kiwisyslog.com/kb/info:-syslog-message-levels/.
const (
	MAV_SEVERITY_EMERGENCY = 0 // System is unusable. This is a "panic" condition.
	MAV_SEVERITY_ALERT     = 1 // Action should be taken immediately. Indicates error in non-critical systems.
	MAV_SEVERITY_CRITICAL  = 2 // Action must be taken immediately. Indicates failure in a primary system.
	MAV_SEVERITY_ERROR     = 3 // Indicates an error in secondary/redundant systems.
	MAV_SEVERITY_WARNING   = 4 // Indicates about a possible future error if this is not resolved within a given timeframe. Example would be a low battery warning.
	MAV_SEVERITY_NOTICE    = 5 // An unusual event has occured, though not an error condition. This should be investigated for the root cause.
	MAV_SEVERITY_INFO      = 6 // Normal operational messages. Useful for logging. No action is required for these messages.
	MAV_SEVERITY_DEBUG     = 7 // Useful non-operational messages that can assist in debugging. These should not occur during normal operation.
)

// MAV_POWER_STATUS: Power supply status flags (bitmask)
const (
	MAV_POWER_STATUS_BRICK_VALID                = 1  // main brick power supply valid
	MAV_POWER_STATUS_SERVO_VALID                = 2  // main servo power supply valid for FMU
	MAV_POWER_STATUS_USB_CONNECTED              = 4  // USB power is connected
	MAV_POWER_STATUS_PERIPH_OVERCURRENT         = 8  // peripheral supply is in over-current state
	MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT = 16 // hi-power peripheral supply is in over-current state
	MAV_POWER_STATUS_CHANGED                    = 32 // Power status has changed since boot
)

// SERIAL_CONTROL_DEV: SERIAL_CONTROL device types
const (
	SERIAL_CONTROL_DEV_TELEM1 = 0 // First telemetry port
	SERIAL_CONTROL_DEV_TELEM2 = 1 // Second telemetry port
	SERIAL_CONTROL_DEV_GPS1   = 2 // First GPS port
	SERIAL_CONTROL_DEV_GPS2   = 3 // Second GPS port
)

// SERIAL_CONTROL_FLAG: SERIAL_CONTROL flags (bitmask)
const (
	SERIAL_CONTROL_FLAG_REPLY     = 1  // Set if this is a reply
	SERIAL_CONTROL_FLAG_RESPOND   = 2  // Set if the sender wants the receiver to send a response as another SERIAL_CONTROL message
	SERIAL_CONTROL_FLAG_EXCLUSIVE = 4  // Set if access to the serial port should be removed from whatever driver is currently using it, giving exclusive access to the SERIAL_CONTROL protocol. The port can be handed back by sending a request without this flag set
	SERIAL_CONTROL_FLAG_BLOCKING  = 8  // Block on writes to the serial port
	SERIAL_CONTROL_FLAG_MULTI     = 16 // Send multiple replies until port is drained
)

// MAV_DISTANCE_SENSOR: Enumeration of distance sensor types
const (
	MAV_DISTANCE_SENSOR_LASER      = 0 // Laser altimeter, e.g. LightWare SF02/F or PulsedLight units
	MAV_DISTANCE_SENSOR_ULTRASOUND = 1 // Ultrasound altimeter, e.g. MaxBotix units
)

// MAV_PROTOCOL_CAPABILITY: Bitmask of (optional) autopilot capabilities (64 bit). If a bit is set, the autopilot supports this capability.
const (
	MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT                  = 1   // Autopilot supports MISSION float message type.
	MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT                    = 2   // Autopilot supports the new param float message type.
	MAV_PROTOCOL_CAPABILITY_MISSION_INT                    = 4   // Autopilot supports MISSION_INT scaled integer message type.
	MAV_PROTOCOL_CAPABILITY_COMMAND_INT                    = 8   // Autopilot supports COMMAND_INT scaled integer message type.
	MAV_PROTOCOL_CAPABILITY_PARAM_UNION                    = 16  // Autopilot supports the new param union message type.
	MAV_PROTOCOL_CAPABILITY_FTP                            = 32  // Autopilot supports the new param union message type.
	MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET            = 64  // Autopilot supports commanding attitude offboard.
	MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED  = 128 // Autopilot supports commanding position and velocity targets in local NED frame.
	MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT = 0   // Autopilot supports commanding position and velocity targets in global scaled integers.
	MAV_PROTOCOL_CAPABILITY_TERRAIN                        = 0   // Autopilot supports terrain protocol / data handling.
)

// MAV_ESTIMATOR_TYPE: Enumeration of estimator types
const (
	MAV_ESTIMATOR_TYPE_NAIVE   = 1 // This is a naive estimator without any real covariance feedback.
	MAV_ESTIMATOR_TYPE_VISION  = 2 // Computer vision based estimate. Might be up to scale.
	MAV_ESTIMATOR_TYPE_VIO     = 3 // Visual-inertial estimate.
	MAV_ESTIMATOR_TYPE_GPS     = 4 // Plain GPS estimate.
	MAV_ESTIMATOR_TYPE_GPS_INS = 5 // Estimator integrating GPS and inertial sensing.
)

// MAV_BATTERY_TYPE: Enumeration of battery types
const (
	MAV_BATTERY_TYPE_UNKNOWN = 0 // Not specified.
	MAV_BATTERY_TYPE_LIPO    = 1 // Lithium polymere battery
	MAV_BATTERY_TYPE_LIFE    = 2 // Lithium ferrite battery
	MAV_BATTERY_TYPE_LION    = 3 // Lithium-ION battery
	MAV_BATTERY_TYPE_NIMH    = 4 // Nickel metal hydride battery
)

// MAV_BATTERY_FUNCTION: Enumeration of battery functions
const (
	MAV_BATTERY_FUNCTION_UNKNOWN    = 0 // Lithium polymere battery
	MAV_BATTERY_FUNCTION_ALL        = 1 // Battery supports all flight systems
	MAV_BATTERY_FUNCTION_PROPULSION = 2 // Battery for the propulsion system
	MAV_BATTERY_FUNCTION_AVIONICS   = 3 // Avionics battery
	MAV_BATTERY_TYPE_PAYLOAD        = 4 // Payload battery
)

////////////////////////////////////////////////////////////////////////////////
// Messages
////////////////////////////////////////////////////////////////////////////////

// The heartbeat message shows that a system is present and responding. The type of the MAV and Autopilot hardware allow the receiving system to treat further messages from this system appropriate (e.g. by laying out the user interface based on the autopilot).
type Heartbeat struct {
	CustomMode     uint32 // A bitfield for use for autopilot-specific flags.
	MavlinkVersion uint8  // MAVLink version, not writable by user, gets added by protocol because of magic data type: uint8_t_mavlink_version
	SystemStatus   uint8  // System status flag, see MAV_STATE ENUM
	BaseMode       uint8  // System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.h
	Autopilot      uint8  // Autopilot type / class. defined in MAV_AUTOPILOT ENUM
	Type           uint8  // Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
}

func NewHeartbeat() *Heartbeat {
	return &Heartbeat{MavlinkVersion: PROTOCOL_VERSION}
}

func (self *Heartbeat) TypeID() uint8 {
	return 0
}

func (self *Heartbeat) TypeName() string {
	return "HEARTBEAT"
}

func (self *Heartbeat) TypeSize() uint8 {
	return 9
}

func (self *Heartbeat) TypeCRCExtra() uint8 {
	return 206
}

// The general system state. If the system is following the MAVLink standard, the system state is mainly defined by three orthogonal states/modes: The system mode, which is either LOCKED (motors shut down and locked), MANUAL (system under RC control), GUIDED (system with autonomous position control, position setpoint controlled manually) or AUTO (system guided by path/waypoint planner). The NAV_MODE defined the current flight state: LIFTOFF (often an open-loop maneuver), LANDING, WAYPOINTS or VECTOR. This represents the internal navigation state machine. The system status shows wether the system is currently active or not and if an emergency occured. During the CRITICAL and EMERGENCY states the MAV is still considered to be active, but should start emergency procedures autonomously. After a failure occured it should first move from active to critical to allow manual intervention and then move to emergency after a certain timeout.
type SysStatus struct {
	OnboardControlSensorsHealth  uint32 // Bitmask showing which onboard controllers and sensors are operational or have an error:  Value of 0: not enabled. Value of 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSOR
	OnboardControlSensorsEnabled uint32 // Bitmask showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSOR
	OnboardControlSensorsPresent uint32 // Bitmask showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present. Indices defined by ENUM MAV_SYS_STATUS_SENSOR
	ErrorsCount4                 uint16 // Autopilot-specific errors
	ErrorsCount3                 uint16 // Autopilot-specific errors
	ErrorsCount2                 uint16 // Autopilot-specific errors
	ErrorsCount1                 uint16 // Autopilot-specific errors
	ErrorsComm                   uint16 // Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
	DropRateComm                 uint16 // Communication drops in percent, (0%: 0, 100%: 10'000), (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
	CurrentBattery               int16  // Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
	VoltageBattery               uint16 // Battery voltage, in millivolts (1 = 1 millivolt)
	Load                         uint16 // Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 1000
	BatteryRemaining             int8   // Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining battery
}

func (self *SysStatus) TypeID() uint8 {
	return 1
}

func (self *SysStatus) TypeName() string {
	return "SYS_STATUS"
}

func (self *SysStatus) TypeSize() uint8 {
	return 31
}

func (self *SysStatus) TypeCRCExtra() uint8 {
	return 118
}

// The system time is the time of the master clock, typically the computer clock of the main onboard computer.
type SystemTime struct {
	TimeUnixUsec uint64 // Timestamp of the master clock in microseconds since UNIX epoch.
	TimeBootMs   uint32 // Timestamp of the component clock since boot time in milliseconds.
}

func (self *SystemTime) TypeID() uint8 {
	return 2
}

func (self *SystemTime) TypeName() string {
	return "SYSTEM_TIME"
}

func (self *SystemTime) TypeSize() uint8 {
	return 12
}

func (self *SystemTime) TypeCRCExtra() uint8 {
	return 137
}

// A ping message either requesting or responding to a ping. This allows to measure the system latencies, including serial port, radio modem and UDP connections.
type Ping struct {
	TimeUsec        uint64 // Unix timestamp in microseconds or since system boot if smaller than MAVLink epoch (1.1.2009)
	Seq             uint32 // PING sequence
	TargetComponent uint8  // 0: request ping from all receiving components, if greater than 0: message is a ping response and number is the system id of the requesting system
	TargetSystem    uint8  // 0: request ping from all receiving systems, if greater than 0: message is a ping response and number is the system id of the requesting system
}

func (self *Ping) TypeID() uint8 {
	return 4
}

func (self *Ping) TypeName() string {
	return "PING"
}

func (self *Ping) TypeSize() uint8 {
	return 14
}

func (self *Ping) TypeCRCExtra() uint8 {
	return 30
}

// Request to control this MAV
type ChangeOperatorControl struct {
	Passkey        Char25 // Password / Key, depending on version plaintext or encrypted. 25 or less characters, NULL terminated. The characters may involve A-Z, a-z, 0-9, and "!?,.-"
	Version        uint8  // 0: key as plaintext, 1-255: future, different hashing/encryption variants. The GCS should in general use the safest mode possible initially and then gradually move down the encryption level if it gets a NACK message indicating an encryption mismatch.
	ControlRequest uint8  // 0: request control of this MAV, 1: Release control of this MAV
	TargetSystem   uint8  // System the GCS requests control for
}

func (self *ChangeOperatorControl) TypeID() uint8 {
	return 5
}

func (self *ChangeOperatorControl) TypeName() string {
	return "CHANGE_OPERATOR_CONTROL"
}

func (self *ChangeOperatorControl) TypeSize() uint8 {
	return 4
}

func (self *ChangeOperatorControl) TypeCRCExtra() uint8 {
	return 136
}

// Accept / deny control of this MAV
type ChangeOperatorControlAck struct {
	Ack            uint8 // 0: ACK, 1: NACK: Wrong passkey, 2: NACK: Unsupported passkey encryption method, 3: NACK: Already under control
	ControlRequest uint8 // 0: request control of this MAV, 1: Release control of this MAV
	GcsSystemId    uint8 // ID of the GCS this message
}

func (self *ChangeOperatorControlAck) TypeID() uint8 {
	return 6
}

func (self *ChangeOperatorControlAck) TypeName() string {
	return "CHANGE_OPERATOR_CONTROL_ACK"
}

func (self *ChangeOperatorControlAck) TypeSize() uint8 {
	return 3
}

func (self *ChangeOperatorControlAck) TypeCRCExtra() uint8 {
	return 13
}

// Emit an encrypted signature / key identifying this system. PLEASE NOTE: This protocol has been kept simple, so transmitting the key requires an encrypted channel for true safety.
type AuthKey struct {
	Key Char32 // key
}

func (self *AuthKey) TypeID() uint8 {
	return 7
}

func (self *AuthKey) TypeName() string {
	return "AUTH_KEY"
}

func (self *AuthKey) TypeSize() uint8 {
	return 1
}

func (self *AuthKey) TypeCRCExtra() uint8 {
	return 152
}

// Set the system mode, as defined by enum MAV_MODE. There is no target component id as the mode is by definition for the overall aircraft, not only for one component.
type SetMode struct {
	CustomMode   uint32 // The new autopilot-specific mode. This field can be ignored by an autopilot.
	BaseMode     uint8  // The new base mode
	TargetSystem uint8  // The system setting the mode
}

func (self *SetMode) TypeID() uint8 {
	return 11
}

func (self *SetMode) TypeName() string {
	return "SET_MODE"
}

func (self *SetMode) TypeSize() uint8 {
	return 6
}

func (self *SetMode) TypeCRCExtra() uint8 {
	return 130
}

// Request to read the onboard parameter with the param_id string id. Onboard parameters are stored as key[const char*] -> value[float]. This allows to send a parameter to any other component (such as the GCS) without the need of previous knowledge of possible parameter names. Thus the same GCS can store different parameters for different autopilots. See also http://qgroundcontrol.org/parameter_interface for a full documentation of QGroundControl and IMU code.
type ParamRequestRead struct {
	ParamIndex      int16  // Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored)
	ParamId         Char16 // Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
	TargetComponent uint8  // Component ID
	TargetSystem    uint8  // System ID
}

func (self *ParamRequestRead) TypeID() uint8 {
	return 20
}

func (self *ParamRequestRead) TypeName() string {
	return "PARAM_REQUEST_READ"
}

func (self *ParamRequestRead) TypeSize() uint8 {
	return 5
}

func (self *ParamRequestRead) TypeCRCExtra() uint8 {
	return 159
}

// Request all parameters of this component. After his request, all parameters are emitted.
type ParamRequestList struct {
	TargetComponent uint8 // Component ID
	TargetSystem    uint8 // System ID
}

func (self *ParamRequestList) TypeID() uint8 {
	return 21
}

func (self *ParamRequestList) TypeName() string {
	return "PARAM_REQUEST_LIST"
}

func (self *ParamRequestList) TypeSize() uint8 {
	return 2
}

func (self *ParamRequestList) TypeCRCExtra() uint8 {
	return 108
}

// Emit the value of a onboard parameter. The inclusion of param_count and param_index in the message allows the recipient to keep track of received parameters and allows him to re-request missing parameters after a loss or timeout.
type ParamValue struct {
	ParamValue float32 // Onboard parameter value
	ParamIndex uint16  // Index of this onboard parameter
	ParamCount uint16  // Total number of onboard parameters
	ParamType  uint8   // Onboard parameter type: see the MAV_PARAM_TYPE enum for supported data types.
	ParamId    Char16  // Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
}

func (self *ParamValue) TypeID() uint8 {
	return 22
}

func (self *ParamValue) TypeName() string {
	return "PARAM_VALUE"
}

func (self *ParamValue) TypeSize() uint8 {
	return 10
}

func (self *ParamValue) TypeCRCExtra() uint8 {
	return 54
}

// Set a parameter value TEMPORARILY to RAM. It will be reset to default on system reboot. Send the ACTION MAV_ACTION_STORAGE_WRITE to PERMANENTLY write the RAM contents to EEPROM. IMPORTANT: The receiving component should acknowledge the new parameter value by sending a param_value message to all communication partners. This will also ensure that multiple GCS all have an up-to-date list of all parameters. If the sending GCS did not receive a PARAM_VALUE message within its timeout time, it should re-send the PARAM_SET message.
type ParamSet struct {
	ParamValue      float32 // Onboard parameter value
	ParamType       uint8   // Onboard parameter type: see the MAV_PARAM_TYPE enum for supported data types.
	ParamId         Char16  // Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
	TargetComponent uint8   // Component ID
	TargetSystem    uint8   // System ID
}

func (self *ParamSet) TypeID() uint8 {
	return 23
}

func (self *ParamSet) TypeName() string {
	return "PARAM_SET"
}

func (self *ParamSet) TypeSize() uint8 {
	return 8
}

func (self *ParamSet) TypeCRCExtra() uint8 {
	return 159
}

// The global position, as returned by the Global Positioning System (GPS). This is
//                 NOT the global position estimate of the sytem, but rather a RAW sensor value. See message GLOBAL_POSITION for the global position estimate. Coordinate frame is right-handed, Z-axis up (GPS frame).
type GpsRawInt struct {
	TimeUsec          uint64 // Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	Alt               int32  // Altitude (WGS84), in meters * 1000 (positive for up)
	Lon               int32  // Longitude (WGS84), in degrees * 1E7
	Lat               int32  // Latitude (WGS84), in degrees * 1E7
	Cog               uint16 // Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
	Vel               uint16 // GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX
	Epv               uint16 // GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: UINT16_MAX
	Eph               uint16 // GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: UINT16_MAX
	SatellitesVisible uint8  // Number of satellites visible. If unknown, set to 255
	FixType           uint8  // 0-1: no fix, 2: 2D fix, 3: 3D fix, 4: DGPS, 5: RTK. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
}

func (self *GpsRawInt) TypeID() uint8 {
	return 24
}

func (self *GpsRawInt) TypeName() string {
	return "GPS_RAW_INT"
}

func (self *GpsRawInt) TypeSize() uint8 {
	return 30
}

func (self *GpsRawInt) TypeCRCExtra() uint8 {
	return 184
}

// The positioning status, as reported by GPS. This message is intended to display status information about each satellite visible to the receiver. See message GLOBAL_POSITION for the global position estimate. This message can contain information for up to 20 satellites.
type GpsStatus struct {
	SatelliteSnr       [20]uint8 // Signal to noise ratio of satellite
	SatelliteAzimuth   [20]uint8 // Direction of satellite, 0: 0 deg, 255: 360 deg.
	SatelliteElevation [20]uint8 // Elevation (0: right on top of receiver, 90: on the horizon) of satellite
	SatelliteUsed      [20]uint8 // 0: Satellite not used, 1: used for localization
	SatellitePrn       [20]uint8 // Global satellite ID
	SatellitesVisible  uint8     // Number of satellites visible
}

func (self *GpsStatus) TypeID() uint8 {
	return 25
}

func (self *GpsStatus) TypeName() string {
	return "GPS_STATUS"
}

func (self *GpsStatus) TypeSize() uint8 {
	return 6
}

func (self *GpsStatus) TypeCRCExtra() uint8 {
	return 174
}

// The RAW IMU readings for the usual 9DOF sensor setup. This message should contain the scaled values to the described units
type ScaledImu struct {
	TimeBootMs uint32 // Timestamp (milliseconds since system boot)
	Zmag       int16  // Z Magnetic field (milli tesla)
	Ymag       int16  // Y Magnetic field (milli tesla)
	Xmag       int16  // X Magnetic field (milli tesla)
	Zgyro      int16  // Angular speed around Z axis (millirad /sec)
	Ygyro      int16  // Angular speed around Y axis (millirad /sec)
	Xgyro      int16  // Angular speed around X axis (millirad /sec)
	Zacc       int16  // Z acceleration (mg)
	Yacc       int16  // Y acceleration (mg)
	Xacc       int16  // X acceleration (mg)
}

func (self *ScaledImu) TypeID() uint8 {
	return 26
}

func (self *ScaledImu) TypeName() string {
	return "SCALED_IMU"
}

func (self *ScaledImu) TypeSize() uint8 {
	return 22
}

func (self *ScaledImu) TypeCRCExtra() uint8 {
	return 135
}

// The RAW IMU readings for the usual 9DOF sensor setup. This message should always contain the true raw values without any scaling to allow data capture and system debugging.
type RawImu struct {
	TimeUsec uint64 // Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	Zmag     int16  // Z Magnetic field (raw)
	Ymag     int16  // Y Magnetic field (raw)
	Xmag     int16  // X Magnetic field (raw)
	Zgyro    int16  // Angular speed around Z axis (raw)
	Ygyro    int16  // Angular speed around Y axis (raw)
	Xgyro    int16  // Angular speed around X axis (raw)
	Zacc     int16  // Z acceleration (raw)
	Yacc     int16  // Y acceleration (raw)
	Xacc     int16  // X acceleration (raw)
}

func (self *RawImu) TypeID() uint8 {
	return 27
}

func (self *RawImu) TypeName() string {
	return "RAW_IMU"
}

func (self *RawImu) TypeSize() uint8 {
	return 26
}

func (self *RawImu) TypeCRCExtra() uint8 {
	return 189
}

// The RAW pressure readings for the typical setup of one absolute pressure and one differential pressure sensor. The sensor values should be the raw, UNSCALED ADC values.
type RawPressure struct {
	TimeUsec    uint64 // Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	Temperature int16  // Raw Temperature measurement (raw)
	PressDiff2  int16  // Differential pressure 2 (raw)
	PressDiff1  int16  // Differential pressure 1 (raw)
	PressAbs    int16  // Absolute pressure (raw)
}

func (self *RawPressure) TypeID() uint8 {
	return 28
}

func (self *RawPressure) TypeName() string {
	return "RAW_PRESSURE"
}

func (self *RawPressure) TypeSize() uint8 {
	return 16
}

func (self *RawPressure) TypeCRCExtra() uint8 {
	return 176
}

// The pressure readings for the typical setup of one absolute and differential pressure sensor. The units are as specified in each field.
type ScaledPressure struct {
	PressDiff   float32 // Differential pressure 1 (hectopascal)
	PressAbs    float32 // Absolute pressure (hectopascal)
	TimeBootMs  uint32  // Timestamp (milliseconds since system boot)
	Temperature int16   // Temperature measurement (0.01 degrees celsius)
}

func (self *ScaledPressure) TypeID() uint8 {
	return 29
}

func (self *ScaledPressure) TypeName() string {
	return "SCALED_PRESSURE"
}

func (self *ScaledPressure) TypeSize() uint8 {
	return 14
}

func (self *ScaledPressure) TypeCRCExtra() uint8 {
	return 37
}

// The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right).
type Attitude struct {
	Yawspeed   float32 // Yaw angular speed (rad/s)
	Pitchspeed float32 // Pitch angular speed (rad/s)
	Rollspeed  float32 // Roll angular speed (rad/s)
	Yaw        float32 // Yaw angle (rad, -pi..+pi)
	Pitch      float32 // Pitch angle (rad, -pi..+pi)
	Roll       float32 // Roll angle (rad, -pi..+pi)
	TimeBootMs uint32  // Timestamp (milliseconds since system boot)
}

func (self *Attitude) TypeID() uint8 {
	return 30
}

func (self *Attitude) TypeName() string {
	return "ATTITUDE"
}

func (self *Attitude) TypeSize() uint8 {
	return 28
}

func (self *Attitude) TypeCRCExtra() uint8 {
	return 36
}

// The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right), expressed as quaternion. Quaternion order is w, x, y, z and a zero rotation would be expressed as (1 0 0 0).
type AttitudeQuaternion struct {
	Yawspeed   float32 // Yaw angular speed (rad/s)
	Pitchspeed float32 // Pitch angular speed (rad/s)
	Rollspeed  float32 // Roll angular speed (rad/s)
	Q4         float32 // Quaternion component 4, z (0 in null-rotation)
	Q3         float32 // Quaternion component 3, y (0 in null-rotation)
	Q2         float32 // Quaternion component 2, x (0 in null-rotation)
	Q1         float32 // Quaternion component 1, w (1 in null-rotation)
	TimeBootMs uint32  // Timestamp (milliseconds since system boot)
}

func (self *AttitudeQuaternion) TypeID() uint8 {
	return 31
}

func (self *AttitudeQuaternion) TypeName() string {
	return "ATTITUDE_QUATERNION"
}

func (self *AttitudeQuaternion) TypeSize() uint8 {
	return 32
}

func (self *AttitudeQuaternion) TypeCRCExtra() uint8 {
	return 18
}

// The filtered local position (e.g. fused computer vision and accelerometers). Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)
type LocalPositionNed struct {
	Vz         float32 // Z Speed
	Vy         float32 // Y Speed
	Vx         float32 // X Speed
	Z          float32 // Z Position
	Y          float32 // Y Position
	X          float32 // X Position
	TimeBootMs uint32  // Timestamp (milliseconds since system boot)
}

func (self *LocalPositionNed) TypeID() uint8 {
	return 32
}

func (self *LocalPositionNed) TypeName() string {
	return "LOCAL_POSITION_NED"
}

func (self *LocalPositionNed) TypeSize() uint8 {
	return 28
}

func (self *LocalPositionNed) TypeCRCExtra() uint8 {
	return 151
}

// The filtered global position (e.g. fused GPS and accelerometers). The position is in GPS-frame (right-handed, Z-up). It
//                is designed as scaled integer message since the resolution of float is not sufficient.
type GlobalPositionInt struct {
	RelativeAlt int32  // Altitude above ground in meters, expressed as * 1000 (millimeters)
	Alt         int32  // Altitude in meters, expressed as * 1000 (millimeters), WGS84 (not AMSL)
	Lon         int32  // Longitude, expressed as * 1E7
	Lat         int32  // Latitude, expressed as * 1E7
	TimeBootMs  uint32 // Timestamp (milliseconds since system boot)
	Hdg         uint16 // Compass heading in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
	Vz          int16  // Ground Z Speed (Altitude), expressed as m/s * 100
	Vy          int16  // Ground Y Speed (Longitude), expressed as m/s * 100
	Vx          int16  // Ground X Speed (Latitude), expressed as m/s * 100
}

func (self *GlobalPositionInt) TypeID() uint8 {
	return 33
}

func (self *GlobalPositionInt) TypeName() string {
	return "GLOBAL_POSITION_INT"
}

func (self *GlobalPositionInt) TypeSize() uint8 {
	return 28
}

func (self *GlobalPositionInt) TypeCRCExtra() uint8 {
	return 26
}

// The scaled values of the RC channels received. (-100%) -10000, (0%) 0, (100%) 10000. Channels that are inactive should be set to UINT16_MAX.
type RcChannelsScaled struct {
	TimeBootMs  uint32 // Timestamp (milliseconds since system boot)
	Chan8Scaled int16  // RC channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
	Chan7Scaled int16  // RC channel 7 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
	Chan6Scaled int16  // RC channel 6 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
	Chan5Scaled int16  // RC channel 5 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
	Chan4Scaled int16  // RC channel 4 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
	Chan3Scaled int16  // RC channel 3 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
	Chan2Scaled int16  // RC channel 2 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
	Chan1Scaled int16  // RC channel 1 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
	Rssi        uint8  // Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
	Port        uint8  // Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows for more than 8 servos.
}

func (self *RcChannelsScaled) TypeID() uint8 {
	return 34
}

func (self *RcChannelsScaled) TypeName() string {
	return "RC_CHANNELS_SCALED"
}

func (self *RcChannelsScaled) TypeSize() uint8 {
	return 22
}

func (self *RcChannelsScaled) TypeCRCExtra() uint8 {
	return 63
}

// The RAW values of the RC channels received. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification.
type RcChannelsRaw struct {
	TimeBootMs uint32 // Timestamp (milliseconds since system boot)
	Chan8Raw   uint16 // RC channel 8 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan7Raw   uint16 // RC channel 7 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan6Raw   uint16 // RC channel 6 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan5Raw   uint16 // RC channel 5 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan4Raw   uint16 // RC channel 4 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan3Raw   uint16 // RC channel 3 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan2Raw   uint16 // RC channel 2 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan1Raw   uint16 // RC channel 1 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Rssi       uint8  // Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
	Port       uint8  // Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows for more than 8 servos.
}

func (self *RcChannelsRaw) TypeID() uint8 {
	return 35
}

func (self *RcChannelsRaw) TypeName() string {
	return "RC_CHANNELS_RAW"
}

func (self *RcChannelsRaw) TypeSize() uint8 {
	return 22
}

func (self *RcChannelsRaw) TypeCRCExtra() uint8 {
	return 191
}

// The RAW values of the servo outputs (for RC input from the remote, use the RC_CHANNELS messages). The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%.
type ServoOutputRaw struct {
	TimeUsec  uint32 // Timestamp (microseconds since system boot)
	Servo8Raw uint16 // Servo output 8 value, in microseconds
	Servo7Raw uint16 // Servo output 7 value, in microseconds
	Servo6Raw uint16 // Servo output 6 value, in microseconds
	Servo5Raw uint16 // Servo output 5 value, in microseconds
	Servo4Raw uint16 // Servo output 4 value, in microseconds
	Servo3Raw uint16 // Servo output 3 value, in microseconds
	Servo2Raw uint16 // Servo output 2 value, in microseconds
	Servo1Raw uint16 // Servo output 1 value, in microseconds
	Port      uint8  // Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows to encode more than 8 servos.
}

func (self *ServoOutputRaw) TypeID() uint8 {
	return 36
}

func (self *ServoOutputRaw) TypeName() string {
	return "SERVO_OUTPUT_RAW"
}

func (self *ServoOutputRaw) TypeSize() uint8 {
	return 21
}

func (self *ServoOutputRaw) TypeCRCExtra() uint8 {
	return 136
}

// Request a partial list of mission items from the system/component. http://qgroundcontrol.org/mavlink/waypoint_protocol. If start and end index are the same, just send one waypoint.
type MissionRequestPartialList struct {
	EndIndex        int16 // End index, -1 by default (-1: send list to end). Else a valid index of the list
	StartIndex      int16 // Start index, 0 by default
	TargetComponent uint8 // Component ID
	TargetSystem    uint8 // System ID
}

func (self *MissionRequestPartialList) TypeID() uint8 {
	return 37
}

func (self *MissionRequestPartialList) TypeName() string {
	return "MISSION_REQUEST_PARTIAL_LIST"
}

func (self *MissionRequestPartialList) TypeSize() uint8 {
	return 6
}

func (self *MissionRequestPartialList) TypeCRCExtra() uint8 {
	return 5
}

// This message is sent to the MAV to write a partial list. If start index == end index, only one item will be transmitted / updated. If the start index is NOT 0 and above the current list size, this request should be REJECTED!
type MissionWritePartialList struct {
	EndIndex        int16 // End index, equal or greater than start index.
	StartIndex      int16 // Start index, 0 by default and smaller / equal to the largest index of the current onboard list.
	TargetComponent uint8 // Component ID
	TargetSystem    uint8 // System ID
}

func (self *MissionWritePartialList) TypeID() uint8 {
	return 38
}

func (self *MissionWritePartialList) TypeName() string {
	return "MISSION_WRITE_PARTIAL_LIST"
}

func (self *MissionWritePartialList) TypeSize() uint8 {
	return 6
}

func (self *MissionWritePartialList) TypeCRCExtra() uint8 {
	return 216
}

// Message encoding a mission item. This message is emitted to announce
//                 the presence of a mission item and to set a mission item on the system. The mission item can be either in x, y, z meters (type: LOCAL) or x:lat, y:lon, z:altitude. Local frame is Z-down, right handed (NED), global frame is Z-up, right handed (ENU). See also http://qgroundcontrol.org/mavlink/waypoint_protocol.
type MissionItem struct {
	Z               float32 // PARAM7 / z position: global: altitude (relative or absolute, depending on frame.
	Y               float32 // PARAM6 / y position: global: longitude
	X               float32 // PARAM5 / local: x position, global: latitude
	Param4          float32 // PARAM4, see MAV_CMD enum
	Param3          float32 // PARAM3, see MAV_CMD enum
	Param2          float32 // PARAM2, see MAV_CMD enum
	Param1          float32 // PARAM1, see MAV_CMD enum
	Command         uint16  // The scheduled action for the MISSION. see MAV_CMD in common.xml MAVLink specs
	Seq             uint16  // Sequence
	Autocontinue    uint8   // autocontinue to next wp
	Current         uint8   // false:0, true:1
	Frame           uint8   // The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h
	TargetComponent uint8   // Component ID
	TargetSystem    uint8   // System ID
}

func (self *MissionItem) TypeID() uint8 {
	return 39
}

func (self *MissionItem) TypeName() string {
	return "MISSION_ITEM"
}

func (self *MissionItem) TypeSize() uint8 {
	return 37
}

func (self *MissionItem) TypeCRCExtra() uint8 {
	return 236
}

// Request the information of the mission item with the sequence number seq. The response of the system to this message should be a MISSION_ITEM message. http://qgroundcontrol.org/mavlink/waypoint_protocol
type MissionRequest struct {
	Seq             uint16 // Sequence
	TargetComponent uint8  // Component ID
	TargetSystem    uint8  // System ID
}

func (self *MissionRequest) TypeID() uint8 {
	return 40
}

func (self *MissionRequest) TypeName() string {
	return "MISSION_REQUEST"
}

func (self *MissionRequest) TypeSize() uint8 {
	return 4
}

func (self *MissionRequest) TypeCRCExtra() uint8 {
	return 21
}

// Set the mission item with sequence number seq as current item. This means that the MAV will continue to this mission item on the shortest path (not following the mission items in-between).
type MissionSetCurrent struct {
	Seq             uint16 // Sequence
	TargetComponent uint8  // Component ID
	TargetSystem    uint8  // System ID
}

func (self *MissionSetCurrent) TypeID() uint8 {
	return 41
}

func (self *MissionSetCurrent) TypeName() string {
	return "MISSION_SET_CURRENT"
}

func (self *MissionSetCurrent) TypeSize() uint8 {
	return 4
}

func (self *MissionSetCurrent) TypeCRCExtra() uint8 {
	return 239
}

// Message that announces the sequence number of the current active mission item. The MAV will fly towards this mission item.
type MissionCurrent struct {
	Seq uint16 // Sequence
}

func (self *MissionCurrent) TypeID() uint8 {
	return 42
}

func (self *MissionCurrent) TypeName() string {
	return "MISSION_CURRENT"
}

func (self *MissionCurrent) TypeSize() uint8 {
	return 2
}

func (self *MissionCurrent) TypeCRCExtra() uint8 {
	return 28
}

// Request the overall list of mission items from the system/component.
type MissionRequestList struct {
	TargetComponent uint8 // Component ID
	TargetSystem    uint8 // System ID
}

func (self *MissionRequestList) TypeID() uint8 {
	return 43
}

func (self *MissionRequestList) TypeName() string {
	return "MISSION_REQUEST_LIST"
}

func (self *MissionRequestList) TypeSize() uint8 {
	return 2
}

func (self *MissionRequestList) TypeCRCExtra() uint8 {
	return 119
}

// This message is emitted as response to MISSION_REQUEST_LIST by the MAV and to initiate a write transaction. The GCS can then request the individual mission item based on the knowledge of the total number of MISSIONs.
type MissionCount struct {
	Count           uint16 // Number of mission items in the sequence
	TargetComponent uint8  // Component ID
	TargetSystem    uint8  // System ID
}

func (self *MissionCount) TypeID() uint8 {
	return 44
}

func (self *MissionCount) TypeName() string {
	return "MISSION_COUNT"
}

func (self *MissionCount) TypeSize() uint8 {
	return 4
}

func (self *MissionCount) TypeCRCExtra() uint8 {
	return 46
}

// Delete all mission items at once.
type MissionClearAll struct {
	TargetComponent uint8 // Component ID
	TargetSystem    uint8 // System ID
}

func (self *MissionClearAll) TypeID() uint8 {
	return 45
}

func (self *MissionClearAll) TypeName() string {
	return "MISSION_CLEAR_ALL"
}

func (self *MissionClearAll) TypeSize() uint8 {
	return 2
}

func (self *MissionClearAll) TypeCRCExtra() uint8 {
	return 27
}

// A certain mission item has been reached. The system will either hold this position (or circle on the orbit) or (if the autocontinue on the WP was set) continue to the next MISSION.
type MissionItemReached struct {
	Seq uint16 // Sequence
}

func (self *MissionItemReached) TypeID() uint8 {
	return 46
}

func (self *MissionItemReached) TypeName() string {
	return "MISSION_ITEM_REACHED"
}

func (self *MissionItemReached) TypeSize() uint8 {
	return 2
}

func (self *MissionItemReached) TypeCRCExtra() uint8 {
	return 11
}

// Ack message during MISSION handling. The type field states if this message is a positive ack (type=0) or if an error happened (type=non-zero).
type MissionAck struct {
	Type            uint8 // See MAV_MISSION_RESULT enum
	TargetComponent uint8 // Component ID
	TargetSystem    uint8 // System ID
}

func (self *MissionAck) TypeID() uint8 {
	return 47
}

func (self *MissionAck) TypeName() string {
	return "MISSION_ACK"
}

func (self *MissionAck) TypeSize() uint8 {
	return 3
}

func (self *MissionAck) TypeCRCExtra() uint8 {
	return 99
}

// As local waypoints exist, the global MISSION reference allows to transform between the local coordinate frame and the global (GPS) coordinate frame. This can be necessary when e.g. in- and outdoor settings are connected and the MAV should move from in- to outdoor.
type SetGpsGlobalOrigin struct {
	Altitude     int32 // Altitude (WGS84), in meters * 1000 (positive for up)
	Longitude    int32 // Longitude (WGS84, in degrees * 1E7
	Latitude     int32 // Latitude (WGS84), in degrees * 1E7
	TargetSystem uint8 // System ID
}

func (self *SetGpsGlobalOrigin) TypeID() uint8 {
	return 48
}

func (self *SetGpsGlobalOrigin) TypeName() string {
	return "SET_GPS_GLOBAL_ORIGIN"
}

func (self *SetGpsGlobalOrigin) TypeSize() uint8 {
	return 13
}

func (self *SetGpsGlobalOrigin) TypeCRCExtra() uint8 {
	return 199
}

// Once the MAV sets a new GPS-Local correspondence, this message announces the origin (0,0,0) position
type GpsGlobalOrigin struct {
	Altitude  int32 // Altitude (WGS84), in meters * 1000 (positive for up)
	Longitude int32 // Longitude (WGS84), in degrees * 1E7
	Latitude  int32 // Latitude (WGS84), in degrees * 1E7
}

func (self *GpsGlobalOrigin) TypeID() uint8 {
	return 49
}

func (self *GpsGlobalOrigin) TypeName() string {
	return "GPS_GLOBAL_ORIGIN"
}

func (self *GpsGlobalOrigin) TypeSize() uint8 {
	return 12
}

func (self *GpsGlobalOrigin) TypeCRCExtra() uint8 {
	return 65
}

// Set a safety zone (volume), which is defined by two corners of a cube. This message can be used to tell the MAV which setpoints/MISSIONs to accept and which to reject. Safety areas are often enforced by national or competition regulations.
type SafetySetAllowedArea struct {
	P2z             float32 // z position 2 / Altitude 2
	P2y             float32 // y position 2 / Longitude 2
	P2x             float32 // x position 2 / Latitude 2
	P1z             float32 // z position 1 / Altitude 1
	P1y             float32 // y position 1 / Longitude 1
	P1x             float32 // x position 1 / Latitude 1
	Frame           uint8   // Coordinate frame, as defined by MAV_FRAME enum in mavlink_types.h. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down.
	TargetComponent uint8   // Component ID
	TargetSystem    uint8   // System ID
}

func (self *SafetySetAllowedArea) TypeID() uint8 {
	return 54
}

func (self *SafetySetAllowedArea) TypeName() string {
	return "SAFETY_SET_ALLOWED_AREA"
}

func (self *SafetySetAllowedArea) TypeSize() uint8 {
	return 27
}

func (self *SafetySetAllowedArea) TypeCRCExtra() uint8 {
	return 197
}

// Read out the safety zone the MAV currently assumes.
type SafetyAllowedArea struct {
	P2z   float32 // z position 2 / Altitude 2
	P2y   float32 // y position 2 / Longitude 2
	P2x   float32 // x position 2 / Latitude 2
	P1z   float32 // z position 1 / Altitude 1
	P1y   float32 // y position 1 / Longitude 1
	P1x   float32 // x position 1 / Latitude 1
	Frame uint8   // Coordinate frame, as defined by MAV_FRAME enum in mavlink_types.h. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down.
}

func (self *SafetyAllowedArea) TypeID() uint8 {
	return 55
}

func (self *SafetyAllowedArea) TypeName() string {
	return "SAFETY_ALLOWED_AREA"
}

func (self *SafetyAllowedArea) TypeSize() uint8 {
	return 25
}

func (self *SafetyAllowedArea) TypeCRCExtra() uint8 {
	return 160
}

// The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right), expressed as quaternion. Quaternion order is w, x, y, z and a zero rotation would be expressed as (1 0 0 0).
type AttitudeQuaternionCov struct {
	Covariance [9]float32 // Attitude covariance
	Yawspeed   float32    // Yaw angular speed (rad/s)
	Pitchspeed float32    // Pitch angular speed (rad/s)
	Rollspeed  float32    // Roll angular speed (rad/s)
	Q          [4]float32 // Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation)
	TimeBootMs uint32     // Timestamp (milliseconds since system boot)
}

func (self *AttitudeQuaternionCov) TypeID() uint8 {
	return 61
}

func (self *AttitudeQuaternionCov) TypeName() string {
	return "ATTITUDE_QUATERNION_COV"
}

func (self *AttitudeQuaternionCov) TypeSize() uint8 {
	return 24
}

func (self *AttitudeQuaternionCov) TypeCRCExtra() uint8 {
	return 199
}

// Outputs of the APM navigation controller. The primary use of this message is to check the response and signs of the controller before actual flight and to assist with tuning controller parameters.
type NavControllerOutput struct {
	XtrackError   float32 // Current crosstrack error on x-y plane in meters
	AspdError     float32 // Current airspeed error in meters/second
	AltError      float32 // Current altitude error in meters
	NavPitch      float32 // Current desired pitch in degrees
	NavRoll       float32 // Current desired roll in degrees
	WpDist        uint16  // Distance to active MISSION in meters
	TargetBearing int16   // Bearing to current MISSION/target in degrees
	NavBearing    int16   // Current desired heading in degrees
}

func (self *NavControllerOutput) TypeID() uint8 {
	return 62
}

func (self *NavControllerOutput) TypeName() string {
	return "NAV_CONTROLLER_OUTPUT"
}

func (self *NavControllerOutput) TypeSize() uint8 {
	return 26
}

func (self *NavControllerOutput) TypeCRCExtra() uint8 {
	return 125
}

// The filtered global position (e.g. fused GPS and accelerometers). The position is in GPS-frame (right-handed, Z-up). It  is designed as scaled integer message since the resolution of float is not sufficient. NOTE: This message is intended for onboard networks / companion computers and higher-bandwidth links and optimized for accuracy and completeness. Please use the GLOBAL_POSITION_INT message for a minimal subset.
type GlobalPositionIntCov struct {
	TimeUtc       uint64      // Timestamp (microseconds since UNIX epoch) in UTC. 0 for unknown. Commonly filled by the precision time source of a GPS receiver.
	Covariance    [36]float32 // Covariance matrix (first six entries are the first ROW, next six entries are the second row, etc.)
	Vz            float32     // Ground Z Speed (Altitude), expressed as m/s
	Vy            float32     // Ground Y Speed (Longitude), expressed as m/s
	Vx            float32     // Ground X Speed (Latitude), expressed as m/s
	RelativeAlt   int32       // Altitude above ground in meters, expressed as * 1000 (millimeters)
	Alt           int32       // Altitude in meters, expressed as * 1000 (millimeters), above MSL
	Lon           int32       // Longitude, expressed as degrees * 1E7
	Lat           int32       // Latitude, expressed as degrees * 1E7
	TimeBootMs    uint32      // Timestamp (milliseconds since system boot)
	EstimatorType uint8       // Class id of the estimator this estimate originated from.
}

func (self *GlobalPositionIntCov) TypeID() uint8 {
	return 63
}

func (self *GlobalPositionIntCov) TypeName() string {
	return "GLOBAL_POSITION_INT_COV"
}

func (self *GlobalPositionIntCov) TypeSize() uint8 {
	return 45
}

func (self *GlobalPositionIntCov) TypeCRCExtra() uint8 {
	return 6
}

// The filtered local position (e.g. fused computer vision and accelerometers). Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)
type LocalPositionNedCov struct {
	TimeUtc       uint64      // Timestamp (microseconds since UNIX epoch) in UTC. 0 for unknown. Commonly filled by the precision time source of a GPS receiver.
	Covariance    [36]float32 // Covariance matrix (first six entries are the first ROW, next six entries are the second row, etc.)
	Vz            float32     // Z Speed
	Vy            float32     // Y Speed
	Vx            float32     // X Speed
	Z             float32     // Z Position
	Y             float32     // Y Position
	X             float32     // X Position
	TimeBootMs    uint32      // Timestamp (milliseconds since system boot)
	EstimatorType uint8       // Class id of the estimator this estimate originated from.
}

func (self *LocalPositionNedCov) TypeID() uint8 {
	return 64
}

func (self *LocalPositionNedCov) TypeName() string {
	return "LOCAL_POSITION_NED_COV"
}

func (self *LocalPositionNedCov) TypeSize() uint8 {
	return 41
}

func (self *LocalPositionNedCov) TypeCRCExtra() uint8 {
	return 115
}

// The PPM values of the RC channels received. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification.
type RcChannels struct {
	TimeBootMs uint32 // Timestamp (milliseconds since system boot)
	Chan18Raw  uint16 // RC channel 18 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan17Raw  uint16 // RC channel 17 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan16Raw  uint16 // RC channel 16 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan15Raw  uint16 // RC channel 15 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan14Raw  uint16 // RC channel 14 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan13Raw  uint16 // RC channel 13 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan12Raw  uint16 // RC channel 12 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan11Raw  uint16 // RC channel 11 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan10Raw  uint16 // RC channel 10 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan9Raw   uint16 // RC channel 9 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan8Raw   uint16 // RC channel 8 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan7Raw   uint16 // RC channel 7 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan6Raw   uint16 // RC channel 6 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan5Raw   uint16 // RC channel 5 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan4Raw   uint16 // RC channel 4 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan3Raw   uint16 // RC channel 3 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan2Raw   uint16 // RC channel 2 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Chan1Raw   uint16 // RC channel 1 value, in microseconds. A value of UINT16_MAX implies the channel is unused.
	Rssi       uint8  // Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.
	Chancount  uint8  // Total number of RC channels being received. This can be larger than 18, indicating that more channels are available but not given in this message. This value should be 0 when no RC channels are available.
}

func (self *RcChannels) TypeID() uint8 {
	return 65
}

func (self *RcChannels) TypeName() string {
	return "RC_CHANNELS"
}

func (self *RcChannels) TypeSize() uint8 {
	return 42
}

func (self *RcChannels) TypeCRCExtra() uint8 {
	return 182
}

//
type RequestDataStream struct {
	ReqMessageRate  uint16 // The requested interval between two messages of this type
	StartStop       uint8  // 1 to start sending, 0 to stop sending.
	ReqStreamId     uint8  // The ID of the requested data stream
	TargetComponent uint8  // The target requested to send the message stream.
	TargetSystem    uint8  // The target requested to send the message stream.
}

func (self *RequestDataStream) TypeID() uint8 {
	return 66
}

func (self *RequestDataStream) TypeName() string {
	return "REQUEST_DATA_STREAM"
}

func (self *RequestDataStream) TypeSize() uint8 {
	return 6
}

func (self *RequestDataStream) TypeCRCExtra() uint8 {
	return 247
}

//
type DataStream struct {
	MessageRate uint16 // The requested interval between two messages of this type
	OnOff       uint8  // 1 stream is enabled, 0 stream is stopped.
	StreamId    uint8  // The ID of the requested data stream
}

func (self *DataStream) TypeID() uint8 {
	return 67
}

func (self *DataStream) TypeName() string {
	return "DATA_STREAM"
}

func (self *DataStream) TypeSize() uint8 {
	return 4
}

func (self *DataStream) TypeCRCExtra() uint8 {
	return 193
}

// This message provides an API for manually controlling the vehicle using standard joystick axes nomenclature, along with a joystick-like input device. Unused axes can be disabled an buttons are also transmit as boolean values of their
type ManualControl struct {
	Buttons uint16 // A bitfield corresponding to the joystick buttons' current state, 1 for pressed, 0 for released. The lowest bit corresponds to Button 1.
	R       int16  // R-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to a twisting of the joystick, with counter-clockwise being 1000 and clockwise being -1000, and the yaw of a vehicle.
	Z       int16  // Z-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to a separate slider movement with maximum being 1000 and minimum being -1000 on a joystick and the thrust of a vehicle.
	Y       int16  // Y-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to left(-1000)-right(1000) movement on a joystick and the roll of a vehicle.
	X       int16  // X-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to forward(1000)-backward(-1000) movement on a joystick and the pitch of a vehicle.
	Target  uint8  // The system to be controlled.
}

func (self *ManualControl) TypeID() uint8 {
	return 69
}

func (self *ManualControl) TypeName() string {
	return "MANUAL_CONTROL"
}

func (self *ManualControl) TypeSize() uint8 {
	return 11
}

func (self *ManualControl) TypeCRCExtra() uint8 {
	return 130
}

// The RAW values of the RC channels sent to the MAV to override info received from the RC radio. A value of UINT16_MAX means no change to that channel. A value of 0 means control of that channel should be released back to the RC radio. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification.
type RcChannelsOverride struct {
	Chan8Raw        uint16 // RC channel 8 value, in microseconds. A value of UINT16_MAX means to ignore this field.
	Chan7Raw        uint16 // RC channel 7 value, in microseconds. A value of UINT16_MAX means to ignore this field.
	Chan6Raw        uint16 // RC channel 6 value, in microseconds. A value of UINT16_MAX means to ignore this field.
	Chan5Raw        uint16 // RC channel 5 value, in microseconds. A value of UINT16_MAX means to ignore this field.
	Chan4Raw        uint16 // RC channel 4 value, in microseconds. A value of UINT16_MAX means to ignore this field.
	Chan3Raw        uint16 // RC channel 3 value, in microseconds. A value of UINT16_MAX means to ignore this field.
	Chan2Raw        uint16 // RC channel 2 value, in microseconds. A value of UINT16_MAX means to ignore this field.
	Chan1Raw        uint16 // RC channel 1 value, in microseconds. A value of UINT16_MAX means to ignore this field.
	TargetComponent uint8  // Component ID
	TargetSystem    uint8  // System ID
}

func (self *RcChannelsOverride) TypeID() uint8 {
	return 70
}

func (self *RcChannelsOverride) TypeName() string {
	return "RC_CHANNELS_OVERRIDE"
}

func (self *RcChannelsOverride) TypeSize() uint8 {
	return 18
}

func (self *RcChannelsOverride) TypeCRCExtra() uint8 {
	return 32
}

// Message encoding a mission item. This message is emitted to announce
//                 the presence of a mission item and to set a mission item on the system. The mission item can be either in x, y, z meters (type: LOCAL) or x:lat, y:lon, z:altitude. Local frame is Z-down, right handed (NED), global frame is Z-up, right handed (ENU). See alsohttp://qgroundcontrol.org/mavlink/waypoint_protocol.
type MissionItemInt struct {
	Z               float32 // PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame.
	Y               int32   // PARAM6 / y position: local: x position in meters * 1e4, global: longitude in degrees *10^7
	X               int32   // PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7
	Param4          float32 // PARAM4, see MAV_CMD enum
	Param3          float32 // PARAM3, see MAV_CMD enum
	Param2          float32 // PARAM2, see MAV_CMD enum
	Param1          float32 // PARAM1, see MAV_CMD enum
	Command         uint16  // The scheduled action for the MISSION. see MAV_CMD in common.xml MAVLink specs
	Seq             uint16  // Waypoint ID (sequence number). Starts at zero. Increases monotonically for each waypoint, no gaps in the sequence (0,1,2,3,4).
	Autocontinue    uint8   // autocontinue to next wp
	Current         uint8   // false:0, true:1
	Frame           uint8   // The coordinate system of the MISSION. see MAV_FRAME in mavlink_types.h
	TargetComponent uint8   // Component ID
	TargetSystem    uint8   // System ID
}

func (self *MissionItemInt) TypeID() uint8 {
	return 73
}

func (self *MissionItemInt) TypeName() string {
	return "MISSION_ITEM_INT"
}

func (self *MissionItemInt) TypeSize() uint8 {
	return 37
}

func (self *MissionItemInt) TypeCRCExtra() uint8 {
	return 253
}

// Metrics typically displayed on a HUD for fixed wing aircraft
type VfrHud struct {
	Climb       float32 // Current climb rate in meters/second
	Alt         float32 // Current altitude (MSL), in meters
	Groundspeed float32 // Current ground speed in m/s
	Airspeed    float32 // Current airspeed in m/s
	Throttle    uint16  // Current throttle setting in integer percent, 0 to 100
	Heading     int16   // Current heading in degrees, in compass units (0..360, 0=north)
}

func (self *VfrHud) TypeID() uint8 {
	return 74
}

func (self *VfrHud) TypeName() string {
	return "VFR_HUD"
}

func (self *VfrHud) TypeSize() uint8 {
	return 20
}

func (self *VfrHud) TypeCRCExtra() uint8 {
	return 238
}

// Message encoding a command with parameters as scaled integers. Scaling depends on the actual command value.
type CommandInt struct {
	Z               float32 // PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame.
	Y               int32   // PARAM6 / local: y position in meters * 1e4, global: longitude in degrees * 10^7
	X               int32   // PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7
	Param4          float32 // PARAM4, see MAV_CMD enum
	Param3          float32 // PARAM3, see MAV_CMD enum
	Param2          float32 // PARAM2, see MAV_CMD enum
	Param1          float32 // PARAM1, see MAV_CMD enum
	Command         uint16  // The scheduled action for the mission item. see MAV_CMD in common.xml MAVLink specs
	Autocontinue    uint8   // autocontinue to next wp
	Current         uint8   // false:0, true:1
	Frame           uint8   // The coordinate system of the COMMAND. see MAV_FRAME in mavlink_types.h
	TargetComponent uint8   // Component ID
	TargetSystem    uint8   // System ID
}

func (self *CommandInt) TypeID() uint8 {
	return 75
}

func (self *CommandInt) TypeName() string {
	return "COMMAND_INT"
}

func (self *CommandInt) TypeSize() uint8 {
	return 35
}

func (self *CommandInt) TypeCRCExtra() uint8 {
	return 109
}

// Send a command with up to seven parameters to the MAV
type CommandLong struct {
	Param7          float32 // Parameter 7, as defined by MAV_CMD enum.
	Param6          float32 // Parameter 6, as defined by MAV_CMD enum.
	Param5          float32 // Parameter 5, as defined by MAV_CMD enum.
	Param4          float32 // Parameter 4, as defined by MAV_CMD enum.
	Param3          float32 // Parameter 3, as defined by MAV_CMD enum.
	Param2          float32 // Parameter 2, as defined by MAV_CMD enum.
	Param1          float32 // Parameter 1, as defined by MAV_CMD enum.
	Command         uint16  // Command ID, as defined by MAV_CMD enum.
	Confirmation    uint8   // 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
	TargetComponent uint8   // Component which should execute the command, 0 for all components
	TargetSystem    uint8   // System which should execute the command
}

func (self *CommandLong) TypeID() uint8 {
	return 76
}

func (self *CommandLong) TypeName() string {
	return "COMMAND_LONG"
}

func (self *CommandLong) TypeSize() uint8 {
	return 33
}

func (self *CommandLong) TypeCRCExtra() uint8 {
	return 140
}

// Report status of a command. Includes feedback wether the command was executed.
type CommandAck struct {
	Command uint16 // Command ID, as defined by MAV_CMD enum.
	Result  uint8  // See MAV_RESULT enum
}

func (self *CommandAck) TypeID() uint8 {
	return 77
}

func (self *CommandAck) TypeName() string {
	return "COMMAND_ACK"
}

func (self *CommandAck) TypeSize() uint8 {
	return 3
}

func (self *CommandAck) TypeCRCExtra() uint8 {
	return 143
}

// Setpoint in roll, pitch, yaw and thrust from the operator
type ManualSetpoint struct {
	Thrust               float32 // Collective thrust, normalized to 0 .. 1
	Yaw                  float32 // Desired yaw rate in radians per second
	Pitch                float32 // Desired pitch rate in radians per second
	Roll                 float32 // Desired roll rate in radians per second
	TimeBootMs           uint32  // Timestamp in milliseconds since system boot
	ManualOverrideSwitch uint8   // Override mode switch position, 0.. 255
	ModeSwitch           uint8   // Flight mode switch position, 0.. 255
}

func (self *ManualSetpoint) TypeID() uint8 {
	return 81
}

func (self *ManualSetpoint) TypeName() string {
	return "MANUAL_SETPOINT"
}

func (self *ManualSetpoint) TypeSize() uint8 {
	return 22
}

func (self *ManualSetpoint) TypeCRCExtra() uint8 {
	return 231
}

// Set the vehicle attitude and body angular rates.
type SetAttitudeTarget struct {
	Thrust          float32    // Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)
	BodyYawRate     float32    // Body roll rate in radians per second
	BodyPitchRate   float32    // Body roll rate in radians per second
	BodyRollRate    float32    // Body roll rate in radians per second
	Q               [4]float32 // Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
	TimeBootMs      uint32     // Timestamp in milliseconds since system boot
	TypeMask        uint8      // Mappings: If any of these bits are set, the corresponding input should be ignored: bit 1: body roll rate, bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 6: reserved, bit 7: throttle, bit 8: attitude
	TargetComponent uint8      // Component ID
	TargetSystem    uint8      // System ID
}

func (self *SetAttitudeTarget) TypeID() uint8 {
	return 82
}

func (self *SetAttitudeTarget) TypeName() string {
	return "SET_ATTITUDE_TARGET"
}

func (self *SetAttitudeTarget) TypeSize() uint8 {
	return 27
}

func (self *SetAttitudeTarget) TypeCRCExtra() uint8 {
	return 250
}

// Set the vehicle attitude and body angular rates.
type AttitudeTarget struct {
	Thrust        float32    // Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)
	BodyYawRate   float32    // Body roll rate in radians per second
	BodyPitchRate float32    // Body roll rate in radians per second
	BodyRollRate  float32    // Body roll rate in radians per second
	Q             [4]float32 // Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
	TimeBootMs    uint32     // Timestamp in milliseconds since system boot
	TypeMask      uint8      // Mappings: If any of these bits are set, the corresponding input should be ignored: bit 1: body roll rate, bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 7: reserved, bit 8: attitude
}

func (self *AttitudeTarget) TypeID() uint8 {
	return 83
}

func (self *AttitudeTarget) TypeName() string {
	return "ATTITUDE_TARGET"
}

func (self *AttitudeTarget) TypeSize() uint8 {
	return 25
}

func (self *AttitudeTarget) TypeCRCExtra() uint8 {
	return 225
}

// Set vehicle position, velocity and acceleration setpoint in local frame.
type SetPositionTargetLocalNed struct {
	YawRate         float32 // yaw rate setpoint in rad/s
	Yaw             float32 // yaw setpoint in rad
	Afz             float32 // Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Afy             float32 // Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Afx             float32 // X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Vz              float32 // Z velocity in NED frame in meter / s
	Vy              float32 // Y velocity in NED frame in meter / s
	Vx              float32 // X velocity in NED frame in meter / s
	Z               float32 // Z Position in NED frame in meters (note, altitude is negative in NED)
	Y               float32 // Y Position in NED frame in meters
	X               float32 // X Position in NED frame in meters
	TimeBootMs      uint32  // Timestamp in milliseconds since system boot
	TypeMask        uint16  // Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint, bit 11: yaw, bit 12: yaw rate
	CoordinateFrame uint8   // Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED = 9
	TargetComponent uint8   // Component ID
	TargetSystem    uint8   // System ID
}

func (self *SetPositionTargetLocalNed) TypeID() uint8 {
	return 84
}

func (self *SetPositionTargetLocalNed) TypeName() string {
	return "SET_POSITION_TARGET_LOCAL_NED"
}

func (self *SetPositionTargetLocalNed) TypeSize() uint8 {
	return 53
}

func (self *SetPositionTargetLocalNed) TypeCRCExtra() uint8 {
	return 109
}

// Set vehicle position, velocity and acceleration setpoint in local frame.
type PositionTargetLocalNed struct {
	YawRate         float32 // yaw rate setpoint in rad/s
	Yaw             float32 // yaw setpoint in rad
	Afz             float32 // Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Afy             float32 // Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Afx             float32 // X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Vz              float32 // Z velocity in NED frame in meter / s
	Vy              float32 // Y velocity in NED frame in meter / s
	Vx              float32 // X velocity in NED frame in meter / s
	Z               float32 // Z Position in NED frame in meters (note, altitude is negative in NED)
	Y               float32 // Y Position in NED frame in meters
	X               float32 // X Position in NED frame in meters
	TimeBootMs      uint32  // Timestamp in milliseconds since system boot
	TypeMask        uint16  // Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint, bit 11: yaw, bit 12: yaw rate
	CoordinateFrame uint8   // Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED = 9
}

func (self *PositionTargetLocalNed) TypeID() uint8 {
	return 85
}

func (self *PositionTargetLocalNed) TypeName() string {
	return "POSITION_TARGET_LOCAL_NED"
}

func (self *PositionTargetLocalNed) TypeSize() uint8 {
	return 51
}

func (self *PositionTargetLocalNed) TypeCRCExtra() uint8 {
	return 211
}

// Set vehicle position, velocity and acceleration setpoint in the WGS84 coordinate system.
type SetPositionTargetGlobalInt struct {
	YawRate         float32 // yaw rate setpoint in rad/s
	Yaw             float32 // yaw setpoint in rad
	Afz             float32 // Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Afy             float32 // Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Afx             float32 // X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Vz              float32 // Z velocity in NED frame in meter / s
	Vy              float32 // Y velocity in NED frame in meter / s
	Vx              float32 // X velocity in NED frame in meter / s
	Alt             float32 // Altitude in meters in WGS84 altitude, not AMSL if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
	LonInt          int32   // Y Position in WGS84 frame in 1e7 * meters
	LatInt          int32   // X Position in WGS84 frame in 1e7 * meters
	TimeBootMs      uint32  // Timestamp in milliseconds since system boot. The rationale for the timestamp in the setpoint is to allow the system to compensate for the transport delay of the setpoint. This allows the system to compensate processing latency.
	TypeMask        uint16  // Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint, bit 11: yaw, bit 12: yaw rate
	CoordinateFrame uint8   // Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11
	TargetComponent uint8   // Component ID
	TargetSystem    uint8   // System ID
}

func (self *SetPositionTargetGlobalInt) TypeID() uint8 {
	return 86
}

func (self *SetPositionTargetGlobalInt) TypeName() string {
	return "SET_POSITION_TARGET_GLOBAL_INT"
}

func (self *SetPositionTargetGlobalInt) TypeSize() uint8 {
	return 53
}

func (self *SetPositionTargetGlobalInt) TypeCRCExtra() uint8 {
	return 207
}

// Set vehicle position, velocity and acceleration setpoint in the WGS84 coordinate system.
type PositionTargetGlobalInt struct {
	YawRate         float32 // yaw rate setpoint in rad/s
	Yaw             float32 // yaw setpoint in rad
	Afz             float32 // Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Afy             float32 // Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Afx             float32 // X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Vz              float32 // Z velocity in NED frame in meter / s
	Vy              float32 // Y velocity in NED frame in meter / s
	Vx              float32 // X velocity in NED frame in meter / s
	Alt             float32 // Altitude in meters in WGS84 altitude, not AMSL if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
	LonInt          int32   // Y Position in WGS84 frame in 1e7 * meters
	LatInt          int32   // X Position in WGS84 frame in 1e7 * meters
	TimeBootMs      uint32  // Timestamp in milliseconds since system boot. The rationale for the timestamp in the setpoint is to allow the system to compensate for the transport delay of the setpoint. This allows the system to compensate processing latency.
	TypeMask        uint16  // Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint, bit 11: yaw, bit 12: yaw rate
	CoordinateFrame uint8   // Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11
}

func (self *PositionTargetGlobalInt) TypeID() uint8 {
	return 87
}

func (self *PositionTargetGlobalInt) TypeName() string {
	return "POSITION_TARGET_GLOBAL_INT"
}

func (self *PositionTargetGlobalInt) TypeSize() uint8 {
	return 51
}

func (self *PositionTargetGlobalInt) TypeCRCExtra() uint8 {
	return 33
}

// The offset in X, Y, Z and yaw between the LOCAL_POSITION_NED messages of MAV X and the global coordinate frame in NED coordinates. Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)
type LocalPositionNedSystemGlobalOffset struct {
	Yaw        float32 // Yaw
	Pitch      float32 // Pitch
	Roll       float32 // Roll
	Z          float32 // Z Position
	Y          float32 // Y Position
	X          float32 // X Position
	TimeBootMs uint32  // Timestamp (milliseconds since system boot)
}

func (self *LocalPositionNedSystemGlobalOffset) TypeID() uint8 {
	return 89
}

func (self *LocalPositionNedSystemGlobalOffset) TypeName() string {
	return "LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET"
}

func (self *LocalPositionNedSystemGlobalOffset) TypeSize() uint8 {
	return 28
}

func (self *LocalPositionNedSystemGlobalOffset) TypeCRCExtra() uint8 {
	return 142
}

// DEPRECATED PACKET! Suffers from missing airspeed fields and singularities due to Euler angles. Please use HIL_STATE_QUATERNION instead. Sent from simulation to autopilot. This packet is useful for high throughput applications such as hardware in the loop simulations.
type HilState struct {
	TimeUsec   uint64  // Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	Alt        int32   // Altitude in meters, expressed as * 1000 (millimeters)
	Lon        int32   // Longitude, expressed as * 1E7
	Lat        int32   // Latitude, expressed as * 1E7
	Yawspeed   float32 // Body frame yaw / psi angular speed (rad/s)
	Pitchspeed float32 // Body frame pitch / theta angular speed (rad/s)
	Rollspeed  float32 // Body frame roll / phi angular speed (rad/s)
	Yaw        float32 // Yaw angle (rad)
	Pitch      float32 // Pitch angle (rad)
	Roll       float32 // Roll angle (rad)
	Zacc       int16   // Z acceleration (mg)
	Yacc       int16   // Y acceleration (mg)
	Xacc       int16   // X acceleration (mg)
	Vz         int16   // Ground Z Speed (Altitude), expressed as m/s * 100
	Vy         int16   // Ground Y Speed (Longitude), expressed as m/s * 100
	Vx         int16   // Ground X Speed (Latitude), expressed as m/s * 100
}

func (self *HilState) TypeID() uint8 {
	return 90
}

func (self *HilState) TypeName() string {
	return "HIL_STATE"
}

func (self *HilState) TypeSize() uint8 {
	return 56
}

func (self *HilState) TypeCRCExtra() uint8 {
	return 135
}

// Sent from autopilot to simulation. Hardware in the loop control outputs
type HilControls struct {
	TimeUsec      uint64  // Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	Aux4          float32 // Aux 4, -1 .. 1
	Aux3          float32 // Aux 3, -1 .. 1
	Aux2          float32 // Aux 2, -1 .. 1
	Aux1          float32 // Aux 1, -1 .. 1
	Throttle      float32 // Throttle 0 .. 1
	YawRudder     float32 // Control output -1 .. 1
	PitchElevator float32 // Control output -1 .. 1
	RollAilerons  float32 // Control output -1 .. 1
	NavMode       uint8   // Navigation mode (MAV_NAV_MODE)
	Mode          uint8   // System mode (MAV_MODE)
}

func (self *HilControls) TypeID() uint8 {
	return 91
}

func (self *HilControls) TypeName() string {
	return "HIL_CONTROLS"
}

func (self *HilControls) TypeSize() uint8 {
	return 42
}

func (self *HilControls) TypeCRCExtra() uint8 {
	return 255
}

// Sent from simulation to autopilot. The RAW values of the RC channels received. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification.
type HilRcInputsRaw struct {
	TimeUsec  uint64 // Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	Chan12Raw uint16 // RC channel 12 value, in microseconds
	Chan11Raw uint16 // RC channel 11 value, in microseconds
	Chan10Raw uint16 // RC channel 10 value, in microseconds
	Chan9Raw  uint16 // RC channel 9 value, in microseconds
	Chan8Raw  uint16 // RC channel 8 value, in microseconds
	Chan7Raw  uint16 // RC channel 7 value, in microseconds
	Chan6Raw  uint16 // RC channel 6 value, in microseconds
	Chan5Raw  uint16 // RC channel 5 value, in microseconds
	Chan4Raw  uint16 // RC channel 4 value, in microseconds
	Chan3Raw  uint16 // RC channel 3 value, in microseconds
	Chan2Raw  uint16 // RC channel 2 value, in microseconds
	Chan1Raw  uint16 // RC channel 1 value, in microseconds
	Rssi      uint8  // Receive signal strength indicator, 0: 0%, 255: 100%
}

func (self *HilRcInputsRaw) TypeID() uint8 {
	return 92
}

func (self *HilRcInputsRaw) TypeName() string {
	return "HIL_RC_INPUTS_RAW"
}

func (self *HilRcInputsRaw) TypeSize() uint8 {
	return 33
}

func (self *HilRcInputsRaw) TypeCRCExtra() uint8 {
	return 190
}

// Optical flow from a flow sensor (e.g. optical mouse sensor)
type OpticalFlow struct {
	TimeUsec       uint64  // Timestamp (UNIX)
	GroundDistance float32 // Ground distance in meters. Positive value: distance known. Negative value: Unknown distance
	FlowCompMY     float32 // Flow in meters in y-sensor direction, angular-speed compensated
	FlowCompMX     float32 // Flow in meters in x-sensor direction, angular-speed compensated
	FlowY          int16   // Flow in pixels * 10 in y-sensor direction (dezi-pixels)
	FlowX          int16   // Flow in pixels * 10 in x-sensor direction (dezi-pixels)
	Quality        uint8   // Optical flow quality / confidence. 0: bad, 255: maximum quality
	SensorId       uint8   // Sensor ID
}

func (self *OpticalFlow) TypeID() uint8 {
	return 100
}

func (self *OpticalFlow) TypeName() string {
	return "OPTICAL_FLOW"
}

func (self *OpticalFlow) TypeSize() uint8 {
	return 26
}

func (self *OpticalFlow) TypeCRCExtra() uint8 {
	return 53
}

//
type GlobalVisionPositionEstimate struct {
	Usec  uint64  // Timestamp (microseconds, synced to UNIX time or since system boot)
	Yaw   float32 // Yaw angle in rad
	Pitch float32 // Pitch angle in rad
	Roll  float32 // Roll angle in rad
	Z     float32 // Global Z position
	Y     float32 // Global Y position
	X     float32 // Global X position
}

func (self *GlobalVisionPositionEstimate) TypeID() uint8 {
	return 101
}

func (self *GlobalVisionPositionEstimate) TypeName() string {
	return "GLOBAL_VISION_POSITION_ESTIMATE"
}

func (self *GlobalVisionPositionEstimate) TypeSize() uint8 {
	return 32
}

func (self *GlobalVisionPositionEstimate) TypeCRCExtra() uint8 {
	return 10
}

//
type VisionPositionEstimate struct {
	Usec  uint64  // Timestamp (microseconds, synced to UNIX time or since system boot)
	Yaw   float32 // Yaw angle in rad
	Pitch float32 // Pitch angle in rad
	Roll  float32 // Roll angle in rad
	Z     float32 // Global Z position
	Y     float32 // Global Y position
	X     float32 // Global X position
}

func (self *VisionPositionEstimate) TypeID() uint8 {
	return 102
}

func (self *VisionPositionEstimate) TypeName() string {
	return "VISION_POSITION_ESTIMATE"
}

func (self *VisionPositionEstimate) TypeSize() uint8 {
	return 32
}

func (self *VisionPositionEstimate) TypeCRCExtra() uint8 {
	return 242
}

//
type VisionSpeedEstimate struct {
	Usec uint64  // Timestamp (microseconds, synced to UNIX time or since system boot)
	Z    float32 // Global Z speed
	Y    float32 // Global Y speed
	X    float32 // Global X speed
}

func (self *VisionSpeedEstimate) TypeID() uint8 {
	return 103
}

func (self *VisionSpeedEstimate) TypeName() string {
	return "VISION_SPEED_ESTIMATE"
}

func (self *VisionSpeedEstimate) TypeSize() uint8 {
	return 20
}

func (self *VisionSpeedEstimate) TypeCRCExtra() uint8 {
	return 117
}

//
type ViconPositionEstimate struct {
	Usec  uint64  // Timestamp (microseconds, synced to UNIX time or since system boot)
	Yaw   float32 // Yaw angle in rad
	Pitch float32 // Pitch angle in rad
	Roll  float32 // Roll angle in rad
	Z     float32 // Global Z position
	Y     float32 // Global Y position
	X     float32 // Global X position
}

func (self *ViconPositionEstimate) TypeID() uint8 {
	return 104
}

func (self *ViconPositionEstimate) TypeName() string {
	return "VICON_POSITION_ESTIMATE"
}

func (self *ViconPositionEstimate) TypeSize() uint8 {
	return 32
}

func (self *ViconPositionEstimate) TypeCRCExtra() uint8 {
	return 84
}

// The IMU readings in SI units in NED body frame
type HighresImu struct {
	TimeUsec      uint64  // Timestamp (microseconds, synced to UNIX time or since system boot)
	Temperature   float32 // Temperature in degrees celsius
	PressureAlt   float32 // Altitude calculated from pressure
	DiffPressure  float32 // Differential pressure in millibar
	AbsPressure   float32 // Absolute pressure in millibar
	Zmag          float32 // Z Magnetic field (Gauss)
	Ymag          float32 // Y Magnetic field (Gauss)
	Xmag          float32 // X Magnetic field (Gauss)
	Zgyro         float32 // Angular speed around Z axis (rad / sec)
	Ygyro         float32 // Angular speed around Y axis (rad / sec)
	Xgyro         float32 // Angular speed around X axis (rad / sec)
	Zacc          float32 // Z acceleration (m/s^2)
	Yacc          float32 // Y acceleration (m/s^2)
	Xacc          float32 // X acceleration (m/s^2)
	FieldsUpdated uint16  // Bitmask for fields that have updated since last message, bit 0 = xacc, bit 12: temperature
}

func (self *HighresImu) TypeID() uint8 {
	return 105
}

func (self *HighresImu) TypeName() string {
	return "HIGHRES_IMU"
}

func (self *HighresImu) TypeSize() uint8 {
	return 62
}

func (self *HighresImu) TypeCRCExtra() uint8 {
	return 100
}

// Optical flow from an angular rate flow sensor (e.g. PX4FLOW or mouse sensor)
type OpticalFlowRad struct {
	TimeUsec            uint64  // Timestamp (microseconds, synced to UNIX time or since system boot)
	Distance            float32 // Distance to the center of the flow field in meters. Positive value (including zero): distance known. Negative value: Unknown distance.
	TimeDeltaDistanceUs uint32  // Time in microseconds since the distance was sampled.
	IntegratedZgyro     float32 // RH rotation around Z axis (rad)
	IntegratedYgyro     float32 // RH rotation around Y axis (rad)
	IntegratedXgyro     float32 // RH rotation around X axis (rad)
	IntegratedY         float32 // Flow in radians around Y axis (Sensor RH rotation about the Y axis induces a positive flow. Sensor linear motion along the positive X axis induces a positive flow.)
	IntegratedX         float32 // Flow in radians around X axis (Sensor RH rotation about the X axis induces a positive flow. Sensor linear motion along the positive Y axis induces a negative flow.)
	IntegrationTimeUs   uint32  // Integration time in microseconds. Divide integrated_x and integrated_y by the integration time to obtain average flow. The integration time also indicates the.
	Temperature         int16   // Temperature * 100 in centi-degrees Celsius
	Quality             uint8   // Optical flow quality / confidence. 0: no valid flow, 255: maximum quality
	SensorId            uint8   // Sensor ID
}

func (self *OpticalFlowRad) TypeID() uint8 {
	return 106
}

func (self *OpticalFlowRad) TypeName() string {
	return "OPTICAL_FLOW_RAD"
}

func (self *OpticalFlowRad) TypeSize() uint8 {
	return 44
}

func (self *OpticalFlowRad) TypeCRCExtra() uint8 {
	return 11
}

// The IMU readings in SI units in NED body frame
type HilSensor struct {
	TimeUsec      uint64  // Timestamp (microseconds, synced to UNIX time or since system boot)
	FieldsUpdated uint32  // Bitmask for fields that have updated since last message, bit 0 = xacc, bit 12: temperature
	Temperature   float32 // Temperature in degrees celsius
	PressureAlt   float32 // Altitude calculated from pressure
	DiffPressure  float32 // Differential pressure (airspeed) in millibar
	AbsPressure   float32 // Absolute pressure in millibar
	Zmag          float32 // Z Magnetic field (Gauss)
	Ymag          float32 // Y Magnetic field (Gauss)
	Xmag          float32 // X Magnetic field (Gauss)
	Zgyro         float32 // Angular speed around Z axis in body frame (rad / sec)
	Ygyro         float32 // Angular speed around Y axis in body frame (rad / sec)
	Xgyro         float32 // Angular speed around X axis in body frame (rad / sec)
	Zacc          float32 // Z acceleration (m/s^2)
	Yacc          float32 // Y acceleration (m/s^2)
	Xacc          float32 // X acceleration (m/s^2)
}

func (self *HilSensor) TypeID() uint8 {
	return 107
}

func (self *HilSensor) TypeName() string {
	return "HIL_SENSOR"
}

func (self *HilSensor) TypeSize() uint8 {
	return 64
}

func (self *HilSensor) TypeCRCExtra() uint8 {
	return 189
}

// Status of simulation environment, if used
type SimState struct {
	Vd         float32 // True velocity in m/s in DOWN direction in earth-fixed NED frame
	Ve         float32 // True velocity in m/s in EAST direction in earth-fixed NED frame
	Vn         float32 // True velocity in m/s in NORTH direction in earth-fixed NED frame
	StdDevVert float32 // Vertical position standard deviation
	StdDevHorz float32 // Horizontal position standard deviation
	Alt        float32 // Altitude in meters
	Lon        float32 // Longitude in degrees
	Lat        float32 // Latitude in degrees
	Zgyro      float32 // Angular speed around Z axis rad/s
	Ygyro      float32 // Angular speed around Y axis rad/s
	Xgyro      float32 // Angular speed around X axis rad/s
	Zacc       float32 // Z acceleration m/s/s
	Yacc       float32 // Y acceleration m/s/s
	Xacc       float32 // X acceleration m/s/s
	Yaw        float32 // Attitude yaw expressed as Euler angles, not recommended except for human-readable outputs
	Pitch      float32 // Attitude pitch expressed as Euler angles, not recommended except for human-readable outputs
	Roll       float32 // Attitude roll expressed as Euler angles, not recommended except for human-readable outputs
	Q4         float32 // True attitude quaternion component 4, z (0 in null-rotation)
	Q3         float32 // True attitude quaternion component 3, y (0 in null-rotation)
	Q2         float32 // True attitude quaternion component 2, x (0 in null-rotation)
	Q1         float32 // True attitude quaternion component 1, w (1 in null-rotation)
}

func (self *SimState) TypeID() uint8 {
	return 108
}

func (self *SimState) TypeName() string {
	return "SIM_STATE"
}

func (self *SimState) TypeSize() uint8 {
	return 84
}

func (self *SimState) TypeCRCExtra() uint8 {
	return 98
}

// Status generated by radio
type RadioStatus struct {
	Fixed    uint16 // count of error corrected packets
	Rxerrors uint16 // receive errors
	Remnoise uint8  // remote background noise level
	Noise    uint8  // background noise level
	Txbuf    uint8  // how full the tx buffer is as a percentage
	Remrssi  uint8  // remote signal strength
	Rssi     uint8  // local signal strength
}

func (self *RadioStatus) TypeID() uint8 {
	return 109
}

func (self *RadioStatus) TypeName() string {
	return "RADIO_STATUS"
}

func (self *RadioStatus) TypeSize() uint8 {
	return 9
}

func (self *RadioStatus) TypeCRCExtra() uint8 {
	return 247
}

// File transfer message
type FileTransferProtocol struct {
	Payload         [251]uint8 // Variable length payload. The length is defined by the remaining message length when subtracting the header and other fields.  The entire content of this block is opaque unless you understand any the encoding message_type.  The particular encoding used can be extension specific and might not always be documented as part of the mavlink specification.
	TargetComponent uint8      // Component ID (0 for broadcast)
	TargetSystem    uint8      // System ID (0 for broadcast)
	TargetNetwork   uint8      // Network ID (0 for broadcast)
}

func (self *FileTransferProtocol) TypeID() uint8 {
	return 110
}

func (self *FileTransferProtocol) TypeName() string {
	return "FILE_TRANSFER_PROTOCOL"
}

func (self *FileTransferProtocol) TypeSize() uint8 {
	return 4
}

func (self *FileTransferProtocol) TypeCRCExtra() uint8 {
	return 79
}

// Time synchronization message.
type Timesync struct {
	Ts1 int64 // Time sync timestamp 2
	Tc1 int64 // Time sync timestamp 1
}

func (self *Timesync) TypeID() uint8 {
	return 111
}

func (self *Timesync) TypeName() string {
	return "TIMESYNC"
}

func (self *Timesync) TypeSize() uint8 {
	return 16
}

func (self *Timesync) TypeCRCExtra() uint8 {
	return 116
}

// The global position, as returned by the Global Positioning System (GPS). This is
//                  NOT the global position estimate of the sytem, but rather a RAW sensor value. See message GLOBAL_POSITION for the global position estimate. Coordinate frame is right-handed, Z-axis up (GPS frame).
type HilGps struct {
	TimeUsec          uint64 // Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	Alt               int32  // Altitude (WGS84), in meters * 1000 (positive for up)
	Lon               int32  // Longitude (WGS84), in degrees * 1E7
	Lat               int32  // Latitude (WGS84), in degrees * 1E7
	Cog               uint16 // Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535
	Vd                int16  // GPS velocity in cm/s in DOWN direction in earth-fixed NED frame
	Ve                int16  // GPS velocity in cm/s in EAST direction in earth-fixed NED frame
	Vn                int16  // GPS velocity in cm/s in NORTH direction in earth-fixed NED frame
	Vel               uint16 // GPS ground speed (m/s * 100). If unknown, set to: 65535
	Epv               uint16 // GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: 65535
	Eph               uint16 // GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535
	SatellitesVisible uint8  // Number of satellites visible. If unknown, set to 255
	FixType           uint8  // 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
}

func (self *HilGps) TypeID() uint8 {
	return 113
}

func (self *HilGps) TypeName() string {
	return "HIL_GPS"
}

func (self *HilGps) TypeSize() uint8 {
	return 36
}

func (self *HilGps) TypeCRCExtra() uint8 {
	return 88
}

// Simulated optical flow from a flow sensor (e.g. PX4FLOW or optical mouse sensor)
type HilOpticalFlow struct {
	TimeUsec            uint64  // Timestamp (microseconds, synced to UNIX time or since system boot)
	Distance            float32 // Distance to the center of the flow field in meters. Positive value (including zero): distance known. Negative value: Unknown distance.
	TimeDeltaDistanceUs uint32  // Time in microseconds since the distance was sampled.
	IntegratedZgyro     float32 // RH rotation around Z axis (rad)
	IntegratedYgyro     float32 // RH rotation around Y axis (rad)
	IntegratedXgyro     float32 // RH rotation around X axis (rad)
	IntegratedY         float32 // Flow in radians around Y axis (Sensor RH rotation about the Y axis induces a positive flow. Sensor linear motion along the positive X axis induces a positive flow.)
	IntegratedX         float32 // Flow in radians around X axis (Sensor RH rotation about the X axis induces a positive flow. Sensor linear motion along the positive Y axis induces a negative flow.)
	IntegrationTimeUs   uint32  // Integration time in microseconds. Divide integrated_x and integrated_y by the integration time to obtain average flow. The integration time also indicates the.
	Temperature         int16   // Temperature * 100 in centi-degrees Celsius
	Quality             uint8   // Optical flow quality / confidence. 0: no valid flow, 255: maximum quality
	SensorId            uint8   // Sensor ID
}

func (self *HilOpticalFlow) TypeID() uint8 {
	return 114
}

func (self *HilOpticalFlow) TypeName() string {
	return "HIL_OPTICAL_FLOW"
}

func (self *HilOpticalFlow) TypeSize() uint8 {
	return 44
}

func (self *HilOpticalFlow) TypeCRCExtra() uint8 {
	return 108
}

// Sent from simulation to autopilot, avoids in contrast to HIL_STATE singularities. This packet is useful for high throughput applications such as hardware in the loop simulations.
type HilStateQuaternion struct {
	TimeUsec           uint64     // Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	Alt                int32      // Altitude in meters, expressed as * 1000 (millimeters)
	Lon                int32      // Longitude, expressed as * 1E7
	Lat                int32      // Latitude, expressed as * 1E7
	Yawspeed           float32    // Body frame yaw / psi angular speed (rad/s)
	Pitchspeed         float32    // Body frame pitch / theta angular speed (rad/s)
	Rollspeed          float32    // Body frame roll / phi angular speed (rad/s)
	AttitudeQuaternion [4]float32 // Vehicle attitude expressed as normalized quaternion in w, x, y, z order (with 1 0 0 0 being the null-rotation)
	Zacc               int16      // Z acceleration (mg)
	Yacc               int16      // Y acceleration (mg)
	Xacc               int16      // X acceleration (mg)
	TrueAirspeed       uint16     // True airspeed, expressed as m/s * 100
	IndAirspeed        uint16     // Indicated airspeed, expressed as m/s * 100
	Vz                 int16      // Ground Z Speed (Altitude), expressed as m/s * 100
	Vy                 int16      // Ground Y Speed (Longitude), expressed as m/s * 100
	Vx                 int16      // Ground X Speed (Latitude), expressed as m/s * 100
}

func (self *HilStateQuaternion) TypeID() uint8 {
	return 115
}

func (self *HilStateQuaternion) TypeName() string {
	return "HIL_STATE_QUATERNION"
}

func (self *HilStateQuaternion) TypeSize() uint8 {
	return 52
}

func (self *HilStateQuaternion) TypeCRCExtra() uint8 {
	return 147
}

// The RAW IMU readings for secondary 9DOF sensor setup. This message should contain the scaled values to the described units
type ScaledImu2 struct {
	TimeBootMs uint32 // Timestamp (milliseconds since system boot)
	Zmag       int16  // Z Magnetic field (milli tesla)
	Ymag       int16  // Y Magnetic field (milli tesla)
	Xmag       int16  // X Magnetic field (milli tesla)
	Zgyro      int16  // Angular speed around Z axis (millirad /sec)
	Ygyro      int16  // Angular speed around Y axis (millirad /sec)
	Xgyro      int16  // Angular speed around X axis (millirad /sec)
	Zacc       int16  // Z acceleration (mg)
	Yacc       int16  // Y acceleration (mg)
	Xacc       int16  // X acceleration (mg)
}

func (self *ScaledImu2) TypeID() uint8 {
	return 116
}

func (self *ScaledImu2) TypeName() string {
	return "SCALED_IMU2"
}

func (self *ScaledImu2) TypeSize() uint8 {
	return 22
}

func (self *ScaledImu2) TypeCRCExtra() uint8 {
	return 97
}

// Request a list of available logs. On some systems calling this may stop on-board logging until LOG_REQUEST_END is called.
type LogRequestList struct {
	End             uint16 // Last log id (0xffff for last available)
	Start           uint16 // First log id (0 for first available)
	TargetComponent uint8  // Component ID
	TargetSystem    uint8  // System ID
}

func (self *LogRequestList) TypeID() uint8 {
	return 117
}

func (self *LogRequestList) TypeName() string {
	return "LOG_REQUEST_LIST"
}

func (self *LogRequestList) TypeSize() uint8 {
	return 6
}

func (self *LogRequestList) TypeCRCExtra() uint8 {
	return 158
}

// Reply to LOG_REQUEST_LIST
type LogEntry struct {
	Size       uint32 // Size of the log (may be approximate) in bytes
	TimeUtc    uint32 // UTC timestamp of log in seconds since 1970, or 0 if not available
	LastLogNum uint16 // High log number
	NumLogs    uint16 // Total number of logs
	Id         uint16 // Log id
}

func (self *LogEntry) TypeID() uint8 {
	return 118
}

func (self *LogEntry) TypeName() string {
	return "LOG_ENTRY"
}

func (self *LogEntry) TypeSize() uint8 {
	return 14
}

func (self *LogEntry) TypeCRCExtra() uint8 {
	return 173
}

// Request a chunk of a log
type LogRequestData struct {
	Count           uint32 // Number of bytes
	Ofs             uint32 // Offset into the log
	Id              uint16 // Log id (from LOG_ENTRY reply)
	TargetComponent uint8  // Component ID
	TargetSystem    uint8  // System ID
}

func (self *LogRequestData) TypeID() uint8 {
	return 119
}

func (self *LogRequestData) TypeName() string {
	return "LOG_REQUEST_DATA"
}

func (self *LogRequestData) TypeSize() uint8 {
	return 12
}

func (self *LogRequestData) TypeCRCExtra() uint8 {
	return 245
}

// Reply to LOG_REQUEST_DATA
type LogData struct {
	Ofs   uint32    // Offset into the log
	Id    uint16    // Log id (from LOG_ENTRY reply)
	Data  [90]uint8 // log data
	Count uint8     // Number of bytes (zero for end of log)
}

func (self *LogData) TypeID() uint8 {
	return 120
}

func (self *LogData) TypeName() string {
	return "LOG_DATA"
}

func (self *LogData) TypeSize() uint8 {
	return 8
}

func (self *LogData) TypeCRCExtra() uint8 {
	return 199
}

// Erase all logs
type LogErase struct {
	TargetComponent uint8 // Component ID
	TargetSystem    uint8 // System ID
}

func (self *LogErase) TypeID() uint8 {
	return 121
}

func (self *LogErase) TypeName() string {
	return "LOG_ERASE"
}

func (self *LogErase) TypeSize() uint8 {
	return 2
}

func (self *LogErase) TypeCRCExtra() uint8 {
	return 30
}

// Stop log transfer and resume normal logging
type LogRequestEnd struct {
	TargetComponent uint8 // Component ID
	TargetSystem    uint8 // System ID
}

func (self *LogRequestEnd) TypeID() uint8 {
	return 122
}

func (self *LogRequestEnd) TypeName() string {
	return "LOG_REQUEST_END"
}

func (self *LogRequestEnd) TypeSize() uint8 {
	return 2
}

func (self *LogRequestEnd) TypeCRCExtra() uint8 {
	return 56
}

// data for injecting into the onboard GPS (used for DGPS)
type GpsInjectData struct {
	Data            [110]uint8 // raw data (110 is enough for 12 satellites of RTCMv2)
	Len             uint8      // data length
	TargetComponent uint8      // Component ID
	TargetSystem    uint8      // System ID
}

func (self *GpsInjectData) TypeID() uint8 {
	return 123
}

func (self *GpsInjectData) TypeName() string {
	return "GPS_INJECT_DATA"
}

func (self *GpsInjectData) TypeSize() uint8 {
	return 4
}

func (self *GpsInjectData) TypeCRCExtra() uint8 {
	return 75
}

// Second GPS data. Coordinate frame is right-handed, Z-axis up (GPS frame).
type Gps2Raw struct {
	TimeUsec          uint64 // Timestamp (microseconds since UNIX epoch or microseconds since system boot)
	DgpsAge           uint32 // Age of DGPS info
	Alt               int32  // Altitude (WGS84), in meters * 1000 (positive for up)
	Lon               int32  // Longitude (WGS84), in degrees * 1E7
	Lat               int32  // Latitude (WGS84), in degrees * 1E7
	Cog               uint16 // Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
	Vel               uint16 // GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX
	Epv               uint16 // GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: UINT16_MAX
	Eph               uint16 // GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: UINT16_MAX
	DgpsNumch         uint8  // Number of DGPS satellites
	SatellitesVisible uint8  // Number of satellites visible. If unknown, set to 255
	FixType           uint8  // 0-1: no fix, 2: 2D fix, 3: 3D fix, 4: DGPS fix, 5: RTK Fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
}

func (self *Gps2Raw) TypeID() uint8 {
	return 124
}

func (self *Gps2Raw) TypeName() string {
	return "GPS2_RAW"
}

func (self *Gps2Raw) TypeSize() uint8 {
	return 35
}

func (self *Gps2Raw) TypeCRCExtra() uint8 {
	return 223
}

// Power supply status
type PowerStatus struct {
	Flags  uint16 // power supply status flags (see MAV_POWER_STATUS enum)
	Vservo uint16 // servo rail voltage in millivolts
	Vcc    uint16 // 5V rail voltage in millivolts
}

func (self *PowerStatus) TypeID() uint8 {
	return 125
}

func (self *PowerStatus) TypeName() string {
	return "POWER_STATUS"
}

func (self *PowerStatus) TypeSize() uint8 {
	return 6
}

func (self *PowerStatus) TypeCRCExtra() uint8 {
	return 145
}

// Control a serial port. This can be used for raw access to an onboard serial peripheral such as a GPS or telemetry radio. It is designed to make it possible to update the devices firmware via MAVLink messages or change the devices settings. A message with zero bytes can be used to change just the baudrate.
type SerialControl struct {
	Baudrate uint32    // Baudrate of transfer. Zero means no change.
	Timeout  uint16    // Timeout for reply data in milliseconds
	Data     [70]uint8 // serial data
	Count    uint8     // how many bytes in this transfer
	Flags    uint8     // See SERIAL_CONTROL_FLAG enum
	Device   uint8     // See SERIAL_CONTROL_DEV enum
}

func (self *SerialControl) TypeID() uint8 {
	return 126
}

func (self *SerialControl) TypeName() string {
	return "SERIAL_CONTROL"
}

func (self *SerialControl) TypeSize() uint8 {
	return 10
}

func (self *SerialControl) TypeCRCExtra() uint8 {
	return 89
}

// RTK GPS data. Gives information on the relative baseline calculation the GPS is reporting
type GpsRtk struct {
	IarNumHypotheses   int32  // Current number of integer ambiguity hypotheses.
	Accuracy           uint32 // Current estimate of baseline accuracy.
	BaselineCMm        int32  // Current baseline in ECEF z or NED down component in mm.
	BaselineBMm        int32  // Current baseline in ECEF y or NED east component in mm.
	BaselineAMm        int32  // Current baseline in ECEF x or NED north component in mm.
	Tow                uint32 // GPS Time of Week of last baseline
	TimeLastBaselineMs uint32 // Time since boot of last baseline message received in ms.
	Wn                 uint16 // GPS Week Number of last baseline
	BaselineCoordsType uint8  // Coordinate system of baseline. 0 == ECEF, 1 == NED
	Nsats              uint8  // Current number of sats used for RTK calculation.
	RtkRate            uint8  // Rate of baseline messages being received by GPS, in HZ
	RtkHealth          uint8  // GPS-specific health report for RTK data.
	RtkReceiverId      uint8  // Identification of connected RTK receiver.
}

func (self *GpsRtk) TypeID() uint8 {
	return 127
}

func (self *GpsRtk) TypeName() string {
	return "GPS_RTK"
}

func (self *GpsRtk) TypeSize() uint8 {
	return 35
}

func (self *GpsRtk) TypeCRCExtra() uint8 {
	return 242
}

// RTK GPS data. Gives information on the relative baseline calculation the GPS is reporting
type Gps2Rtk struct {
	IarNumHypotheses   int32  // Current number of integer ambiguity hypotheses.
	Accuracy           uint32 // Current estimate of baseline accuracy.
	BaselineCMm        int32  // Current baseline in ECEF z or NED down component in mm.
	BaselineBMm        int32  // Current baseline in ECEF y or NED east component in mm.
	BaselineAMm        int32  // Current baseline in ECEF x or NED north component in mm.
	Tow                uint32 // GPS Time of Week of last baseline
	TimeLastBaselineMs uint32 // Time since boot of last baseline message received in ms.
	Wn                 uint16 // GPS Week Number of last baseline
	BaselineCoordsType uint8  // Coordinate system of baseline. 0 == ECEF, 1 == NED
	Nsats              uint8  // Current number of sats used for RTK calculation.
	RtkRate            uint8  // Rate of baseline messages being received by GPS, in HZ
	RtkHealth          uint8  // GPS-specific health report for RTK data.
	RtkReceiverId      uint8  // Identification of connected RTK receiver.
}

func (self *Gps2Rtk) TypeID() uint8 {
	return 128
}

func (self *Gps2Rtk) TypeName() string {
	return "GPS2_RTK"
}

func (self *Gps2Rtk) TypeSize() uint8 {
	return 35
}

func (self *Gps2Rtk) TypeCRCExtra() uint8 {
	return 9
}

//
type DataTransmissionHandshake struct {
	Size       uint32 // total data size in bytes (set on ACK only)
	Packets    uint16 // number of packets beeing sent (set on ACK only)
	Height     uint16 // Height of a matrix or image
	Width      uint16 // Width of a matrix or image
	JpgQuality uint8  // JPEG quality out of [1,100]
	Payload    uint8  // payload size per packet (normally 253 byte, see DATA field size in message ENCAPSULATED_DATA) (set on ACK only)
	Type       uint8  // type of requested/acknowledged data (as defined in ENUM DATA_TYPES in mavlink/include/mavlink_types.h)
}

func (self *DataTransmissionHandshake) TypeID() uint8 {
	return 130
}

func (self *DataTransmissionHandshake) TypeName() string {
	return "DATA_TRANSMISSION_HANDSHAKE"
}

func (self *DataTransmissionHandshake) TypeSize() uint8 {
	return 13
}

func (self *DataTransmissionHandshake) TypeCRCExtra() uint8 {
	return 29
}

//
type EncapsulatedData struct {
	Seqnr uint16     // sequence number (starting with 0 on every transmission)
	Data  [253]uint8 // image data bytes
}

func (self *EncapsulatedData) TypeID() uint8 {
	return 131
}

func (self *EncapsulatedData) TypeName() string {
	return "ENCAPSULATED_DATA"
}

func (self *EncapsulatedData) TypeSize() uint8 {
	return 3
}

func (self *EncapsulatedData) TypeCRCExtra() uint8 {
	return 30
}

//
type DistanceSensor struct {
	TimeBootMs      uint32 // Time since system boot
	CurrentDistance uint16 // Current distance reading
	MaxDistance     uint16 // Maximum distance the sensor can measure in centimeters
	MinDistance     uint16 // Minimum distance the sensor can measure in centimeters
	Covariance      uint8  // Measurement covariance in centimeters, 0 for unknown / invalid readings
	Orientation     uint8  // Direction the sensor faces from FIXME enum.
	Id              uint8  // Onboard ID of the sensor
	Type            uint8  // Type from MAV_DISTANCE_SENSOR enum.
}

func (self *DistanceSensor) TypeID() uint8 {
	return 132
}

func (self *DistanceSensor) TypeName() string {
	return "DISTANCE_SENSOR"
}

func (self *DistanceSensor) TypeSize() uint8 {
	return 14
}

func (self *DistanceSensor) TypeCRCExtra() uint8 {
	return 125
}

// Request for terrain data and terrain status
type TerrainRequest struct {
	Mask        uint64 // Bitmask of requested 4x4 grids (row major 8x7 array of grids, 56 bits)
	Lon         int32  // Longitude of SW corner of first grid (in degrees *10^7)
	Lat         int32  // Latitude of SW corner of first grid (degrees *10^7)
	GridSpacing uint16 // Grid spacing in meters
}

func (self *TerrainRequest) TypeID() uint8 {
	return 133
}

func (self *TerrainRequest) TypeName() string {
	return "TERRAIN_REQUEST"
}

func (self *TerrainRequest) TypeSize() uint8 {
	return 18
}

func (self *TerrainRequest) TypeCRCExtra() uint8 {
	return 212
}

// Terrain data sent from GCS. The lat/lon and grid_spacing must be the same as a lat/lon from a TERRAIN_REQUEST
type TerrainData struct {
	Lon         int32     // Longitude of SW corner of first grid (in degrees *10^7)
	Lat         int32     // Latitude of SW corner of first grid (degrees *10^7)
	Data        [16]int16 // Terrain data in meters AMSL
	GridSpacing uint16    // Grid spacing in meters
	Gridbit     uint8     // bit within the terrain request mask
}

func (self *TerrainData) TypeID() uint8 {
	return 134
}

func (self *TerrainData) TypeName() string {
	return "TERRAIN_DATA"
}

func (self *TerrainData) TypeSize() uint8 {
	return 13
}

func (self *TerrainData) TypeCRCExtra() uint8 {
	return 243
}

// Request that the vehicle report terrain height at the given location. Used by GCS to check if vehicle has all terrain data needed for a mission.
type TerrainCheck struct {
	Lon int32 // Longitude (degrees *10^7)
	Lat int32 // Latitude (degrees *10^7)
}

func (self *TerrainCheck) TypeID() uint8 {
	return 135
}

func (self *TerrainCheck) TypeName() string {
	return "TERRAIN_CHECK"
}

func (self *TerrainCheck) TypeSize() uint8 {
	return 8
}

func (self *TerrainCheck) TypeCRCExtra() uint8 {
	return 155
}

// Response from a TERRAIN_CHECK request
type TerrainReport struct {
	CurrentHeight float32 // Current vehicle height above lat/lon terrain height (meters)
	TerrainHeight float32 // Terrain height in meters AMSL
	Lon           int32   // Longitude (degrees *10^7)
	Lat           int32   // Latitude (degrees *10^7)
	Loaded        uint16  // Number of 4x4 terrain blocks in memory
	Pending       uint16  // Number of 4x4 terrain blocks waiting to be received or read from disk
	Spacing       uint16  // grid spacing (zero if terrain at this location unavailable)
}

func (self *TerrainReport) TypeID() uint8 {
	return 136
}

func (self *TerrainReport) TypeName() string {
	return "TERRAIN_REPORT"
}

func (self *TerrainReport) TypeSize() uint8 {
	return 22
}

func (self *TerrainReport) TypeCRCExtra() uint8 {
	return 239
}

// Battery information
type BatteryStatus struct {
	EnergyConsumed   int32      // Consumed energy, in 100*Joules (intergrated U*I*dt)  (1 = 100 Joule), -1: autopilot does not provide energy consumption estimate
	CurrentConsumed  int32      // Consumed charge, in milliampere hours (1 = 1 mAh), -1: autopilot does not provide mAh consumption estimate
	CurrentBattery   int16      // Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
	Voltages         [10]uint16 // Battery voltage of cells, in millivolts (1 = 1 millivolt)
	Temperature      int16      // Temperature of the battery in centi-degrees celsius. INT16_MAX for unknown temperature.
	BatteryRemaining int8       // Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot does not estimate the remaining battery
	Type             uint8      // Type (chemistry) of the battery
	BatteryFunction  uint8      // Function of the battery
	Id               uint8      // Battery ID
}

func (self *BatteryStatus) TypeID() uint8 {
	return 147
}

func (self *BatteryStatus) TypeName() string {
	return "BATTERY_STATUS"
}

func (self *BatteryStatus) TypeSize() uint8 {
	return 18
}

func (self *BatteryStatus) TypeCRCExtra() uint8 {
	return 162
}

// Version and capability of autopilot software
type AutopilotVersion struct {
	Capabilities  uint64   // bitmask of capabilities (see MAV_PROTOCOL_CAPABILITY enum)
	Version       uint32   // Firmware version number
	CustomVersion [8]uint8 // Custom version field, commonly the first 8 bytes (16 characters) of the git hash. This is not an unique identifier, but should allow to identify the commit using the main version number even for very large code bases.
}

func (self *AutopilotVersion) TypeID() uint8 {
	return 148
}

func (self *AutopilotVersion) TypeName() string {
	return "AUTOPILOT_VERSION"
}

func (self *AutopilotVersion) TypeSize() uint8 {
	return 13
}

func (self *AutopilotVersion) TypeCRCExtra() uint8 {
	return 216
}

// Message implementing parts of the V2 payload specs in V1 frames for transitional support.
type V2Extension struct {
	MessageType     uint16     // A code that identifies the software component that understands this message (analogous to usb device classes or mime type strings).  If this code is less than 32768, it is considered a 'registered' protocol extension and the corresponding entry should be added to https://github.com/mavlink/mavlink/extension-message-ids.xml.  Software creators can register blocks of message IDs as needed (useful for GCS specific metadata, etc...). Message_types greater than 32767 are considered local experiments and should not be checked in to any widely distributed codebase.
	Payload         [249]uint8 // Variable length payload. The length is defined by the remaining message length when subtracting the header and other fields.  The entire content of this block is opaque unless you understand any the encoding message_type.  The particular encoding used can be extension specific and might not always be documented as part of the mavlink specification.
	TargetComponent uint8      // Component ID (0 for broadcast)
	TargetSystem    uint8      // System ID (0 for broadcast)
	TargetNetwork   uint8      // Network ID (0 for broadcast)
}

func (self *V2Extension) TypeID() uint8 {
	return 248
}

func (self *V2Extension) TypeName() string {
	return "V2_EXTENSION"
}

func (self *V2Extension) TypeSize() uint8 {
	return 6
}

func (self *V2Extension) TypeCRCExtra() uint8 {
	return 41
}

// Send raw controller memory. The use of this message is discouraged for normal packets, but a quite efficient way for testing new messages and getting experimental debug output.
type MemoryVect struct {
	Address uint16   // Starting address of the debug variables
	Value   [32]int8 // Memory contents at specified address
	Type    uint8    // Type code of the memory variables. for ver = 1: 0=16 x int16_t, 1=16 x uint16_t, 2=16 x Q15, 3=16 x 1Q14
	Ver     uint8    // Version code of the type variable. 0=unknown, type ignored and assumed int16_t. 1=as below
}

func (self *MemoryVect) TypeID() uint8 {
	return 249
}

func (self *MemoryVect) TypeName() string {
	return "MEMORY_VECT"
}

func (self *MemoryVect) TypeSize() uint8 {
	return 5
}

func (self *MemoryVect) TypeCRCExtra() uint8 {
	return 112
}

//
type DebugVect struct {
	TimeUsec uint64  // Timestamp
	Z        float32 // z
	Y        float32 // y
	X        float32 // x
	Name     Char10  // Name
}

func (self *DebugVect) TypeID() uint8 {
	return 250
}

func (self *DebugVect) TypeName() string {
	return "DEBUG_VECT"
}

func (self *DebugVect) TypeSize() uint8 {
	return 21
}

func (self *DebugVect) TypeCRCExtra() uint8 {
	return 141
}

// Send a key-value pair as float. The use of this message is discouraged for normal packets, but a quite efficient way for testing new messages and getting experimental debug output.
type NamedValueFloat struct {
	Value      float32 // Floating point value
	TimeBootMs uint32  // Timestamp (milliseconds since system boot)
	Name       Char10  // Name of the debug variable
}

func (self *NamedValueFloat) TypeID() uint8 {
	return 251
}

func (self *NamedValueFloat) TypeName() string {
	return "NAMED_VALUE_FLOAT"
}

func (self *NamedValueFloat) TypeSize() uint8 {
	return 9
}

func (self *NamedValueFloat) TypeCRCExtra() uint8 {
	return 253
}

// Send a key-value pair as integer. The use of this message is discouraged for normal packets, but a quite efficient way for testing new messages and getting experimental debug output.
type NamedValueInt struct {
	Value      int32  // Signed integer value
	TimeBootMs uint32 // Timestamp (milliseconds since system boot)
	Name       Char10 // Name of the debug variable
}

func (self *NamedValueInt) TypeID() uint8 {
	return 252
}

func (self *NamedValueInt) TypeName() string {
	return "NAMED_VALUE_INT"
}

func (self *NamedValueInt) TypeSize() uint8 {
	return 9
}

func (self *NamedValueInt) TypeCRCExtra() uint8 {
	return 86
}

// Status text message. These messages are printed in yellow in the COMM console of QGroundControl. WARNING: They consume quite some bandwidth, so use only for important status and error messages. If implemented wisely, these messages are buffered on the MCU and sent only at a limited rate (e.g. 10 Hz).
type Statustext struct {
	Text     Char50 // Status text message, without null termination character
	Severity uint8  // Severity of status. Relies on the definitions within RFC-5424. See enum MAV_SEVERITY.
}

func (self *Statustext) TypeID() uint8 {
	return 253
}

func (self *Statustext) TypeName() string {
	return "STATUSTEXT"
}

func (self *Statustext) TypeSize() uint8 {
	return 2
}

func (self *Statustext) TypeCRCExtra() uint8 {
	return 78
}

// Send a debug value. The index is used to discriminate between values. These values show up in the plot of QGroundControl as DEBUG N.
type Debug struct {
	Value      float32 // DEBUG value
	TimeBootMs uint32  // Timestamp (milliseconds since system boot)
	Ind        uint8   // index of debug variable
}

func (self *Debug) TypeID() uint8 {
	return 254
}

func (self *Debug) TypeName() string {
	return "DEBUG"
}

func (self *Debug) TypeSize() uint8 {
	return 9
}

func (self *Debug) TypeCRCExtra() uint8 {
	return 29
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

type Char4 [4]byte

func (chars *Char4) String() string {
	return string(truncate(chars[:]))
}

type Char8 [8]byte

func (chars *Char8) String() string {
	return string(truncate(chars[:]))
}

type Char9 [9]byte

func (chars *Char9) String() string {
	return string(truncate(chars[:]))
}

type Char10 [10]byte

func (chars *Char10) String() string {
	return string(truncate(chars[:]))
}

type Char16 [16]byte

func (chars *Char16) String() string {
	return string(truncate(chars[:]))
}

type Char20 [20]byte

func (chars *Char20) String() string {
	return string(truncate(chars[:]))
}

type Char25 [25]byte

func (chars *Char25) String() string {
	return string(truncate(chars[:]))
}

type Char32 [32]byte

func (chars *Char32) String() string {
	return string(truncate(chars[:]))
}

type Char36 [36]byte

func (chars *Char36) String() string {
	return string(truncate(chars[:]))
}

type Char50 [50]byte

func (chars *Char50) String() string {
	return string(truncate(chars[:]))
}

type Char70 [70]byte

func (chars *Char70) String() string {
	return string(truncate(chars[:]))
}

type Char90 [90]byte

func (chars *Char90) String() string {
	return string(truncate(chars[:]))
}

type Char110 [110]byte

func (chars *Char110) String() string {
	return string(truncate(chars[:]))
}

type Char249 [249]byte

func (chars *Char249) String() string {
	return string(truncate(chars[:]))
}

type Char251 [251]byte

func (chars *Char251) String() string {
	return string(truncate(chars[:]))
}

type Char253 [253]byte

func (chars *Char253) String() string {
	return string(truncate(chars[:]))
}
