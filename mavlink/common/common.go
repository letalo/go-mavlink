package common

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
	MAV_STATE_BOOT        = 0 // System is booting up.
	MAV_STATE_CALIBRATING = 0 // System is calibrating and not flight-ready.
	MAV_STATE_STANDBY     = 0 // System is grounded and on standby. It can be launched any time.
	MAV_STATE_ACTIVE      = 0 // System is active and might be already airborne. Motors are engaged.
	MAV_STATE_CRITICAL    = 0 // System is in a non-normal flight mode. It can however still navigate.
	MAV_STATE_EMERGENCY   = 0 // System is in a non-normal flight mode. It lost control over parts or over the whole airframe. It is in mayday and going down.
	MAV_STATE_POWEROFF    = 0 // System just initialized its power-down sequence, will shut down now.
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
	MAVLINK_DATA_STREAM_IMG_BMP    = 0 //
	MAVLINK_DATA_STREAM_IMG_RAW8U  = 0 //
	MAVLINK_DATA_STREAM_IMG_RAW32U = 0 //
	MAVLINK_DATA_STREAM_IMG_PGM    = 0 //
	MAVLINK_DATA_STREAM_IMG_PNG    = 0 //
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
	MAV_CMD_ACK_ERR_FAIL                           = 0 // Generic error message if none of the other reasons fails or if no detailed error reporting is implemented.
	MAV_CMD_ACK_ERR_ACCESS_DENIED                  = 0 // The system is refusing to accept this command from this source / communication partner.
	MAV_CMD_ACK_ERR_NOT_SUPPORTED                  = 0 // Command or mission item is not supported, other commands would be accepted.
	MAV_CMD_ACK_ERR_COORDINATE_FRAME_NOT_SUPPORTED = 0 // The coordinate frame of this command / mission item is not supported.
	MAV_CMD_ACK_ERR_COORDINATES_OUT_OF_RANGE       = 0 // The coordinate frame of this command is ok, but he coordinate values exceed the safety limits of this system. This is a generic error, please use the more specific error messages below if possible.
	MAV_CMD_ACK_ERR_X_LAT_OUT_OF_RANGE             = 0 // The X or latitude value is out of range.
	MAV_CMD_ACK_ERR_Y_LON_OUT_OF_RANGE             = 0 // The Y or longitude value is out of range.
	MAV_CMD_ACK_ERR_Z_ALT_OUT_OF_RANGE             = 0 // The Z or altitude value is out of range.
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
	Type           uint8
	Autopilot      uint8
	BaseMode       uint8
	CustomMode     uint32
	SystemStatus   uint8
	MavlinkVersion uint8
}

func (self *Heartbeat) MsgID() uint8 {
	return 0
}

func (self *Heartbeat) MsgName() string {
	return "Heartbeat"
}

func (self *Heartbeat) MsgSize() uint8 {
	return 0
}

// The general system state. If the system is following the MAVLink standard, the system state is mainly defined by three orthogonal states/modes: The system mode, which is either LOCKED (motors shut down and locked), MANUAL (system under RC control), GUIDED (system with autonomous position control, position setpoint controlled manually) or AUTO (system guided by path/waypoint planner). The NAV_MODE defined the current flight state: LIFTOFF (often an open-loop maneuver), LANDING, WAYPOINTS or VECTOR. This represents the internal navigation state machine. The system status shows wether the system is currently active or not and if an emergency occured. During the CRITICAL and EMERGENCY states the MAV is still considered to be active, but should start emergency procedures autonomously. After a failure occured it should first move from active to critical to allow manual intervention and then move to emergency after a certain timeout.
type SysStatus struct {
	OnboardControlSensorsPresent uint32
	OnboardControlSensorsEnabled uint32
	OnboardControlSensorsHealth  uint32
	Load                         uint16
	VoltageBattery               uint16
	CurrentBattery               int16
	BatteryRemaining             int8
	DropRateComm                 uint16
	ErrorsComm                   uint16
	ErrorsCount1                 uint16
	ErrorsCount2                 uint16
	ErrorsCount3                 uint16
	ErrorsCount4                 uint16
}

func (self *SysStatus) MsgID() uint8 {
	return 1
}

func (self *SysStatus) MsgName() string {
	return "SysStatus"
}

func (self *SysStatus) MsgSize() uint8 {
	return 0
}

// The system time is the time of the master clock, typically the computer clock of the main onboard computer.
type SystemTime struct {
	TimeUnixUsec uint64
	TimeBootMs   uint32
}

func (self *SystemTime) MsgID() uint8 {
	return 2
}

func (self *SystemTime) MsgName() string {
	return "SystemTime"
}

func (self *SystemTime) MsgSize() uint8 {
	return 0
}

// A ping message either requesting or responding to a ping. This allows to measure the system latencies, including serial port, radio modem and UDP connections.
type Ping struct {
	TimeUsec        uint64
	Seq             uint32
	TargetSystem    uint8
	TargetComponent uint8
}

func (self *Ping) MsgID() uint8 {
	return 4
}

func (self *Ping) MsgName() string {
	return "Ping"
}

func (self *Ping) MsgSize() uint8 {
	return 0
}

// Request to control this MAV
type ChangeOperatorControl struct {
	TargetSystem   uint8
	ControlRequest uint8
	Version        uint8
	Passkey        Char25
}

func (self *ChangeOperatorControl) MsgID() uint8 {
	return 5
}

func (self *ChangeOperatorControl) MsgName() string {
	return "ChangeOperatorControl"
}

func (self *ChangeOperatorControl) MsgSize() uint8 {
	return 0
}

// Accept / deny control of this MAV
type ChangeOperatorControlAck struct {
	GcsSystemId    uint8
	ControlRequest uint8
	Ack            uint8
}

func (self *ChangeOperatorControlAck) MsgID() uint8 {
	return 6
}

func (self *ChangeOperatorControlAck) MsgName() string {
	return "ChangeOperatorControlAck"
}

func (self *ChangeOperatorControlAck) MsgSize() uint8 {
	return 0
}

// Emit an encrypted signature / key identifying this system. PLEASE NOTE: This protocol has been kept simple, so transmitting the key requires an encrypted channel for true safety.
type AuthKey struct {
	Key Char32
}

func (self *AuthKey) MsgID() uint8 {
	return 7
}

func (self *AuthKey) MsgName() string {
	return "AuthKey"
}

func (self *AuthKey) MsgSize() uint8 {
	return 0
}

// Set the system mode, as defined by enum MAV_MODE. There is no target component id as the mode is by definition for the overall aircraft, not only for one component.
type SetMode struct {
	TargetSystem uint8
	BaseMode     uint8
	CustomMode   uint32
}

func (self *SetMode) MsgID() uint8 {
	return 11
}

func (self *SetMode) MsgName() string {
	return "SetMode"
}

func (self *SetMode) MsgSize() uint8 {
	return 0
}

// Request to read the onboard parameter with the param_id string id. Onboard parameters are stored as key[const char*] -> value[float]. This allows to send a parameter to any other component (such as the GCS) without the need of previous knowledge of possible parameter names. Thus the same GCS can store different parameters for different autopilots. See also http://qgroundcontrol.org/parameter_interface for a full documentation of QGroundControl and IMU code.
type ParamRequestRead struct {
	TargetSystem    uint8
	TargetComponent uint8
	ParamId         Char16
	ParamIndex      int16
}

func (self *ParamRequestRead) MsgID() uint8 {
	return 20
}

func (self *ParamRequestRead) MsgName() string {
	return "ParamRequestRead"
}

func (self *ParamRequestRead) MsgSize() uint8 {
	return 0
}

// Request all parameters of this component. After his request, all parameters are emitted.
type ParamRequestList struct {
	TargetSystem    uint8
	TargetComponent uint8
}

func (self *ParamRequestList) MsgID() uint8 {
	return 21
}

func (self *ParamRequestList) MsgName() string {
	return "ParamRequestList"
}

func (self *ParamRequestList) MsgSize() uint8 {
	return 0
}

// Emit the value of a onboard parameter. The inclusion of param_count and param_index in the message allows the recipient to keep track of received parameters and allows him to re-request missing parameters after a loss or timeout.
type ParamValue struct {
	ParamId    Char16
	ParamValue float32
	ParamType  uint8
	ParamCount uint16
	ParamIndex uint16
}

func (self *ParamValue) MsgID() uint8 {
	return 22
}

func (self *ParamValue) MsgName() string {
	return "ParamValue"
}

func (self *ParamValue) MsgSize() uint8 {
	return 0
}

// Set a parameter value TEMPORARILY to RAM. It will be reset to default on system reboot. Send the ACTION MAV_ACTION_STORAGE_WRITE to PERMANENTLY write the RAM contents to EEPROM. IMPORTANT: The receiving component should acknowledge the new parameter value by sending a param_value message to all communication partners. This will also ensure that multiple GCS all have an up-to-date list of all parameters. If the sending GCS did not receive a PARAM_VALUE message within its timeout time, it should re-send the PARAM_SET message.
type ParamSet struct {
	TargetSystem    uint8
	TargetComponent uint8
	ParamId         Char16
	ParamValue      float32
	ParamType       uint8
}

func (self *ParamSet) MsgID() uint8 {
	return 23
}

func (self *ParamSet) MsgName() string {
	return "ParamSet"
}

func (self *ParamSet) MsgSize() uint8 {
	return 0
}

// The global position, as returned by the Global Positioning System (GPS). This is
//                 NOT the global position estimate of the sytem, but rather a RAW sensor value. See message GLOBAL_POSITION for the global position estimate. Coordinate frame is right-handed, Z-axis up (GPS frame).
type GpsRawInt struct {
	TimeUsec          uint64
	FixType           uint8
	Lat               int32
	Lon               int32
	Alt               int32
	Eph               uint16
	Epv               uint16
	Vel               uint16
	Cog               uint16
	SatellitesVisible uint8
}

func (self *GpsRawInt) MsgID() uint8 {
	return 24
}

func (self *GpsRawInt) MsgName() string {
	return "GpsRawInt"
}

func (self *GpsRawInt) MsgSize() uint8 {
	return 0
}

// The positioning status, as reported by GPS. This message is intended to display status information about each satellite visible to the receiver. See message GLOBAL_POSITION for the global position estimate. This message can contain information for up to 20 satellites.
type GpsStatus struct {
	SatellitesVisible  uint8
	SatellitePrn       [20]uint8
	SatelliteUsed      [20]uint8
	SatelliteElevation [20]uint8
	SatelliteAzimuth   [20]uint8
	SatelliteSnr       [20]uint8
}

func (self *GpsStatus) MsgID() uint8 {
	return 25
}

func (self *GpsStatus) MsgName() string {
	return "GpsStatus"
}

func (self *GpsStatus) MsgSize() uint8 {
	return 0
}

// The RAW IMU readings for the usual 9DOF sensor setup. This message should contain the scaled values to the described units
type ScaledImu struct {
	TimeBootMs uint32
	Xacc       int16
	Yacc       int16
	Zacc       int16
	Xgyro      int16
	Ygyro      int16
	Zgyro      int16
	Xmag       int16
	Ymag       int16
	Zmag       int16
}

func (self *ScaledImu) MsgID() uint8 {
	return 26
}

func (self *ScaledImu) MsgName() string {
	return "ScaledImu"
}

func (self *ScaledImu) MsgSize() uint8 {
	return 0
}

// The RAW IMU readings for the usual 9DOF sensor setup. This message should always contain the true raw values without any scaling to allow data capture and system debugging.
type RawImu struct {
	TimeUsec uint64
	Xacc     int16
	Yacc     int16
	Zacc     int16
	Xgyro    int16
	Ygyro    int16
	Zgyro    int16
	Xmag     int16
	Ymag     int16
	Zmag     int16
}

func (self *RawImu) MsgID() uint8 {
	return 27
}

func (self *RawImu) MsgName() string {
	return "RawImu"
}

func (self *RawImu) MsgSize() uint8 {
	return 0
}

// The RAW pressure readings for the typical setup of one absolute pressure and one differential pressure sensor. The sensor values should be the raw, UNSCALED ADC values.
type RawPressure struct {
	TimeUsec    uint64
	PressAbs    int16
	PressDiff1  int16
	PressDiff2  int16
	Temperature int16
}

func (self *RawPressure) MsgID() uint8 {
	return 28
}

func (self *RawPressure) MsgName() string {
	return "RawPressure"
}

func (self *RawPressure) MsgSize() uint8 {
	return 0
}

// The pressure readings for the typical setup of one absolute and differential pressure sensor. The units are as specified in each field.
type ScaledPressure struct {
	TimeBootMs  uint32
	PressAbs    float32
	PressDiff   float32
	Temperature int16
}

func (self *ScaledPressure) MsgID() uint8 {
	return 29
}

func (self *ScaledPressure) MsgName() string {
	return "ScaledPressure"
}

func (self *ScaledPressure) MsgSize() uint8 {
	return 0
}

// The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right).
type Attitude struct {
	TimeBootMs uint32
	Roll       float32
	Pitch      float32
	Yaw        float32
	Rollspeed  float32
	Pitchspeed float32
	Yawspeed   float32
}

func (self *Attitude) MsgID() uint8 {
	return 30
}

func (self *Attitude) MsgName() string {
	return "Attitude"
}

func (self *Attitude) MsgSize() uint8 {
	return 0
}

// The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right), expressed as quaternion. Quaternion order is w, x, y, z and a zero rotation would be expressed as (1 0 0 0).
type AttitudeQuaternion struct {
	TimeBootMs uint32
	Q1         float32
	Q2         float32
	Q3         float32
	Q4         float32
	Rollspeed  float32
	Pitchspeed float32
	Yawspeed   float32
}

func (self *AttitudeQuaternion) MsgID() uint8 {
	return 31
}

func (self *AttitudeQuaternion) MsgName() string {
	return "AttitudeQuaternion"
}

func (self *AttitudeQuaternion) MsgSize() uint8 {
	return 0
}

// The filtered local position (e.g. fused computer vision and accelerometers). Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)
type LocalPositionNed struct {
	TimeBootMs uint32
	X          float32
	Y          float32
	Z          float32
	Vx         float32
	Vy         float32
	Vz         float32
}

func (self *LocalPositionNed) MsgID() uint8 {
	return 32
}

func (self *LocalPositionNed) MsgName() string {
	return "LocalPositionNed"
}

func (self *LocalPositionNed) MsgSize() uint8 {
	return 0
}

// The filtered global position (e.g. fused GPS and accelerometers). The position is in GPS-frame (right-handed, Z-up). It
//                is designed as scaled integer message since the resolution of float is not sufficient.
type GlobalPositionInt struct {
	TimeBootMs  uint32
	Lat         int32
	Lon         int32
	Alt         int32
	RelativeAlt int32
	Vx          int16
	Vy          int16
	Vz          int16
	Hdg         uint16
}

func (self *GlobalPositionInt) MsgID() uint8 {
	return 33
}

func (self *GlobalPositionInt) MsgName() string {
	return "GlobalPositionInt"
}

func (self *GlobalPositionInt) MsgSize() uint8 {
	return 0
}

// The scaled values of the RC channels received. (-100%) -10000, (0%) 0, (100%) 10000. Channels that are inactive should be set to UINT16_MAX.
type RcChannelsScaled struct {
	TimeBootMs  uint32
	Port        uint8
	Chan1Scaled int16
	Chan2Scaled int16
	Chan3Scaled int16
	Chan4Scaled int16
	Chan5Scaled int16
	Chan6Scaled int16
	Chan7Scaled int16
	Chan8Scaled int16
	Rssi        uint8
}

func (self *RcChannelsScaled) MsgID() uint8 {
	return 34
}

func (self *RcChannelsScaled) MsgName() string {
	return "RcChannelsScaled"
}

func (self *RcChannelsScaled) MsgSize() uint8 {
	return 0
}

// The RAW values of the RC channels received. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification.
type RcChannelsRaw struct {
	TimeBootMs uint32
	Port       uint8
	Chan1Raw   uint16
	Chan2Raw   uint16
	Chan3Raw   uint16
	Chan4Raw   uint16
	Chan5Raw   uint16
	Chan6Raw   uint16
	Chan7Raw   uint16
	Chan8Raw   uint16
	Rssi       uint8
}

func (self *RcChannelsRaw) MsgID() uint8 {
	return 35
}

func (self *RcChannelsRaw) MsgName() string {
	return "RcChannelsRaw"
}

func (self *RcChannelsRaw) MsgSize() uint8 {
	return 0
}

// The RAW values of the servo outputs (for RC input from the remote, use the RC_CHANNELS messages). The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%.
type ServoOutputRaw struct {
	TimeUsec  uint32
	Port      uint8
	Servo1Raw uint16
	Servo2Raw uint16
	Servo3Raw uint16
	Servo4Raw uint16
	Servo5Raw uint16
	Servo6Raw uint16
	Servo7Raw uint16
	Servo8Raw uint16
}

func (self *ServoOutputRaw) MsgID() uint8 {
	return 36
}

func (self *ServoOutputRaw) MsgName() string {
	return "ServoOutputRaw"
}

func (self *ServoOutputRaw) MsgSize() uint8 {
	return 0
}

// Request a partial list of mission items from the system/component. http://qgroundcontrol.org/mavlink/waypoint_protocol. If start and end index are the same, just send one waypoint.
type MissionRequestPartialList struct {
	TargetSystem    uint8
	TargetComponent uint8
	StartIndex      int16
	EndIndex        int16
}

func (self *MissionRequestPartialList) MsgID() uint8 {
	return 37
}

func (self *MissionRequestPartialList) MsgName() string {
	return "MissionRequestPartialList"
}

func (self *MissionRequestPartialList) MsgSize() uint8 {
	return 0
}

// This message is sent to the MAV to write a partial list. If start index == end index, only one item will be transmitted / updated. If the start index is NOT 0 and above the current list size, this request should be REJECTED!
type MissionWritePartialList struct {
	TargetSystem    uint8
	TargetComponent uint8
	StartIndex      int16
	EndIndex        int16
}

func (self *MissionWritePartialList) MsgID() uint8 {
	return 38
}

func (self *MissionWritePartialList) MsgName() string {
	return "MissionWritePartialList"
}

func (self *MissionWritePartialList) MsgSize() uint8 {
	return 0
}

// Message encoding a mission item. This message is emitted to announce
//                 the presence of a mission item and to set a mission item on the system. The mission item can be either in x, y, z meters (type: LOCAL) or x:lat, y:lon, z:altitude. Local frame is Z-down, right handed (NED), global frame is Z-up, right handed (ENU). See also http://qgroundcontrol.org/mavlink/waypoint_protocol.
type MissionItem struct {
	TargetSystem    uint8
	TargetComponent uint8
	Seq             uint16
	Frame           uint8
	Command         uint16
	Current         uint8
	Autocontinue    uint8
	Param1          float32
	Param2          float32
	Param3          float32
	Param4          float32
	X               float32
	Y               float32
	Z               float32
}

func (self *MissionItem) MsgID() uint8 {
	return 39
}

func (self *MissionItem) MsgName() string {
	return "MissionItem"
}

func (self *MissionItem) MsgSize() uint8 {
	return 0
}

// Request the information of the mission item with the sequence number seq. The response of the system to this message should be a MISSION_ITEM message. http://qgroundcontrol.org/mavlink/waypoint_protocol
type MissionRequest struct {
	TargetSystem    uint8
	TargetComponent uint8
	Seq             uint16
}

func (self *MissionRequest) MsgID() uint8 {
	return 40
}

func (self *MissionRequest) MsgName() string {
	return "MissionRequest"
}

func (self *MissionRequest) MsgSize() uint8 {
	return 0
}

// Set the mission item with sequence number seq as current item. This means that the MAV will continue to this mission item on the shortest path (not following the mission items in-between).
type MissionSetCurrent struct {
	TargetSystem    uint8
	TargetComponent uint8
	Seq             uint16
}

func (self *MissionSetCurrent) MsgID() uint8 {
	return 41
}

func (self *MissionSetCurrent) MsgName() string {
	return "MissionSetCurrent"
}

func (self *MissionSetCurrent) MsgSize() uint8 {
	return 0
}

// Message that announces the sequence number of the current active mission item. The MAV will fly towards this mission item.
type MissionCurrent struct {
	Seq uint16
}

func (self *MissionCurrent) MsgID() uint8 {
	return 42
}

func (self *MissionCurrent) MsgName() string {
	return "MissionCurrent"
}

func (self *MissionCurrent) MsgSize() uint8 {
	return 0
}

// Request the overall list of mission items from the system/component.
type MissionRequestList struct {
	TargetSystem    uint8
	TargetComponent uint8
}

func (self *MissionRequestList) MsgID() uint8 {
	return 43
}

func (self *MissionRequestList) MsgName() string {
	return "MissionRequestList"
}

func (self *MissionRequestList) MsgSize() uint8 {
	return 0
}

// This message is emitted as response to MISSION_REQUEST_LIST by the MAV and to initiate a write transaction. The GCS can then request the individual mission item based on the knowledge of the total number of MISSIONs.
type MissionCount struct {
	TargetSystem    uint8
	TargetComponent uint8
	Count           uint16
}

func (self *MissionCount) MsgID() uint8 {
	return 44
}

func (self *MissionCount) MsgName() string {
	return "MissionCount"
}

func (self *MissionCount) MsgSize() uint8 {
	return 0
}

// Delete all mission items at once.
type MissionClearAll struct {
	TargetSystem    uint8
	TargetComponent uint8
}

func (self *MissionClearAll) MsgID() uint8 {
	return 45
}

func (self *MissionClearAll) MsgName() string {
	return "MissionClearAll"
}

func (self *MissionClearAll) MsgSize() uint8 {
	return 0
}

// A certain mission item has been reached. The system will either hold this position (or circle on the orbit) or (if the autocontinue on the WP was set) continue to the next MISSION.
type MissionItemReached struct {
	Seq uint16
}

func (self *MissionItemReached) MsgID() uint8 {
	return 46
}

func (self *MissionItemReached) MsgName() string {
	return "MissionItemReached"
}

func (self *MissionItemReached) MsgSize() uint8 {
	return 0
}

// Ack message during MISSION handling. The type field states if this message is a positive ack (type=0) or if an error happened (type=non-zero).
type MissionAck struct {
	TargetSystem    uint8
	TargetComponent uint8
	Type            uint8
}

func (self *MissionAck) MsgID() uint8 {
	return 47
}

func (self *MissionAck) MsgName() string {
	return "MissionAck"
}

func (self *MissionAck) MsgSize() uint8 {
	return 0
}

// As local waypoints exist, the global MISSION reference allows to transform between the local coordinate frame and the global (GPS) coordinate frame. This can be necessary when e.g. in- and outdoor settings are connected and the MAV should move from in- to outdoor.
type SetGpsGlobalOrigin struct {
	TargetSystem uint8
	Latitude     int32
	Longitude    int32
	Altitude     int32
}

func (self *SetGpsGlobalOrigin) MsgID() uint8 {
	return 48
}

func (self *SetGpsGlobalOrigin) MsgName() string {
	return "SetGpsGlobalOrigin"
}

func (self *SetGpsGlobalOrigin) MsgSize() uint8 {
	return 0
}

// Once the MAV sets a new GPS-Local correspondence, this message announces the origin (0,0,0) position
type GpsGlobalOrigin struct {
	Latitude  int32
	Longitude int32
	Altitude  int32
}

func (self *GpsGlobalOrigin) MsgID() uint8 {
	return 49
}

func (self *GpsGlobalOrigin) MsgName() string {
	return "GpsGlobalOrigin"
}

func (self *GpsGlobalOrigin) MsgSize() uint8 {
	return 0
}

// Set a safety zone (volume), which is defined by two corners of a cube. This message can be used to tell the MAV which setpoints/MISSIONs to accept and which to reject. Safety areas are often enforced by national or competition regulations.
type SafetySetAllowedArea struct {
	TargetSystem    uint8
	TargetComponent uint8
	Frame           uint8
	P1x             float32
	P1y             float32
	P1z             float32
	P2x             float32
	P2y             float32
	P2z             float32
}

func (self *SafetySetAllowedArea) MsgID() uint8 {
	return 54
}

func (self *SafetySetAllowedArea) MsgName() string {
	return "SafetySetAllowedArea"
}

func (self *SafetySetAllowedArea) MsgSize() uint8 {
	return 0
}

// Read out the safety zone the MAV currently assumes.
type SafetyAllowedArea struct {
	Frame uint8
	P1x   float32
	P1y   float32
	P1z   float32
	P2x   float32
	P2y   float32
	P2z   float32
}

func (self *SafetyAllowedArea) MsgID() uint8 {
	return 55
}

func (self *SafetyAllowedArea) MsgName() string {
	return "SafetyAllowedArea"
}

func (self *SafetyAllowedArea) MsgSize() uint8 {
	return 0
}

// The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right), expressed as quaternion. Quaternion order is w, x, y, z and a zero rotation would be expressed as (1 0 0 0).
type AttitudeQuaternionCov struct {
	TimeBootMs uint32
	Q          [4]float32
	Rollspeed  float32
	Pitchspeed float32
	Yawspeed   float32
	Covariance [9]float32
}

func (self *AttitudeQuaternionCov) MsgID() uint8 {
	return 61
}

func (self *AttitudeQuaternionCov) MsgName() string {
	return "AttitudeQuaternionCov"
}

func (self *AttitudeQuaternionCov) MsgSize() uint8 {
	return 0
}

// Outputs of the APM navigation controller. The primary use of this message is to check the response and signs of the controller before actual flight and to assist with tuning controller parameters.
type NavControllerOutput struct {
	NavRoll       float32
	NavPitch      float32
	NavBearing    int16
	TargetBearing int16
	WpDist        uint16
	AltError      float32
	AspdError     float32
	XtrackError   float32
}

func (self *NavControllerOutput) MsgID() uint8 {
	return 62
}

func (self *NavControllerOutput) MsgName() string {
	return "NavControllerOutput"
}

func (self *NavControllerOutput) MsgSize() uint8 {
	return 0
}

// The filtered global position (e.g. fused GPS and accelerometers). The position is in GPS-frame (right-handed, Z-up). It  is designed as scaled integer message since the resolution of float is not sufficient. NOTE: This message is intended for onboard networks / companion computers and higher-bandwidth links and optimized for accuracy and completeness. Please use the GLOBAL_POSITION_INT message for a minimal subset.
type GlobalPositionIntCov struct {
	TimeBootMs    uint32
	TimeUtc       uint64
	EstimatorType uint8
	Lat           int32
	Lon           int32
	Alt           int32
	RelativeAlt   int32
	Vx            float32
	Vy            float32
	Vz            float32
	Covariance    [36]float32
}

func (self *GlobalPositionIntCov) MsgID() uint8 {
	return 63
}

func (self *GlobalPositionIntCov) MsgName() string {
	return "GlobalPositionIntCov"
}

func (self *GlobalPositionIntCov) MsgSize() uint8 {
	return 0
}

// The filtered local position (e.g. fused computer vision and accelerometers). Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)
type LocalPositionNedCov struct {
	TimeBootMs    uint32
	TimeUtc       uint64
	EstimatorType uint8
	X             float32
	Y             float32
	Z             float32
	Vx            float32
	Vy            float32
	Vz            float32
	Covariance    [36]float32
}

func (self *LocalPositionNedCov) MsgID() uint8 {
	return 64
}

func (self *LocalPositionNedCov) MsgName() string {
	return "LocalPositionNedCov"
}

func (self *LocalPositionNedCov) MsgSize() uint8 {
	return 0
}

// The PPM values of the RC channels received. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification.
type RcChannels struct {
	TimeBootMs uint32
	Chancount  uint8
	Chan1Raw   uint16
	Chan2Raw   uint16
	Chan3Raw   uint16
	Chan4Raw   uint16
	Chan5Raw   uint16
	Chan6Raw   uint16
	Chan7Raw   uint16
	Chan8Raw   uint16
	Chan9Raw   uint16
	Chan10Raw  uint16
	Chan11Raw  uint16
	Chan12Raw  uint16
	Chan13Raw  uint16
	Chan14Raw  uint16
	Chan15Raw  uint16
	Chan16Raw  uint16
	Chan17Raw  uint16
	Chan18Raw  uint16
	Rssi       uint8
}

func (self *RcChannels) MsgID() uint8 {
	return 65
}

func (self *RcChannels) MsgName() string {
	return "RcChannels"
}

func (self *RcChannels) MsgSize() uint8 {
	return 0
}

//
type RequestDataStream struct {
	TargetSystem    uint8
	TargetComponent uint8
	ReqStreamId     uint8
	ReqMessageRate  uint16
	StartStop       uint8
}

func (self *RequestDataStream) MsgID() uint8 {
	return 66
}

func (self *RequestDataStream) MsgName() string {
	return "RequestDataStream"
}

func (self *RequestDataStream) MsgSize() uint8 {
	return 0
}

//
type DataStream struct {
	StreamId    uint8
	MessageRate uint16
	OnOff       uint8
}

func (self *DataStream) MsgID() uint8 {
	return 67
}

func (self *DataStream) MsgName() string {
	return "DataStream"
}

func (self *DataStream) MsgSize() uint8 {
	return 0
}

// This message provides an API for manually controlling the vehicle using standard joystick axes nomenclature, along with a joystick-like input device. Unused axes can be disabled an buttons are also transmit as boolean values of their
type ManualControl struct {
	Target  uint8
	X       int16
	Y       int16
	Z       int16
	R       int16
	Buttons uint16
}

func (self *ManualControl) MsgID() uint8 {
	return 69
}

func (self *ManualControl) MsgName() string {
	return "ManualControl"
}

func (self *ManualControl) MsgSize() uint8 {
	return 0
}

// The RAW values of the RC channels sent to the MAV to override info received from the RC radio. A value of UINT16_MAX means no change to that channel. A value of 0 means control of that channel should be released back to the RC radio. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification.
type RcChannelsOverride struct {
	TargetSystem    uint8
	TargetComponent uint8
	Chan1Raw        uint16
	Chan2Raw        uint16
	Chan3Raw        uint16
	Chan4Raw        uint16
	Chan5Raw        uint16
	Chan6Raw        uint16
	Chan7Raw        uint16
	Chan8Raw        uint16
}

func (self *RcChannelsOverride) MsgID() uint8 {
	return 70
}

func (self *RcChannelsOverride) MsgName() string {
	return "RcChannelsOverride"
}

func (self *RcChannelsOverride) MsgSize() uint8 {
	return 0
}

// Message encoding a mission item. This message is emitted to announce
//                 the presence of a mission item and to set a mission item on the system. The mission item can be either in x, y, z meters (type: LOCAL) or x:lat, y:lon, z:altitude. Local frame is Z-down, right handed (NED), global frame is Z-up, right handed (ENU). See alsohttp://qgroundcontrol.org/mavlink/waypoint_protocol.
type MissionItemInt struct {
	TargetSystem    uint8
	TargetComponent uint8
	Seq             uint16
	Frame           uint8
	Command         uint16
	Current         uint8
	Autocontinue    uint8
	Param1          float32
	Param2          float32
	Param3          float32
	Param4          float32
	X               int32
	Y               int32
	Z               float32
}

func (self *MissionItemInt) MsgID() uint8 {
	return 73
}

func (self *MissionItemInt) MsgName() string {
	return "MissionItemInt"
}

func (self *MissionItemInt) MsgSize() uint8 {
	return 0
}

// Metrics typically displayed on a HUD for fixed wing aircraft
type VfrHud struct {
	Airspeed    float32
	Groundspeed float32
	Heading     int16
	Throttle    uint16
	Alt         float32
	Climb       float32
}

func (self *VfrHud) MsgID() uint8 {
	return 74
}

func (self *VfrHud) MsgName() string {
	return "VfrHud"
}

func (self *VfrHud) MsgSize() uint8 {
	return 0
}

// Message encoding a command with parameters as scaled integers. Scaling depends on the actual command value.
type CommandInt struct {
	TargetSystem    uint8
	TargetComponent uint8
	Frame           uint8
	Command         uint16
	Current         uint8
	Autocontinue    uint8
	Param1          float32
	Param2          float32
	Param3          float32
	Param4          float32
	X               int32
	Y               int32
	Z               float32
}

func (self *CommandInt) MsgID() uint8 {
	return 75
}

func (self *CommandInt) MsgName() string {
	return "CommandInt"
}

func (self *CommandInt) MsgSize() uint8 {
	return 0
}

// Send a command with up to seven parameters to the MAV
type CommandLong struct {
	TargetSystem    uint8
	TargetComponent uint8
	Command         uint16
	Confirmation    uint8
	Param1          float32
	Param2          float32
	Param3          float32
	Param4          float32
	Param5          float32
	Param6          float32
	Param7          float32
}

func (self *CommandLong) MsgID() uint8 {
	return 76
}

func (self *CommandLong) MsgName() string {
	return "CommandLong"
}

func (self *CommandLong) MsgSize() uint8 {
	return 0
}

// Report status of a command. Includes feedback wether the command was executed.
type CommandAck struct {
	Command uint16
	Result  uint8
}

func (self *CommandAck) MsgID() uint8 {
	return 77
}

func (self *CommandAck) MsgName() string {
	return "CommandAck"
}

func (self *CommandAck) MsgSize() uint8 {
	return 0
}

// Setpoint in roll, pitch, yaw and thrust from the operator
type ManualSetpoint struct {
	TimeBootMs           uint32
	Roll                 float32
	Pitch                float32
	Yaw                  float32
	Thrust               float32
	ModeSwitch           uint8
	ManualOverrideSwitch uint8
}

func (self *ManualSetpoint) MsgID() uint8 {
	return 81
}

func (self *ManualSetpoint) MsgName() string {
	return "ManualSetpoint"
}

func (self *ManualSetpoint) MsgSize() uint8 {
	return 0
}

// Set the vehicle attitude and body angular rates.
type SetAttitudeTarget struct {
	TimeBootMs      uint32
	TargetSystem    uint8
	TargetComponent uint8
	TypeMask        uint8
	Q               [4]float32
	BodyRollRate    float32
	BodyPitchRate   float32
	BodyYawRate     float32
	Thrust          float32
}

func (self *SetAttitudeTarget) MsgID() uint8 {
	return 82
}

func (self *SetAttitudeTarget) MsgName() string {
	return "SetAttitudeTarget"
}

func (self *SetAttitudeTarget) MsgSize() uint8 {
	return 0
}

// Set the vehicle attitude and body angular rates.
type AttitudeTarget struct {
	TimeBootMs    uint32
	TypeMask      uint8
	Q             [4]float32
	BodyRollRate  float32
	BodyPitchRate float32
	BodyYawRate   float32
	Thrust        float32
}

func (self *AttitudeTarget) MsgID() uint8 {
	return 83
}

func (self *AttitudeTarget) MsgName() string {
	return "AttitudeTarget"
}

func (self *AttitudeTarget) MsgSize() uint8 {
	return 0
}

// Set vehicle position, velocity and acceleration setpoint in local frame.
type SetPositionTargetLocalNed struct {
	TimeBootMs      uint32
	TargetSystem    uint8
	TargetComponent uint8
	CoordinateFrame uint8
	TypeMask        uint16
	X               float32
	Y               float32
	Z               float32
	Vx              float32
	Vy              float32
	Vz              float32
	Afx             float32
	Afy             float32
	Afz             float32
	Yaw             float32
	YawRate         float32
}

func (self *SetPositionTargetLocalNed) MsgID() uint8 {
	return 84
}

func (self *SetPositionTargetLocalNed) MsgName() string {
	return "SetPositionTargetLocalNed"
}

func (self *SetPositionTargetLocalNed) MsgSize() uint8 {
	return 0
}

// Set vehicle position, velocity and acceleration setpoint in local frame.
type PositionTargetLocalNed struct {
	TimeBootMs      uint32
	CoordinateFrame uint8
	TypeMask        uint16
	X               float32
	Y               float32
	Z               float32
	Vx              float32
	Vy              float32
	Vz              float32
	Afx             float32
	Afy             float32
	Afz             float32
	Yaw             float32
	YawRate         float32
}

func (self *PositionTargetLocalNed) MsgID() uint8 {
	return 85
}

func (self *PositionTargetLocalNed) MsgName() string {
	return "PositionTargetLocalNed"
}

func (self *PositionTargetLocalNed) MsgSize() uint8 {
	return 0
}

// Set vehicle position, velocity and acceleration setpoint in the WGS84 coordinate system.
type SetPositionTargetGlobalInt struct {
	TimeBootMs      uint32
	TargetSystem    uint8
	TargetComponent uint8
	CoordinateFrame uint8
	TypeMask        uint16
	LatInt          int32
	LonInt          int32
	Alt             float32
	Vx              float32
	Vy              float32
	Vz              float32
	Afx             float32
	Afy             float32
	Afz             float32
	Yaw             float32
	YawRate         float32
}

func (self *SetPositionTargetGlobalInt) MsgID() uint8 {
	return 86
}

func (self *SetPositionTargetGlobalInt) MsgName() string {
	return "SetPositionTargetGlobalInt"
}

func (self *SetPositionTargetGlobalInt) MsgSize() uint8 {
	return 0
}

// Set vehicle position, velocity and acceleration setpoint in the WGS84 coordinate system.
type PositionTargetGlobalInt struct {
	TimeBootMs      uint32
	CoordinateFrame uint8
	TypeMask        uint16
	LatInt          int32
	LonInt          int32
	Alt             float32
	Vx              float32
	Vy              float32
	Vz              float32
	Afx             float32
	Afy             float32
	Afz             float32
	Yaw             float32
	YawRate         float32
}

func (self *PositionTargetGlobalInt) MsgID() uint8 {
	return 87
}

func (self *PositionTargetGlobalInt) MsgName() string {
	return "PositionTargetGlobalInt"
}

func (self *PositionTargetGlobalInt) MsgSize() uint8 {
	return 0
}

// The offset in X, Y, Z and yaw between the LOCAL_POSITION_NED messages of MAV X and the global coordinate frame in NED coordinates. Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)
type LocalPositionNedSystemGlobalOffset struct {
	TimeBootMs uint32
	X          float32
	Y          float32
	Z          float32
	Roll       float32
	Pitch      float32
	Yaw        float32
}

func (self *LocalPositionNedSystemGlobalOffset) MsgID() uint8 {
	return 89
}

func (self *LocalPositionNedSystemGlobalOffset) MsgName() string {
	return "LocalPositionNedSystemGlobalOffset"
}

func (self *LocalPositionNedSystemGlobalOffset) MsgSize() uint8 {
	return 0
}

// DEPRECATED PACKET! Suffers from missing airspeed fields and singularities due to Euler angles. Please use HIL_STATE_QUATERNION instead. Sent from simulation to autopilot. This packet is useful for high throughput applications such as hardware in the loop simulations.
type HilState struct {
	TimeUsec   uint64
	Roll       float32
	Pitch      float32
	Yaw        float32
	Rollspeed  float32
	Pitchspeed float32
	Yawspeed   float32
	Lat        int32
	Lon        int32
	Alt        int32
	Vx         int16
	Vy         int16
	Vz         int16
	Xacc       int16
	Yacc       int16
	Zacc       int16
}

func (self *HilState) MsgID() uint8 {
	return 90
}

func (self *HilState) MsgName() string {
	return "HilState"
}

func (self *HilState) MsgSize() uint8 {
	return 0
}

// Sent from autopilot to simulation. Hardware in the loop control outputs
type HilControls struct {
	TimeUsec      uint64
	RollAilerons  float32
	PitchElevator float32
	YawRudder     float32
	Throttle      float32
	Aux1          float32
	Aux2          float32
	Aux3          float32
	Aux4          float32
	Mode          uint8
	NavMode       uint8
}

func (self *HilControls) MsgID() uint8 {
	return 91
}

func (self *HilControls) MsgName() string {
	return "HilControls"
}

func (self *HilControls) MsgSize() uint8 {
	return 0
}

// Sent from simulation to autopilot. The RAW values of the RC channels received. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification.
type HilRcInputsRaw struct {
	TimeUsec  uint64
	Chan1Raw  uint16
	Chan2Raw  uint16
	Chan3Raw  uint16
	Chan4Raw  uint16
	Chan5Raw  uint16
	Chan6Raw  uint16
	Chan7Raw  uint16
	Chan8Raw  uint16
	Chan9Raw  uint16
	Chan10Raw uint16
	Chan11Raw uint16
	Chan12Raw uint16
	Rssi      uint8
}

func (self *HilRcInputsRaw) MsgID() uint8 {
	return 92
}

func (self *HilRcInputsRaw) MsgName() string {
	return "HilRcInputsRaw"
}

func (self *HilRcInputsRaw) MsgSize() uint8 {
	return 0
}

// Optical flow from a flow sensor (e.g. optical mouse sensor)
type OpticalFlow struct {
	TimeUsec       uint64
	SensorId       uint8
	FlowX          int16
	FlowY          int16
	FlowCompMX     float32
	FlowCompMY     float32
	Quality        uint8
	GroundDistance float32
}

func (self *OpticalFlow) MsgID() uint8 {
	return 100
}

func (self *OpticalFlow) MsgName() string {
	return "OpticalFlow"
}

func (self *OpticalFlow) MsgSize() uint8 {
	return 0
}

//
type GlobalVisionPositionEstimate struct {
	Usec  uint64
	X     float32
	Y     float32
	Z     float32
	Roll  float32
	Pitch float32
	Yaw   float32
}

func (self *GlobalVisionPositionEstimate) MsgID() uint8 {
	return 101
}

func (self *GlobalVisionPositionEstimate) MsgName() string {
	return "GlobalVisionPositionEstimate"
}

func (self *GlobalVisionPositionEstimate) MsgSize() uint8 {
	return 0
}

//
type VisionPositionEstimate struct {
	Usec  uint64
	X     float32
	Y     float32
	Z     float32
	Roll  float32
	Pitch float32
	Yaw   float32
}

func (self *VisionPositionEstimate) MsgID() uint8 {
	return 102
}

func (self *VisionPositionEstimate) MsgName() string {
	return "VisionPositionEstimate"
}

func (self *VisionPositionEstimate) MsgSize() uint8 {
	return 0
}

//
type VisionSpeedEstimate struct {
	Usec uint64
	X    float32
	Y    float32
	Z    float32
}

func (self *VisionSpeedEstimate) MsgID() uint8 {
	return 103
}

func (self *VisionSpeedEstimate) MsgName() string {
	return "VisionSpeedEstimate"
}

func (self *VisionSpeedEstimate) MsgSize() uint8 {
	return 0
}

//
type ViconPositionEstimate struct {
	Usec  uint64
	X     float32
	Y     float32
	Z     float32
	Roll  float32
	Pitch float32
	Yaw   float32
}

func (self *ViconPositionEstimate) MsgID() uint8 {
	return 104
}

func (self *ViconPositionEstimate) MsgName() string {
	return "ViconPositionEstimate"
}

func (self *ViconPositionEstimate) MsgSize() uint8 {
	return 0
}

// The IMU readings in SI units in NED body frame
type HighresImu struct {
	TimeUsec      uint64
	Xacc          float32
	Yacc          float32
	Zacc          float32
	Xgyro         float32
	Ygyro         float32
	Zgyro         float32
	Xmag          float32
	Ymag          float32
	Zmag          float32
	AbsPressure   float32
	DiffPressure  float32
	PressureAlt   float32
	Temperature   float32
	FieldsUpdated uint16
}

func (self *HighresImu) MsgID() uint8 {
	return 105
}

func (self *HighresImu) MsgName() string {
	return "HighresImu"
}

func (self *HighresImu) MsgSize() uint8 {
	return 0
}

// Optical flow from an angular rate flow sensor (e.g. PX4FLOW or mouse sensor)
type OpticalFlowRad struct {
	TimeUsec            uint64
	SensorId            uint8
	IntegrationTimeUs   uint32
	IntegratedX         float32
	IntegratedY         float32
	IntegratedXgyro     float32
	IntegratedYgyro     float32
	IntegratedZgyro     float32
	Temperature         int16
	Quality             uint8
	TimeDeltaDistanceUs uint32
	Distance            float32
}

func (self *OpticalFlowRad) MsgID() uint8 {
	return 106
}

func (self *OpticalFlowRad) MsgName() string {
	return "OpticalFlowRad"
}

func (self *OpticalFlowRad) MsgSize() uint8 {
	return 0
}

// The IMU readings in SI units in NED body frame
type HilSensor struct {
	TimeUsec      uint64
	Xacc          float32
	Yacc          float32
	Zacc          float32
	Xgyro         float32
	Ygyro         float32
	Zgyro         float32
	Xmag          float32
	Ymag          float32
	Zmag          float32
	AbsPressure   float32
	DiffPressure  float32
	PressureAlt   float32
	Temperature   float32
	FieldsUpdated uint32
}

func (self *HilSensor) MsgID() uint8 {
	return 107
}

func (self *HilSensor) MsgName() string {
	return "HilSensor"
}

func (self *HilSensor) MsgSize() uint8 {
	return 0
}

// Status of simulation environment, if used
type SimState struct {
	Q1         float32
	Q2         float32
	Q3         float32
	Q4         float32
	Roll       float32
	Pitch      float32
	Yaw        float32
	Xacc       float32
	Yacc       float32
	Zacc       float32
	Xgyro      float32
	Ygyro      float32
	Zgyro      float32
	Lat        float32
	Lon        float32
	Alt        float32
	StdDevHorz float32
	StdDevVert float32
	Vn         float32
	Ve         float32
	Vd         float32
}

func (self *SimState) MsgID() uint8 {
	return 108
}

func (self *SimState) MsgName() string {
	return "SimState"
}

func (self *SimState) MsgSize() uint8 {
	return 0
}

// Status generated by radio
type RadioStatus struct {
	Rssi     uint8
	Remrssi  uint8
	Txbuf    uint8
	Noise    uint8
	Remnoise uint8
	Rxerrors uint16
	Fixed    uint16
}

func (self *RadioStatus) MsgID() uint8 {
	return 109
}

func (self *RadioStatus) MsgName() string {
	return "RadioStatus"
}

func (self *RadioStatus) MsgSize() uint8 {
	return 0
}

// File transfer message
type FileTransferProtocol struct {
	TargetNetwork   uint8
	TargetSystem    uint8
	TargetComponent uint8
	Payload         [251]uint8
}

func (self *FileTransferProtocol) MsgID() uint8 {
	return 110
}

func (self *FileTransferProtocol) MsgName() string {
	return "FileTransferProtocol"
}

func (self *FileTransferProtocol) MsgSize() uint8 {
	return 0
}

// Time synchronization message.
type Timesync struct {
	Tc1 int64
	Ts1 int64
}

func (self *Timesync) MsgID() uint8 {
	return 111
}

func (self *Timesync) MsgName() string {
	return "Timesync"
}

func (self *Timesync) MsgSize() uint8 {
	return 0
}

// The global position, as returned by the Global Positioning System (GPS). This is
//                  NOT the global position estimate of the sytem, but rather a RAW sensor value. See message GLOBAL_POSITION for the global position estimate. Coordinate frame is right-handed, Z-axis up (GPS frame).
type HilGps struct {
	TimeUsec          uint64
	FixType           uint8
	Lat               int32
	Lon               int32
	Alt               int32
	Eph               uint16
	Epv               uint16
	Vel               uint16
	Vn                int16
	Ve                int16
	Vd                int16
	Cog               uint16
	SatellitesVisible uint8
}

func (self *HilGps) MsgID() uint8 {
	return 113
}

func (self *HilGps) MsgName() string {
	return "HilGps"
}

func (self *HilGps) MsgSize() uint8 {
	return 0
}

// Simulated optical flow from a flow sensor (e.g. PX4FLOW or optical mouse sensor)
type HilOpticalFlow struct {
	TimeUsec            uint64
	SensorId            uint8
	IntegrationTimeUs   uint32
	IntegratedX         float32
	IntegratedY         float32
	IntegratedXgyro     float32
	IntegratedYgyro     float32
	IntegratedZgyro     float32
	Temperature         int16
	Quality             uint8
	TimeDeltaDistanceUs uint32
	Distance            float32
}

func (self *HilOpticalFlow) MsgID() uint8 {
	return 114
}

func (self *HilOpticalFlow) MsgName() string {
	return "HilOpticalFlow"
}

func (self *HilOpticalFlow) MsgSize() uint8 {
	return 0
}

// Sent from simulation to autopilot, avoids in contrast to HIL_STATE singularities. This packet is useful for high throughput applications such as hardware in the loop simulations.
type HilStateQuaternion struct {
	TimeUsec           uint64
	AttitudeQuaternion [4]float32
	Rollspeed          float32
	Pitchspeed         float32
	Yawspeed           float32
	Lat                int32
	Lon                int32
	Alt                int32
	Vx                 int16
	Vy                 int16
	Vz                 int16
	IndAirspeed        uint16
	TrueAirspeed       uint16
	Xacc               int16
	Yacc               int16
	Zacc               int16
}

func (self *HilStateQuaternion) MsgID() uint8 {
	return 115
}

func (self *HilStateQuaternion) MsgName() string {
	return "HilStateQuaternion"
}

func (self *HilStateQuaternion) MsgSize() uint8 {
	return 0
}

// The RAW IMU readings for secondary 9DOF sensor setup. This message should contain the scaled values to the described units
type ScaledImu2 struct {
	TimeBootMs uint32
	Xacc       int16
	Yacc       int16
	Zacc       int16
	Xgyro      int16
	Ygyro      int16
	Zgyro      int16
	Xmag       int16
	Ymag       int16
	Zmag       int16
}

func (self *ScaledImu2) MsgID() uint8 {
	return 116
}

func (self *ScaledImu2) MsgName() string {
	return "ScaledImu2"
}

func (self *ScaledImu2) MsgSize() uint8 {
	return 0
}

// Request a list of available logs. On some systems calling this may stop on-board logging until LOG_REQUEST_END is called.
type LogRequestList struct {
	TargetSystem    uint8
	TargetComponent uint8
	Start           uint16
	End             uint16
}

func (self *LogRequestList) MsgID() uint8 {
	return 117
}

func (self *LogRequestList) MsgName() string {
	return "LogRequestList"
}

func (self *LogRequestList) MsgSize() uint8 {
	return 0
}

// Reply to LOG_REQUEST_LIST
type LogEntry struct {
	Id         uint16
	NumLogs    uint16
	LastLogNum uint16
	TimeUtc    uint32
	Size       uint32
}

func (self *LogEntry) MsgID() uint8 {
	return 118
}

func (self *LogEntry) MsgName() string {
	return "LogEntry"
}

func (self *LogEntry) MsgSize() uint8 {
	return 0
}

// Request a chunk of a log
type LogRequestData struct {
	TargetSystem    uint8
	TargetComponent uint8
	Id              uint16
	Ofs             uint32
	Count           uint32
}

func (self *LogRequestData) MsgID() uint8 {
	return 119
}

func (self *LogRequestData) MsgName() string {
	return "LogRequestData"
}

func (self *LogRequestData) MsgSize() uint8 {
	return 0
}

// Reply to LOG_REQUEST_DATA
type LogData struct {
	Id    uint16
	Ofs   uint32
	Count uint8
	Data  [90]uint8
}

func (self *LogData) MsgID() uint8 {
	return 120
}

func (self *LogData) MsgName() string {
	return "LogData"
}

func (self *LogData) MsgSize() uint8 {
	return 0
}

// Erase all logs
type LogErase struct {
	TargetSystem    uint8
	TargetComponent uint8
}

func (self *LogErase) MsgID() uint8 {
	return 121
}

func (self *LogErase) MsgName() string {
	return "LogErase"
}

func (self *LogErase) MsgSize() uint8 {
	return 0
}

// Stop log transfer and resume normal logging
type LogRequestEnd struct {
	TargetSystem    uint8
	TargetComponent uint8
}

func (self *LogRequestEnd) MsgID() uint8 {
	return 122
}

func (self *LogRequestEnd) MsgName() string {
	return "LogRequestEnd"
}

func (self *LogRequestEnd) MsgSize() uint8 {
	return 0
}

// data for injecting into the onboard GPS (used for DGPS)
type GpsInjectData struct {
	TargetSystem    uint8
	TargetComponent uint8
	Len             uint8
	Data            [110]uint8
}

func (self *GpsInjectData) MsgID() uint8 {
	return 123
}

func (self *GpsInjectData) MsgName() string {
	return "GpsInjectData"
}

func (self *GpsInjectData) MsgSize() uint8 {
	return 0
}

// Second GPS data. Coordinate frame is right-handed, Z-axis up (GPS frame).
type Gps2Raw struct {
	TimeUsec          uint64
	FixType           uint8
	Lat               int32
	Lon               int32
	Alt               int32
	Eph               uint16
	Epv               uint16
	Vel               uint16
	Cog               uint16
	SatellitesVisible uint8
	DgpsNumch         uint8
	DgpsAge           uint32
}

func (self *Gps2Raw) MsgID() uint8 {
	return 124
}

func (self *Gps2Raw) MsgName() string {
	return "Gps2Raw"
}

func (self *Gps2Raw) MsgSize() uint8 {
	return 0
}

// Power supply status
type PowerStatus struct {
	Vcc    uint16
	Vservo uint16
	Flags  uint16
}

func (self *PowerStatus) MsgID() uint8 {
	return 125
}

func (self *PowerStatus) MsgName() string {
	return "PowerStatus"
}

func (self *PowerStatus) MsgSize() uint8 {
	return 0
}

// Control a serial port. This can be used for raw access to an onboard serial peripheral such as a GPS or telemetry radio. It is designed to make it possible to update the devices firmware via MAVLink messages or change the devices settings. A message with zero bytes can be used to change just the baudrate.
type SerialControl struct {
	Device   uint8
	Flags    uint8
	Timeout  uint16
	Baudrate uint32
	Count    uint8
	Data     [70]uint8
}

func (self *SerialControl) MsgID() uint8 {
	return 126
}

func (self *SerialControl) MsgName() string {
	return "SerialControl"
}

func (self *SerialControl) MsgSize() uint8 {
	return 0
}

// RTK GPS data. Gives information on the relative baseline calculation the GPS is reporting
type GpsRtk struct {
	TimeLastBaselineMs uint32
	RtkReceiverId      uint8
	Wn                 uint16
	Tow                uint32
	RtkHealth          uint8
	RtkRate            uint8
	Nsats              uint8
	BaselineCoordsType uint8
	BaselineAMm        int32
	BaselineBMm        int32
	BaselineCMm        int32
	Accuracy           uint32
	IarNumHypotheses   int32
}

func (self *GpsRtk) MsgID() uint8 {
	return 127
}

func (self *GpsRtk) MsgName() string {
	return "GpsRtk"
}

func (self *GpsRtk) MsgSize() uint8 {
	return 0
}

// RTK GPS data. Gives information on the relative baseline calculation the GPS is reporting
type Gps2Rtk struct {
	TimeLastBaselineMs uint32
	RtkReceiverId      uint8
	Wn                 uint16
	Tow                uint32
	RtkHealth          uint8
	RtkRate            uint8
	Nsats              uint8
	BaselineCoordsType uint8
	BaselineAMm        int32
	BaselineBMm        int32
	BaselineCMm        int32
	Accuracy           uint32
	IarNumHypotheses   int32
}

func (self *Gps2Rtk) MsgID() uint8 {
	return 128
}

func (self *Gps2Rtk) MsgName() string {
	return "Gps2Rtk"
}

func (self *Gps2Rtk) MsgSize() uint8 {
	return 0
}

//
type DataTransmissionHandshake struct {
	Type       uint8
	Size       uint32
	Width      uint16
	Height     uint16
	Packets    uint16
	Payload    uint8
	JpgQuality uint8
}

func (self *DataTransmissionHandshake) MsgID() uint8 {
	return 130
}

func (self *DataTransmissionHandshake) MsgName() string {
	return "DataTransmissionHandshake"
}

func (self *DataTransmissionHandshake) MsgSize() uint8 {
	return 0
}

//
type EncapsulatedData struct {
	Seqnr uint16
	Data  [253]uint8
}

func (self *EncapsulatedData) MsgID() uint8 {
	return 131
}

func (self *EncapsulatedData) MsgName() string {
	return "EncapsulatedData"
}

func (self *EncapsulatedData) MsgSize() uint8 {
	return 0
}

//
type DistanceSensor struct {
	TimeBootMs      uint32
	MinDistance     uint16
	MaxDistance     uint16
	CurrentDistance uint16
	Type            uint8
	Id              uint8
	Orientation     uint8
	Covariance      uint8
}

func (self *DistanceSensor) MsgID() uint8 {
	return 132
}

func (self *DistanceSensor) MsgName() string {
	return "DistanceSensor"
}

func (self *DistanceSensor) MsgSize() uint8 {
	return 0
}

// Request for terrain data and terrain status
type TerrainRequest struct {
	Lat         int32
	Lon         int32
	GridSpacing uint16
	Mask        uint64
}

func (self *TerrainRequest) MsgID() uint8 {
	return 133
}

func (self *TerrainRequest) MsgName() string {
	return "TerrainRequest"
}

func (self *TerrainRequest) MsgSize() uint8 {
	return 0
}

// Terrain data sent from GCS. The lat/lon and grid_spacing must be the same as a lat/lon from a TERRAIN_REQUEST
type TerrainData struct {
	Lat         int32
	Lon         int32
	GridSpacing uint16
	Gridbit     uint8
	Data        [16]int16
}

func (self *TerrainData) MsgID() uint8 {
	return 134
}

func (self *TerrainData) MsgName() string {
	return "TerrainData"
}

func (self *TerrainData) MsgSize() uint8 {
	return 0
}

// Request that the vehicle report terrain height at the given location. Used by GCS to check if vehicle has all terrain data needed for a mission.
type TerrainCheck struct {
	Lat int32
	Lon int32
}

func (self *TerrainCheck) MsgID() uint8 {
	return 135
}

func (self *TerrainCheck) MsgName() string {
	return "TerrainCheck"
}

func (self *TerrainCheck) MsgSize() uint8 {
	return 0
}

// Response from a TERRAIN_CHECK request
type TerrainReport struct {
	Lat           int32
	Lon           int32
	Spacing       uint16
	TerrainHeight float32
	CurrentHeight float32
	Pending       uint16
	Loaded        uint16
}

func (self *TerrainReport) MsgID() uint8 {
	return 136
}

func (self *TerrainReport) MsgName() string {
	return "TerrainReport"
}

func (self *TerrainReport) MsgSize() uint8 {
	return 0
}

// Battery information
type BatteryStatus struct {
	Id               uint8
	BatteryFunction  uint8
	Type             uint8
	Temperature      int16
	Voltages         [10]uint16
	CurrentBattery   int16
	CurrentConsumed  int32
	EnergyConsumed   int32
	BatteryRemaining int8
}

func (self *BatteryStatus) MsgID() uint8 {
	return 147
}

func (self *BatteryStatus) MsgName() string {
	return "BatteryStatus"
}

func (self *BatteryStatus) MsgSize() uint8 {
	return 0
}

// Version and capability of autopilot software
type AutopilotVersion struct {
	Capabilities  uint64
	Version       uint32
	CustomVersion [8]uint8
}

func (self *AutopilotVersion) MsgID() uint8 {
	return 148
}

func (self *AutopilotVersion) MsgName() string {
	return "AutopilotVersion"
}

func (self *AutopilotVersion) MsgSize() uint8 {
	return 0
}

// Message implementing parts of the V2 payload specs in V1 frames for transitional support.
type V2Extension struct {
	TargetNetwork   uint8
	TargetSystem    uint8
	TargetComponent uint8
	MessageType     uint16
	Payload         [249]uint8
}

func (self *V2Extension) MsgID() uint8 {
	return 248
}

func (self *V2Extension) MsgName() string {
	return "V2Extension"
}

func (self *V2Extension) MsgSize() uint8 {
	return 0
}

// Send raw controller memory. The use of this message is discouraged for normal packets, but a quite efficient way for testing new messages and getting experimental debug output.
type MemoryVect struct {
	Address uint16
	Ver     uint8
	Type    uint8
	Value   [32]int8
}

func (self *MemoryVect) MsgID() uint8 {
	return 249
}

func (self *MemoryVect) MsgName() string {
	return "MemoryVect"
}

func (self *MemoryVect) MsgSize() uint8 {
	return 0
}

//
type DebugVect struct {
	Name     Char10
	TimeUsec uint64
	X        float32
	Y        float32
	Z        float32
}

func (self *DebugVect) MsgID() uint8 {
	return 250
}

func (self *DebugVect) MsgName() string {
	return "DebugVect"
}

func (self *DebugVect) MsgSize() uint8 {
	return 0
}

// Send a key-value pair as float. The use of this message is discouraged for normal packets, but a quite efficient way for testing new messages and getting experimental debug output.
type NamedValueFloat struct {
	TimeBootMs uint32
	Name       Char10
	Value      float32
}

func (self *NamedValueFloat) MsgID() uint8 {
	return 251
}

func (self *NamedValueFloat) MsgName() string {
	return "NamedValueFloat"
}

func (self *NamedValueFloat) MsgSize() uint8 {
	return 0
}

// Send a key-value pair as integer. The use of this message is discouraged for normal packets, but a quite efficient way for testing new messages and getting experimental debug output.
type NamedValueInt struct {
	TimeBootMs uint32
	Name       Char10
	Value      int32
}

func (self *NamedValueInt) MsgID() uint8 {
	return 252
}

func (self *NamedValueInt) MsgName() string {
	return "NamedValueInt"
}

func (self *NamedValueInt) MsgSize() uint8 {
	return 0
}

// Status text message. These messages are printed in yellow in the COMM console of QGroundControl. WARNING: They consume quite some bandwidth, so use only for important status and error messages. If implemented wisely, these messages are buffered on the MCU and sent only at a limited rate (e.g. 10 Hz).
type Statustext struct {
	Severity uint8
	Text     Char50
}

func (self *Statustext) MsgID() uint8 {
	return 253
}

func (self *Statustext) MsgName() string {
	return "Statustext"
}

func (self *Statustext) MsgSize() uint8 {
	return 0
}

// Send a debug value. The index is used to discriminate between values. These values show up in the plot of QGroundControl as DEBUG N.
type Debug struct {
	TimeBootMs uint32
	Ind        uint8
	Value      float32
}

func (self *Debug) MsgID() uint8 {
	return 254
}

func (self *Debug) MsgName() string {
	return "Debug"
}

func (self *Debug) MsgSize() uint8 {
	return 0
}

////////////////////////////////////////////////////////////////////////////////
// String Helpers
////////////////////////////////////////////////////////////////////////////////

type Char10 [10]byte

func (chars *Char10) String() string {
	i := 0
	for chars[i] != 0 && i < len(chars) {
		i++
	}
	return string(chars[:i])
}

type Char16 [16]byte

func (chars *Char16) String() string {
	i := 0
	for chars[i] != 0 && i < len(chars) {
		i++
	}
	return string(chars[:i])
}

type Char25 [25]byte

func (chars *Char25) String() string {
	i := 0
	for chars[i] != 0 && i < len(chars) {
		i++
	}
	return string(chars[:i])
}

type Char32 [32]byte

func (chars *Char32) String() string {
	i := 0
	for chars[i] != 0 && i < len(chars) {
		i++
	}
	return string(chars[:i])
}

type Char50 [50]byte

func (chars *Char50) String() string {
	i := 0
	for chars[i] != 0 && i < len(chars) {
		i++
	}
	return string(chars[:i])
}
