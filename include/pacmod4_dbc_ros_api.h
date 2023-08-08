// Copyright (c) 2022 AutonomouStuff, LLC
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef PACMOD4_DBC_ROS_API_H
#define PACMOD4_DBC_ROS_API_H

#include "pacmod4_can_ids.h"

#include <string>
#include <vector>
#include <memory>
#include <mutex>

#ifndef ROS_VERSION
  #define ROS_VERSION 1
#endif

#if ROS_VERSION==1

#include <ros/ros.h>

#include <can_msgs/Frame.h>

// Only include the msgs that are currently supported by pacmod4_common
#include <pacmod4_msgs/AccelAuxRpt.h>
#include <pacmod4_msgs/AirPressureRpt.h>
#include <pacmod4_msgs/AllSystemStatuses.h>
#include <pacmod4_msgs/AngVelRpt.h>
#include <pacmod4_msgs/AutomsManSwitchRpt.h>
#include <pacmod4_msgs/BatteryVoltageLevelRpt.h>
#include <pacmod4_msgs/BrakeAuxRpt.h>
#include <pacmod4_msgs/BrakeDecelAuxRpt.h>
#include <pacmod4_msgs/BrakeDecelCmd.h>
#include <pacmod4_msgs/CabinClimateCmd.h>
#include <pacmod4_msgs/CabinClimateRpt.h>
#include <pacmod4_msgs/ComponentRpt.h>
#include <pacmod4_msgs/DateTimeRpt.h>
#include <pacmod4_msgs/DetectedObjectRpt.h>
#include <pacmod4_msgs/DifferentialLocksCmd.h>
#include <pacmod4_msgs/DifferentialLocksRpt.h>
#include <pacmod4_msgs/DoorRpt.h>
#include <pacmod4_msgs/DrivetrainFeatureRpt.h>
#include <pacmod4_msgs/EStopRpt.h>
#include <pacmod4_msgs/EngineAuxRpt.h>
#include <pacmod4_msgs/EngineAuxRpt2.h>
#include <pacmod4_msgs/EngineBrakeAuxRpt.h>
#include <pacmod4_msgs/EngineBrakeCmd.h>
#include <pacmod4_msgs/EngineBrakeRpt.h>
#include <pacmod4_msgs/EngineLoadFactorRpt.h>
#include <pacmod4_msgs/FireSuppressionRpt.h>
#include <pacmod4_msgs/GlobalCmd.h>
#include <pacmod4_msgs/GlobalRpt.h>
#include <pacmod4_msgs/GlobalRpt2.h>
#include <pacmod4_msgs/GnssTime.h>
#include <pacmod4_msgs/HeadlightAuxRpt.h>
#include <pacmod4_msgs/InteriorLightsRpt.h>
#include <pacmod4_msgs/LatLonHeadingRpt.h>
#include <pacmod4_msgs/LinearAccelRpt.h>
#include <pacmod4_msgs/MotorRpt1.h>
#include <pacmod4_msgs/MotorRpt2.h>
#include <pacmod4_msgs/MotorRpt3.h>
#include <pacmod4_msgs/NotificationCmd.h>
#include <pacmod4_msgs/NotificationRpt.h>
#include <pacmod4_msgs/OccupancyRpt.h>
#include <pacmod4_msgs/ParkingBrakeAuxRpt.h>
#include <pacmod4_msgs/RearLightsRpt.h>
#include <pacmod4_msgs/RemoteStopRpt.h>
#include <pacmod4_msgs/SafetyBrakeCmd.h>
#include <pacmod4_msgs/SafetyBrakeRpt.h>
#include <pacmod4_msgs/SafetyFuncCmd.h>
#include <pacmod4_msgs/SafetyFuncCriticalStopRpt.h>
#include <pacmod4_msgs/SafetyFuncRpt.h>
#include <pacmod4_msgs/SafetyFuncRpt2.h>
#include <pacmod4_msgs/SafetyResponseRpt.h>
#include <pacmod4_msgs/ShiftAuxRpt.h>
#include <pacmod4_msgs/SoftwareVersionRpt.h>
#include <pacmod4_msgs/SteeringAuxRpt.h>
#include <pacmod4_msgs/SteeringCmd.h>
#include <pacmod4_msgs/SteeringCmdLimitRpt.h>
#include <pacmod4_msgs/SupervisoryCtrl.h>
#include <pacmod4_msgs/SystemCmdBool.h>
#include <pacmod4_msgs/SystemCmdFloat.h>
#include <pacmod4_msgs/SystemCmdInt.h>
#include <pacmod4_msgs/SystemCmdLimitRpt.h>
#include <pacmod4_msgs/SystemRptBool.h>
#include <pacmod4_msgs/SystemRptBoolWithControlStatus.h>
#include <pacmod4_msgs/SystemRptFloat.h>
#include <pacmod4_msgs/SystemRptInt.h>
#include <pacmod4_msgs/SystemRptIntWithControlStatus.h>
#include <pacmod4_msgs/TipperBodyAuxRpt.h>
#include <pacmod4_msgs/TirePressureExtendedRpt.h>
#include <pacmod4_msgs/TirePressureRpt.h>
#include <pacmod4_msgs/TireTemperatureExtendedRpt.h>
#include <pacmod4_msgs/TrailerBrakePressureRpt.h>
#include <pacmod4_msgs/TrailerFaultRpt.h>
#include <pacmod4_msgs/TrailerPayloadRpt.h>
#include <pacmod4_msgs/TrailerWheelSpeedRpt.h>
#include <pacmod4_msgs/TurnAuxRpt.h>
#include <pacmod4_msgs/UserPcHealthRpt.h>
#include <pacmod4_msgs/VehicleDynamicsRpt.h>
#include <pacmod4_msgs/VehicleFaultRpt.h>
#include <pacmod4_msgs/VehicleFaultRpt2.h>
#include <pacmod4_msgs/VehicleSpeedRpt.h>
#include <pacmod4_msgs/VinRpt.h>
#include <pacmod4_msgs/WatchdogRpt.h>
#include <pacmod4_msgs/WatchdogRpt2.h>
#include <pacmod4_msgs/WheelSpeedRpt.h>
#include <pacmod4_msgs/WheelSpeedRpt.h>
#include <pacmod4_msgs/WiperAuxRpt.h>
#include <pacmod4_msgs/YawRateRpt.h>

namespace pm_msgs = pacmod4_msgs;
namespace cn_msgs = can_msgs;

#elif ROS_VERSION==2

#include <rclcpp/rclcpp.hpp>

#include <can_msgs/msg/frame.hpp>

// Only include the msgs that are currently supported by pacmod4_common
#include <pacmod4_msgs/msg/accel_aux_rpt.hpp>
#include <pacmod4_msgs/msg/air_pressure_rpt.hpp>
#include <pacmod4_msgs/msg/all_system_statuses.hpp>
#include <pacmod4_msgs/msg/ang_vel_rpt.hpp>
#include <pacmod4_msgs/msg/automs_man_switch_rpt.hpp>
#include <pacmod4_msgs/msg/battery_voltage_level_rpt.hpp>
#include <pacmod4_msgs/msg/brake_aux_rpt.hpp>
#include <pacmod4_msgs/msg/brake_decel_aux_rpt.hpp>
#include <pacmod4_msgs/msg/brake_decel_cmd.hpp>
#include <pacmod4_msgs/msg/cabin_climate_cmd.hpp>
#include <pacmod4_msgs/msg/cabin_climate_rpt.hpp>
#include <pacmod4_msgs/msg/component_rpt.hpp>
#include <pacmod4_msgs/msg/date_time_rpt.hpp>
#include <pacmod4_msgs/msg/detected_object_rpt.hpp>
#include <pacmod4_msgs/msg/differential_locks_cmd.hpp>
#include <pacmod4_msgs/msg/differential_locks_rpt.hpp>
#include <pacmod4_msgs/msg/door_rpt.hpp>
#include <pacmod4_msgs/msg/drivetrain_feature_rpt.hpp>
#include <pacmod4_msgs/msg/estop_rpt.hpp>
#include <pacmod4_msgs/msg/engine_aux_rpt.hpp>
#include <pacmod4_msgs/msg/engine_aux_rpt2.hpp>
#include <pacmod4_msgs/msg/engine_brake_aux_rpt.hpp>
#include <pacmod4_msgs/msg/engine_brake_cmd.hpp>
#include <pacmod4_msgs/msg/engine_brake_rpt.hpp>
#include <pacmod4_msgs/msg/engine_load_factor_rpt.hpp>
#include <pacmod4_msgs/msg/fire_suppression_rpt.hpp>
#include <pacmod4_msgs/msg/global_cmd.hpp>
#include <pacmod4_msgs/msg/global_rpt.hpp>
#include <pacmod4_msgs/msg/global_rpt2.hpp>
#include <pacmod4_msgs/msg/gnss_time.hpp>
#include <pacmod4_msgs/msg/headlight_aux_rpt.hpp>
#include <pacmod4_msgs/msg/interior_lights_rpt.hpp>
#include <pacmod4_msgs/msg/lat_lon_heading_rpt.hpp>
#include <pacmod4_msgs/msg/linear_accel_rpt.hpp>
#include <pacmod4_msgs/msg/motor_rpt1.hpp>
#include <pacmod4_msgs/msg/motor_rpt2.hpp>
#include <pacmod4_msgs/msg/motor_rpt3.hpp>
#include <pacmod4_msgs/msg/notification_cmd.hpp>
#include <pacmod4_msgs/msg/notification_rpt.hpp>
#include <pacmod4_msgs/msg/occupancy_rpt.hpp>
#include <pacmod4_msgs/msg/parking_brake_aux_rpt.hpp>
#include <pacmod4_msgs/msg/rear_lights_rpt.hpp>
#include <pacmod4_msgs/msg/remote_stop_rpt.hpp>
#include <pacmod4_msgs/msg/safety_brake_cmd.hpp>
#include <pacmod4_msgs/msg/safety_brake_rpt.hpp>
#include <pacmod4_msgs/msg/safety_func_cmd.hpp>
#include <pacmod4_msgs/msg/safety_func_critical_stop_rpt.hpp>
#include <pacmod4_msgs/msg/safety_func_rpt.hpp>
#include <pacmod4_msgs/msg/safety_func_rpt2.hpp>
#include <pacmod4_msgs/msg/safety_response_rpt.hpp>
#include <pacmod4_msgs/msg/shift_aux_rpt.hpp>
#include <pacmod4_msgs/msg/software_version_rpt.hpp>
#include <pacmod4_msgs/msg/steering_aux_rpt.hpp>
#include <pacmod4_msgs/msg/steering_cmd.hpp>
#include <pacmod4_msgs/msg/steering_cmd_limit_rpt.hpp>
#include <pacmod4_msgs/msg/supervisory_ctrl.hpp>
#include <pacmod4_msgs/msg/system_cmd_bool.hpp>
#include <pacmod4_msgs/msg/system_cmd_float.hpp>
#include <pacmod4_msgs/msg/system_cmd_int.hpp>
#include <pacmod4_msgs/msg/system_cmd_limit_rpt.hpp>
#include <pacmod4_msgs/msg/system_rpt_bool.hpp>
#include <pacmod4_msgs/msg/system_rpt_bool_with_control_status.hpp>
#include <pacmod4_msgs/msg/system_rpt_float.hpp>
#include <pacmod4_msgs/msg/system_rpt_int.hpp>
#include <pacmod4_msgs/msg/system_rpt_int_with_control_status.hpp>
#include <pacmod4_msgs/msg/tipper_body_aux_rpt.hpp>
#include <pacmod4_msgs/msg/tire_pressure_extended_rpt.hpp>
#include <pacmod4_msgs/msg/tire_pressure_rpt.hpp>
#include <pacmod4_msgs/msg/tire_temperature_extended_rpt.hpp>
#include <pacmod4_msgs/msg/trailer_brake_pressure_rpt.hpp>
#include <pacmod4_msgs/msg/trailer_fault_rpt.hpp>
#include <pacmod4_msgs/msg/trailer_payload_rpt.hpp>
#include <pacmod4_msgs/msg/trailer_wheel_speed_rpt.hpp>
#include <pacmod4_msgs/msg/turn_aux_rpt.hpp>
#include <pacmod4_msgs/msg/user_pc_health_rpt.hpp>
#include <pacmod4_msgs/msg/vehicle_dynamics_rpt.hpp>
#include <pacmod4_msgs/msg/vehicle_fault_rpt.hpp>
#include <pacmod4_msgs/msg/vehicle_fault_rpt2.hpp>
#include <pacmod4_msgs/msg/vehicle_speed_rpt.hpp>
#include <pacmod4_msgs/msg/vin_rpt.hpp>
#include <pacmod4_msgs/msg/watchdog_rpt.hpp>
#include <pacmod4_msgs/msg/watchdog_rpt2.hpp>
#include <pacmod4_msgs/msg/wheel_speed_rpt.hpp>
#include <pacmod4_msgs/msg/wheel_speed_rpt.hpp>
#include <pacmod4_msgs/msg/wiper_aux_rpt.hpp>
#include <pacmod4_msgs/msg/yaw_rate_rpt.hpp>


namespace pm_msgs = pacmod4_msgs::msg;
namespace cn_msgs = can_msgs::msg;

#endif

namespace pacmod4_common
{

class DbcApi
{
public:
  explicit DbcApi(uint32_t version = 0):dbc_major_version_(version){};
  virtual ~DbcApi() = default;

  // Parsing functions take in a ROS CAN msg and return a pointer to a ROS pacmod msg
  // These virtual functions represent all the currently-supported pacmod reports across all DBC versions
  virtual std::shared_ptr<void> ParseAccelAuxRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseAirPressureRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseAngVelRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseAutomsManSwitchRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseBatteryVoltageLevelRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseBrakeAuxRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseBrakeDecelAuxRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseCabinClimateRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseCmdLimitRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseComponentRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseDateTimeRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseDetectedObjectRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseDifferentialLocksRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseDoorRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseDrivetrainFeatureRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseEStopRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseEngineAuxRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseEngineAuxRpt2(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseEngineBrakeAuxRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseEngineBrakeRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseEngineLoadFactorRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseFireSuppressionRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseGlobalRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseGlobalRpt2(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseGnssTime(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseHeadlightAuxRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseInteriorLightsRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseLatLonHeadingRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseLinearAccelRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseMotorRpt1(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseMotorRpt2(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseMotorRpt3(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseNotificationRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseOccupancyRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseParkingBrakeAuxRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseRearLightsRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseRemoteStopRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseSafetyFuncCriticalStopRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseSafetyFuncRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseSafetyFuncRpt2(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseSafetyResponseRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseShiftAuxRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseSoftwareVersionRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseSteeringAuxRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseSteeringCmdLimitRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseSteeringRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseSupervisoryCtrl(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseSystemRptBool(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseSystemRptBoolWithControlStatus(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseSystemRptFloat(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseSystemRptInt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseSystemRptIntWithControlStatus(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseTipperBodyAuxRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseTirePressureExtendedRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseTirePressureRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseTireTemperatureExtendedRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseTrailerBrakePressureRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseTrailerFaultRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseTrailerPayloadRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseTurnAuxRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseVehicleDynamicsRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseVehicleFaultRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseVehicleFaultRpt2(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseVehicleSpeedRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseVinRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseWheelSpeedRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseWiperAuxRpt(const cn_msgs::Frame& can_msg) = 0;
  virtual std::shared_ptr<void> ParseYawRateRpt(const cn_msgs::Frame& can_msg) = 0;

  // Encoding functions take in a ROS pacmod msg and return and ROS CAN frame msg.
  // These virtual functions represent all the currently-supported pacmod commands across all DBC versions
  virtual cn_msgs::Frame EncodeCmd(const pm_msgs::BrakeDecelCmd& msg) = 0;
  virtual cn_msgs::Frame EncodeCmd(const pm_msgs::CabinClimateCmd& msg) = 0;
  virtual cn_msgs::Frame EncodeCmd(const pm_msgs::DifferentialLocksCmd& msg) = 0;
  virtual cn_msgs::Frame EncodeCmd(const pm_msgs::EngineBrakeCmd& msg) = 0;
  virtual cn_msgs::Frame EncodeCmd(const pm_msgs::GlobalCmd& msg) = 0;
  virtual cn_msgs::Frame EncodeCmd(const pm_msgs::NotificationCmd& msg) = 0;
  virtual cn_msgs::Frame EncodeCmd(const pm_msgs::SafetyBrakeCmd& msg) = 0;
  virtual cn_msgs::Frame EncodeCmd(const pm_msgs::SteeringCmd& msg) = 0;
  virtual cn_msgs::Frame EncodeCmd(const pm_msgs::SystemCmdBool& msg) = 0;
  virtual cn_msgs::Frame EncodeCmd(const pm_msgs::SystemCmdFloat& msg) = 0;
  virtual cn_msgs::Frame EncodeCmd(const pm_msgs::SystemCmdInt& msg) = 0;
  virtual cn_msgs::Frame EncodeCmd(const pm_msgs::SafetyFuncCmd& msg) = 0;
  virtual cn_msgs::Frame EncodeCmd(const pm_msgs::UserPcHealthRpt& msg) = 0;

  uint32_t GetDbcVersion();
  void PrintParseError(const std::string& msg_type);
  void PrintEncodeError(const std::string& msg_type);

private:
  uint32_t dbc_major_version_ = 0;
};
}  // namespace pacmod4_common

#endif  // PACMOD4_DBC_ROS_API_H
