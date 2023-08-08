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

#include "pacmod4_dbc13_ros_api.h"
#include "autogen/pacmod13.h"

#include <vector>
#include <string>
#include <memory>


namespace pacmod4_common
{

std::shared_ptr<void> Dbc13Api::ParseAccelAuxRpt(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::AccelAuxRpt>();

  ACCEL_AUX_RPT_t parsed_rpt;
  Unpack_ACCEL_AUX_RPT_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->operator_interaction = parsed_rpt.OPERATOR_INTERACTION;
  new_msg->accel_limiting_active = parsed_rpt.ACCEL_LIMITING_ACTIVE;
  new_msg->park_brake_interlock_active = parsed_rpt.PRK_BRK_INTERLOCK_ACTIVE;
  new_msg->brake_interlock_active = parsed_rpt.BRAKE_INTERLOCK_ACTIVE;

  new_msg->operator_interaction_avail = parsed_rpt.OPERATOR_INTERACTION_AVAIL;
  new_msg->accel_limiting_active_avail = parsed_rpt.ACCEL_LIMITING_ACTIVE_AVAIL;
  new_msg->park_brake_interlock_active_avail = parsed_rpt.PRK_BRK_INTERLOCK_ACTIVE_AVAIL;
  new_msg->brake_interlock_active_avail = parsed_rpt.BRAKE_INTERLOCK_ACTIVE_AVAIL;

  return new_msg;
}

std::shared_ptr<void> Dbc13Api::ParseAirPressureRpt(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::AirPressureRpt>();

  AIR_PRESSURE_RPT_t parsed_rpt;
  Unpack_AIR_PRESSURE_RPT_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->pneumatic_supply_pressure = parsed_rpt.PNEUMATIC_SUPPLY_PRESSURE_phys;
  new_msg->brake_circuit_1_pressure = parsed_rpt.BRAKE_CIRCUIT_1_PRESSURE_phys;
  new_msg->brake_circuit_2_pressure = parsed_rpt.BRAKE_CIRCUIT_2_PRESSURE_phys;
  new_msg->park_trailer_air_pressure = parsed_rpt.PARK_TRAILER_AIR_PRESSURE_phys;
  new_msg->powertrain_air_pressure = parsed_rpt.POWERTRAIN_AIR_PRESSURE_phys;
  new_msg->air_compressor_status = parsed_rpt.AIR_COMPRESSOR_STATUS;
  new_msg->pneumatic_supply_pressure_avail = parsed_rpt.PNEUMATIC_SUPPLY_PRESSURE_AVAIL;
  new_msg->brake_circuit_1_pressure_avail = parsed_rpt.BRAKE_CIRCUIT_1_PRESSURE_AVAIL;
  new_msg->brake_circuit_2_pressure_avail = parsed_rpt.BRAKE_CIRCUIT_2_PRESSURE_AVAIL;
  new_msg->park_trailer_air_pressure_avail = parsed_rpt.PARK_TRAILER_AIR_PRESSURE_AVAIL;
  new_msg->powertrain_air_pressure_avail = parsed_rpt.POWERTRAIN_AIR_PRESSURE_AVAIL;
  new_msg->air_compressor_status_avail = parsed_rpt.AIR_COMPRESSOR_STATUS_AVAIL;

  return new_msg;
}

std::shared_ptr<void> Dbc13Api::ParseAngVelRpt(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::AngVelRpt>();

  ANG_VEL_RPT_t parsed_rpt;
  Unpack_ANG_VEL_RPT_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->pitch_new_data_rx = parsed_rpt.PITCH_NEW_DATA_RX;
  new_msg->roll_new_data_rx = parsed_rpt.ROLL_NEW_DATA_RX;
  new_msg->yaw_new_data_rx = parsed_rpt.YAW_NEW_DATA_RX;

  new_msg->pitch_valid = parsed_rpt.PITCH_VALID;
  new_msg->roll_valid = parsed_rpt.ROLL_VALID;
  new_msg->yaw_valid = parsed_rpt.YAW_VALID;

  new_msg->pitch_vel = parsed_rpt.PITCH_VEL_phys;
  new_msg->roll_vel = parsed_rpt.ROLL_VEL_phys;
  new_msg->yaw_vel = parsed_rpt.YAW_VEL_phys;

  return new_msg;
}

std::shared_ptr<void> Dbc13Api::ParseAutomsManSwitchRpt(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::AutomsManSwitchRpt>();

  AUTOMS_MAN_SWITCH_RPT_t parsed_rpt;
  Unpack_AUTOMS_MAN_SWITCH_RPT_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->automs_man_opctrl = parsed_rpt.AUTOMS_MAN_OPCTRL;
  new_msg->automs_man_extctrl = parsed_rpt.AUTOMS_MAN_EXTCTRL;
  new_msg->automs_man_opctrl_avail = parsed_rpt.AUTOMS_MAN_OPCTRL_AVAIL;
  new_msg->automs_man_extctrl_avail = parsed_rpt.AUTOMS_MAN_EXTCTRL_AVAIL;

  return new_msg;
}

std::shared_ptr<void> Dbc13Api::ParseBatteryVoltageLevelRpt(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::BatteryVoltageLevelRpt>();

  BATTERY_VOLTAGE_LEVEL_RPT_1_t parsed_rpt;
  Unpack_BATTERY_VOLTAGE_LEVEL_RPT_1_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->battery_voltage_1 = parsed_rpt.BATTERY_VOLTAGE_1_phys;
  new_msg->battery_voltage_2 = parsed_rpt.BATTERY_VOLTAGE_2_phys;

  return new_msg;
}

std::shared_ptr<void> Dbc13Api::ParseBrakeAuxRpt(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::BrakeAuxRpt>();

  BRAKE_AUX_RPT_t parsed_rpt;
  Unpack_BRAKE_AUX_RPT_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->brake_pressure_2 = parsed_rpt.BRAKE_PRESSURE_2_phys;
  new_msg->brake_pressure = parsed_rpt.BRAKE_PRESSURE_phys;
  new_msg->operator_interaction = parsed_rpt.OPERATOR_INTERACTION;
  new_msg->brake_on_off = parsed_rpt.BRAKE_ON_OFF;
  new_msg->brake_limiting_active = parsed_rpt.BRAKE_LIMITING_ACTIVE;
  new_msg->brake_reduced_assist = parsed_rpt.BRAKE_REDUCED_ASSIST;
  new_msg->calibration_status = parsed_rpt.CALIBRATION_STATUS;
  new_msg->brake_pressure_2_avail = parsed_rpt.BRAKE_PRESSURE_2_AVAIL;
  new_msg->brake_pressure_avail = parsed_rpt.BRAKE_PRESSURE_AVAIL;
  new_msg->brake_on_off_avail = parsed_rpt.BRAKE_ON_OFF_AVAIL;
  new_msg->brake_limiting_active_avail = parsed_rpt.BRAKE_LIMITING_ACTIVE_AVAIL;
  new_msg->brake_reduced_assist_avail = parsed_rpt.BRAKE_REDUCED_ASSIST_AVAIL;

  return new_msg;
}

std::shared_ptr<void> Dbc13Api::ParseBrakeDecelAuxRpt(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::BrakeDecelAuxRpt>();

  BRAKE_DECEL_AUX_RPT_t parsed_rpt;
  Unpack_BRAKE_DECEL_AUX_RPT_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->xbr_active_control_mode = parsed_rpt.XBR_ACTIVE_CONTROL_MODE;
  new_msg->xbr_system_state = parsed_rpt.XBR_SYSTEM_STATE;
  new_msg->foundation_brake_use = parsed_rpt.FOUNDATION_BRAKE_USE;
  new_msg->hill_holder_mode = parsed_rpt.HILL_HOLDER_MODE;
  new_msg->xbr_active_control_mode_avail = parsed_rpt.XBR_ACTIVE_CONTROL_MODE_AVAIL;
  new_msg->xbr_system_state_avail = parsed_rpt.XBR_SYSTEM_STATE_AVAIL;
  new_msg->foundation_brake_use_avail = parsed_rpt.FOUNDATION_BRAKE_USE_AVAIL;
  new_msg->hill_holder_mode_avail = parsed_rpt.HILL_HOLDER_MODE_AVAIL;

  return new_msg;
}

std::shared_ptr<void> Dbc13Api::ParseCabinClimateRpt(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::CabinClimateRpt>();

  CABIN_CLIMATE_RPT_t parsed_rpt;
  Unpack_CABIN_CLIMATE_RPT_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->enabled = parsed_rpt.ENABLED;
  new_msg->override_active = parsed_rpt.OVERRIDE_ACTIVE;
  new_msg->command_output_fault = parsed_rpt.COMMAND_OUTPUT_FAULT;
  new_msg->input_output_fault = parsed_rpt.INPUT_OUTPUT_FAULT;
  new_msg->output_reported_fault = parsed_rpt.OUTPUT_REPORTED_FAULT;
  new_msg->pacmod_fault = parsed_rpt.PACMOD_FAULT;
  new_msg->vehicle_fault = parsed_rpt.VEHICLE_FAULT;
  new_msg->command_timeout = parsed_rpt.COMMAND_TIMEOUT;
  new_msg->man_ac_off_on = parsed_rpt.MAN_AC_OFF_ON;
  new_msg->man_max_ac_off_on = parsed_rpt.MAN_MAX_AC_OFF_ON;
  new_msg->man_defrost_off_on = parsed_rpt.MAN_DEFROST_OFF_ON;
  new_msg->man_max_defrost_off_on = parsed_rpt.MAN_MAX_DEFROST_OFF_ON;
  new_msg->man_dir_up_off_on = parsed_rpt.MAN_DIR_UP_OFF_ON;
  new_msg->man_dir_down_off_on = parsed_rpt.MAN_DIR_DOWN_OFF_ON;
  new_msg->cmd_ac_off_on = parsed_rpt.CMD_AC_OFF_ON;
  new_msg->cmd_max_ac_off_on = parsed_rpt.CMD_MAX_AC_OFF_ON;
  new_msg->cmd_defrost_off_on = parsed_rpt.CMD_DEFROST_OFF_ON;
  new_msg->cmd_max_defrost_off_on = parsed_rpt.CMD_MAX_DEFROST_OFF_ON;
  new_msg->cmd_dir_up_off_on = parsed_rpt.CMD_DIR_UP_OFF_ON;
  new_msg->cmd_dir_down_off_on = parsed_rpt.CMD_DIR_DOWN_OFF_ON;
  new_msg->out_ac_off_on = parsed_rpt.OUT_AC_OFF_ON;
  new_msg->out_max_ac_off_on = parsed_rpt.OUT_MAX_AC_OFF_ON;
  new_msg->out_defrost_off_on = parsed_rpt.OUT_DEFROST_OFF_ON;
  new_msg->out_max_defrost_off_on = parsed_rpt.OUT_MAX_DEFROST_OFF_ON;
  new_msg->out_dir_up_off_on = parsed_rpt.OUT_DIR_UP_OFF_ON;
  new_msg->out_dir_down_off_on = parsed_rpt.OUT_DIR_DOWN_OFF_ON;

  return new_msg;
}

std::shared_ptr<void> Dbc13Api::ParseCmdLimitRpt(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::SystemCmdLimitRpt>();

  ACCEL_CMD_LIMIT_RPT_t parsed_rpt;
  Unpack_ACCEL_CMD_LIMIT_RPT_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->sys_cmd_limit = parsed_rpt.ACCEL_CMD_LIMIT_phys;
  new_msg->limited_sys_cmd = parsed_rpt.LIMITED_ACCEL_CMD_phys;

  return new_msg;
}

std::shared_ptr<void> Dbc13Api::ParseComponentRpt(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::ComponentRpt>();

  COMPONENT_RPT_00_t parsed_rpt;
  Unpack_COMPONENT_RPT_00_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->component_type = parsed_rpt.COMPONENT_TYPE;
  new_msg->accel = parsed_rpt.ACCEL;
  new_msg->brake = parsed_rpt.BRAKE;
  new_msg->cruise_control_buttons = parsed_rpt.CRUISE_CONTROL_BUTTONS;
  new_msg->dash_controls_left = parsed_rpt.DASH_CONTROLS_LEFT;
  new_msg->dash_controls_right = parsed_rpt.DASH_CONTROLS_RIGHT;
  new_msg->hazard_lights = parsed_rpt.HAZARD_LIGHTS;
  new_msg->headlight = parsed_rpt.HEADLIGHT;
  new_msg->horn = parsed_rpt.HORN;
  new_msg->media_controls = parsed_rpt.MEDIA_CONTROLS;
  new_msg->parking_brake = parsed_rpt.PARKING_BRAKE;
  new_msg->shift = parsed_rpt.SHIFT;
  new_msg->sprayer = parsed_rpt.SPRAYER;
  new_msg->steering = parsed_rpt.STEERING;
  new_msg->turn = parsed_rpt.TURN;
  new_msg->wiper = parsed_rpt.WIPER;
  new_msg->watchdog = parsed_rpt.WATCHDOG;
  new_msg->brake_decel = parsed_rpt.BRAKE_DECEL;
  new_msg->rear_pass_door = parsed_rpt.REAR_PASS_DOOR;
  new_msg->engine_brake = parsed_rpt.ENGINE_BRAKE;
  new_msg->marker_lamp = parsed_rpt.MARKER_LAMP;
  new_msg->cabin_climate = parsed_rpt.CABIN_CLIMATE;
  new_msg->cabin_fan_speed = parsed_rpt.CABIN_FAN_SPEED;
  new_msg->cabin_temp = parsed_rpt.CABIN_TEMP;
  new_msg->exhaust_brake = parsed_rpt.EXHAUST_BRAKE;
  new_msg->power_take_off = parsed_rpt.POWER_TAKE_OFF;
  new_msg->tipper_body_00 = parsed_rpt.TIPPER_BODY_00;
  new_msg->trailer_air_supply = parsed_rpt.TRAILER_AIR_SUPPLY;
  new_msg->trailer_brake = parsed_rpt.TRAILER_BRAKE;

  new_msg->counter = parsed_rpt.COUNTER;
  new_msg->complement = parsed_rpt.COMPLEMENT;
  new_msg->config_fault = parsed_rpt.CONFIG_FAULT;
  new_msg->can_timeout_fault = parsed_rpt.CAN_TIMEOUT_FAULT;
  new_msg->internal_supply_voltage_fault = parsed_rpt.INTERNAL_SUPPLY_VOLTAGE_FAULT;
  new_msg->supervisory_timeout = parsed_rpt.SUPERVISORY_TIMEOUT;
  new_msg->supervisory_sanity_fault = parsed_rpt.SUPERVISORY_SANITY_FAULT;
  new_msg->watchdog_sanity_fault = parsed_rpt.WATCHDOG_SANITY_FAULT;
  new_msg->watchdog_system_present_fault = parsed_rpt.WATCHDOG_SYSTEM_PRESENT_FAULT;
  new_msg->component_ready = parsed_rpt.COMPONENT_READY;
  new_msg->engine = parsed_rpt.ENGINE;
  new_msg->tipper_body_01 = parsed_rpt.TIPPER_BODY_01;
  new_msg->tipper_body_02 = parsed_rpt.TIPPER_BODY_02;
  new_msg->system_enabled = parsed_rpt.SYSTEM_ENABLED;
  new_msg->system_override_active = parsed_rpt.SYSTEM_OVERRIDE_ACTIVE;
  new_msg->system_fault_active = parsed_rpt.SYSTEM_FAULT_ACTIVE;
  new_msg->user_pc_health_fault_00 = parsed_rpt.USER_PC_HEALTH_FAULT_00;
  new_msg->user_pc_health_fault_01 = parsed_rpt.USER_PC_HEALTH_FAULT_01;
  new_msg->differential_locks = parsed_rpt.DIFFERENTIAL_LOCKS;

  return new_msg;
}

std::shared_ptr<void> Dbc13Api::ParseDateTimeRpt(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::DateTimeRpt>();

  DATE_TIME_RPT_t parsed_rpt;
  Unpack_DATE_TIME_RPT_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->year = parsed_rpt.DATE_YEAR_phys;
  new_msg->month = parsed_rpt.DATE_MONTH_phys;
  new_msg->day = parsed_rpt.DATE_DAY_phys;

  new_msg->hour = parsed_rpt.TIME_HOUR;
  new_msg->minute = parsed_rpt.TIME_MINUTE;
  new_msg->second = parsed_rpt.TIME_SECOND;

  return new_msg;
}

std::shared_ptr<void> Dbc13Api::ParseDifferentialLocksRpt(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::DifferentialLocksRpt>();

  DIFFERENTIAL_LOCKS_RPT_t parsed_rpt;
  Unpack_DIFFERENTIAL_LOCKS_RPT_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->enabled = parsed_rpt.ENABLED;
  new_msg->override_active = parsed_rpt.OVERRIDE_ACTIVE;
  new_msg->command_output_fault = parsed_rpt.COMMAND_OUTPUT_FAULT;
  new_msg->input_output_fault = parsed_rpt.INPUT_OUTPUT_FAULT;
  new_msg->output_reported_fault = parsed_rpt.OUTPUT_REPORTED_FAULT;
  new_msg->pacmod_fault = parsed_rpt.PACMOD_FAULT;
  new_msg->vehicle_fault = parsed_rpt.VEHICLE_FAULT;
  new_msg->command_timeout = parsed_rpt.COMMAND_TIMEOUT;
  new_msg->man_front_lock = parsed_rpt.MAN_FRONT_LOCK;
  new_msg->cmd_front_lock = parsed_rpt.CMD_FRONT_LOCK;
  new_msg->out_front_lock = parsed_rpt.OUT_FRONT_LOCK;
  new_msg->man_center_lock = parsed_rpt.MAN_CENTER_LOCK;
  new_msg->cmd_center_lock = parsed_rpt.CMD_CENTER_LOCK;
  new_msg->out_center_lock = parsed_rpt.OUT_CENTER_LOCK;
  new_msg->man_rear_lock = parsed_rpt.MAN_REAR_LOCK;
  new_msg->cmd_rear_lock = parsed_rpt.CMD_REAR_LOCK;
  new_msg->out_rear_lock = parsed_rpt.OUT_REAR_LOCK;

  return new_msg;
}

std::shared_ptr<void> Dbc13Api::ParseDetectedObjectRpt(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::DetectedObjectRpt>();

  DETECTED_OBJECT_RPT_t parsed_rpt;
  Unpack_DETECTED_OBJECT_RPT_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->front_object_distance_low_res = parsed_rpt.FRONT_OBJECT_DISTANCE_LOW_RES_phys;
  new_msg->front_object_distance_high_res = parsed_rpt.FRONT_OBJECT_DISTANCE_HIGH_RES_phys;

  return new_msg;
}

std::shared_ptr<void> Dbc13Api::ParseDoorRpt(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::DoorRpt>();

  DOOR_RPT_t parsed_rpt;
  Unpack_DOOR_RPT_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->driver_door_open = parsed_rpt.DRIVER_DOOR_OPEN;
  new_msg->passenger_door_open = parsed_rpt.PASS_DOOR_OPEN;
  new_msg->rear_driver_door_open = parsed_rpt.REAR_DRIVER_DOOR_OPEN;
  new_msg->rear_passenger_door_open = parsed_rpt.REAR_PASS_DOOR_OPEN;
  new_msg->hood_open = parsed_rpt.HOOD_OPEN;
  new_msg->trunk_open = parsed_rpt.TRUNK_OPEN;
  new_msg->fuel_door_open = parsed_rpt.FUEL_DOOR_OPEN;
  new_msg->driver_door_open_avail = parsed_rpt.DRIVER_DOOR_OPEN_AVAIL;
  new_msg->passenger_door_open_avail = parsed_rpt.PASS_DOOR_OPEN_AVAIL;
  new_msg->rear_driver_door_open_avail = parsed_rpt.REAR_DRIVER_DOOR_OPEN_AVAIL;
  new_msg->rear_passenger_door_open_avail = parsed_rpt.REAR_PASS_DOOR_OPEN_AVAIL;
  new_msg->hood_open_avail = parsed_rpt.HOOD_OPEN_AVAIL;
  new_msg->trunk_open_avail = parsed_rpt.TRUNK_OPEN_AVAIL;
  new_msg->fuel_door_open_avail = parsed_rpt.FUEL_DOOR_OPEN_AVAIL;

  return new_msg;
}

std::shared_ptr<void> Dbc13Api::ParseDrivetrainFeatureRpt(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::DrivetrainFeatureRpt>();

  DRIVE_TRAIN_FEATURE_RPT_t parsed_rpt;
  Unpack_DRIVE_TRAIN_FEATURE_RPT_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->antilock_brake_active = parsed_rpt.ANTILOCK_BRAKE_ACTIVE;
  new_msg->traction_control_active = parsed_rpt.TRACTION_CONTROL_ACTIVE;
  new_msg->four_wheel_drive_active = parsed_rpt.FOUR_WHEEL_DRIVE_ACTIVE;
  new_msg->antilock_brake_disabled = parsed_rpt.ANTILOCK_BRAKE_DISABLED;
  new_msg->antilock_brake_active_avail = parsed_rpt.ANTILOCK_BRAKE_ACTIVE_AVAIL;
  new_msg->traction_control_active_avail = parsed_rpt.TRACTION_CONTROL_ACTIVE_AVAIL;
  new_msg->four_wheel_drive_active_avail = parsed_rpt.FOUR_WHEEL_DRIVE_ACTIVE_AVAIL;
  new_msg->antilock_brake_disabled_avail = parsed_rpt.ANTILOCK_BRAKE_DISABLED_AVAIL;
  new_msg->drive_mode = parsed_rpt.DRIVE_MODE;
  new_msg->drive_mode_avail = parsed_rpt.DRIVE_MODE_AVAIL;
  new_msg->traction_control_disabled = parsed_rpt.TRACTION_CONTROL_DISABLED;
  new_msg->traction_control_disabled_avail = parsed_rpt.TRACTION_CONTROL_DISABLED_AVAIL;

  return new_msg;
}

std::shared_ptr<void> Dbc13Api::ParseEStopRpt(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::EStopRpt>();

  ESTOP_RPT_t parsed_rpt;
  Unpack_ESTOP_RPT_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->estop_status = parsed_rpt.ESTOP;
  new_msg->estop_fault = parsed_rpt.ESTOP_FAULT;

  return new_msg;
}

std::shared_ptr<void> Dbc13Api::ParseEngineAuxRpt(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::EngineAuxRpt>();

  ENGINE_AUX_RPT_t parsed_rpt;
  Unpack_ENGINE_AUX_RPT_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->engine_speed = parsed_rpt.ENGINE_SPEED_phys;
  new_msg->engine_torque = parsed_rpt.ENGINE_TORQUE_phys;
  new_msg->engine_coolant_temp = parsed_rpt.ENGINE_COOLANT_TEMP_phys;
  new_msg->engine_speed_avail = parsed_rpt.ENGINE_SPEED_AVAIL;
  new_msg->engine_torque_avail = parsed_rpt.ENGINE_TORQUE_AVAIL;
  new_msg->engine_coolant_temp_avail = parsed_rpt.ENGINE_COOLANT_TEMP_AVAIL;
  new_msg->fuel_level_avail = parsed_rpt.FUEL_LEVEL_AVAIL;
  new_msg->diesel_exhaust_fluid_level_avail = parsed_rpt.DIESEL_EXHAUST_FLUID_LEVEL_AVAIL;
  new_msg->fuel_level = parsed_rpt.FUEL_LEVEL_phys;
  new_msg->diesel_exhaust_fluid_level = parsed_rpt.DIESEL_EXHAUST_FLUID_LEVEL_phys;

  return new_msg;
}

std::shared_ptr<void> Dbc13Api::ParseEngineAuxRpt2(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::EngineAuxRpt2>();

  ENGINE_AUX_RPT_2_t parsed_rpt;
  Unpack_ENGINE_AUX_RPT_2_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->fuel_rate = parsed_rpt.FUEL_RATE_phys;
  new_msg->fuel_rate_avail = parsed_rpt.FUEL_RATE_AVAIL;
  new_msg->oil_level_avail = parsed_rpt.OIL_LEVEL_AVAIL;
  new_msg->oil_pressure_avail = parsed_rpt.OIL_PRESSURE_AVAIL;
  new_msg->oil_level = parsed_rpt.OIL_LEVEL_phys;
  new_msg->oil_pressure = parsed_rpt.OIL_PRESSURE_phys;

  return new_msg;
}

std::shared_ptr<void> Dbc13Api::ParseEngineLoadFactorRpt(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::EngineLoadFactorRpt>();

  ENGINE_LOAD_FACTOR_RPT_t parsed_rpt;
  Unpack_ENGINE_LOAD_FACTOR_RPT_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->drvr_demanded_eng_torque = parsed_rpt.DRVR_DEMANDED_ENG_TORQUE_phys;
  new_msg->actual_engine_torque = parsed_rpt.ACTUAL_ENGINE_TORQUE_phys;
  new_msg->eng_load_at_current_speed = parsed_rpt.ENG_LOAD_AT_CURRENT_SPEED;
  new_msg->reference_engine_torque = parsed_rpt.REFERENCE_ENGINE_TORQUE;
  new_msg->drvr_demanded_eng_torque_avail = parsed_rpt.DRVR_DEMANDED_ENG_TORQUE_AVAIL;
  new_msg->actual_engine_torque_avail = parsed_rpt.ACTUAL_ENGINE_TORQUE_AVAIL;
  new_msg->eng_load_at_current_speed_avail = parsed_rpt.ENG_LOAD_AT_CURRENT_SPEED_AVAIL;
  new_msg->reference_engine_torque_avail = parsed_rpt.REFERENCE_ENGINE_TORQUE_AVAIL;

  return new_msg;
}

std::shared_ptr<void> Dbc13Api::ParseEngineBrakeAuxRpt(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::EngineBrakeAuxRpt>();

  ENGINE_BRAKE_AUX_RPT_t parsed_rpt;
  Unpack_ENGINE_BRAKE_AUX_RPT_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->engine_brake_status = parsed_rpt.ENGINE_BRAKE_STATUS;
  new_msg->actual_engine_brk_torque = parsed_rpt.ACTUAL_ENGINE_BRK_TORQUE_phys;
  new_msg->intended_engine_brk_torque = parsed_rpt.INTENDED_ENGINE_BRK_TORQUE_phys;
  new_msg->engine_brake_status_avail = parsed_rpt.ENGINE_BRAKE_STATUS_AVAIL;
  new_msg->actual_engine_brk_torque_avail = parsed_rpt.ACTUAL_ENGINE_BRK_TORQUE_AVAIL;
  new_msg->intended_engine_brk_torque_avail = parsed_rpt.INTENDED_ENGINE_BRK_TORQUE_AVAIL;

  return new_msg;
}

std::shared_ptr<void> Dbc13Api::ParseEngineBrakeRpt(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::EngineBrakeRpt>();

  ENGINE_BRAKE_RPT_t parsed_rpt;
  Unpack_ENGINE_BRAKE_RPT_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));
  new_msg->enabled = parsed_rpt.ENABLED;
  new_msg->override_active = parsed_rpt.OVERRIDE_ACTIVE;
  new_msg->command_output_fault = parsed_rpt.COMMAND_OUTPUT_FAULT;
  new_msg->input_output_fault = parsed_rpt.INPUT_OUTPUT_FAULT;
  new_msg->output_reported_fault = parsed_rpt.OUTPUT_REPORTED_FAULT;
  new_msg->pacmod_fault = parsed_rpt.PACMOD_FAULT;
  new_msg->vehicle_fault = parsed_rpt.VEHICLE_FAULT;
  new_msg->command_timeout = parsed_rpt.COMMAND_TIMEOUT;
  new_msg->manual_input = parsed_rpt.MANUAL_INPUT;
  new_msg->commanded_value = parsed_rpt.COMMANDED_VALUE;
  new_msg->output_value = parsed_rpt.OUTPUT_VALUE;
  new_msg->man_auto = parsed_rpt.MAN_AUTO;
  new_msg->cmd_auto = parsed_rpt.CMD_AUTO;
  new_msg->out_auto = parsed_rpt.OUT_AUTO;

  return new_msg;
}

std::shared_ptr<void> Dbc13Api::ParseFireSuppressionRpt(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::FireSuppressionRpt>();

  FIRE_SUPPRESSION_RPT_t parsed_rpt;
  Unpack_FIRE_SUPPRESSION_RPT_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));
  new_msg->fire_suppression_alarm_status = parsed_rpt.FIRE_SUPPRESSION_ALARM_STATUS;

  return new_msg;
}

std::shared_ptr<void> Dbc13Api::ParseGlobalRpt(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::GlobalRpt>();

  GLOBAL_RPT_t parsed_rpt;
  Unpack_GLOBAL_RPT_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->enabled = parsed_rpt.PACMOD_SYSTEM_ENABLED;
  new_msg->override_active = parsed_rpt.PACMOD_SYSTEM_OVERRIDE_ACTIVE;
  new_msg->pacmod_sys_fault_active = parsed_rpt.PACMOD_SYSTEM_FAULT_ACTIVE;
  new_msg->config_fault_active = parsed_rpt.CONFIG_FAULT_ACTIVE;
  new_msg->user_can_timeout = parsed_rpt.USR_CAN_TIMEOUT;
  new_msg->steering_can_timeout = parsed_rpt.STR_CAN_TIMEOUT;
  new_msg->brake_can_timeout = parsed_rpt.BRK_CAN_TIMEOUT;
  new_msg->subsystem_can_timeout = parsed_rpt.PACMOD_SUBSYSTEM_TIMEOUT;
  new_msg->vehicle_can_timeout = parsed_rpt.VEH_CAN_TIMEOUT;
  new_msg->user_can_read_errors = parsed_rpt.USR_CAN_READ_ERRORS;
  new_msg->supervisory_enable_required = parsed_rpt.SUPERVISORY_ENABLE_REQUIRED;

  return new_msg;
}

std::shared_ptr<void> Dbc13Api::ParseGlobalRpt2(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::GlobalRpt2>();

  GLOBAL_RPT_2_t parsed_rpt;
  Unpack_GLOBAL_RPT_2_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->system_enabled = parsed_rpt.SYSTEM_ENABLED;
  new_msg->system_override_active = parsed_rpt.SYSTEM_OVERRIDE_ACTIVE;
  new_msg->system_fault_active = parsed_rpt.SYSTEM_FAULT_ACTIVE;
  new_msg->supervisory_enable_required = parsed_rpt.SUPERVISORY_ENABLE_REQUIRED;
  new_msg->disable_all_systems = parsed_rpt.DISABLE_ALL_SYSTEMS;
  new_msg->system_ready = parsed_rpt.SYSTEM_READY;
  new_msg->enable_method = parsed_rpt.ENABLE_METHOD;
  new_msg->override_mode = parsed_rpt.OVERRIDE_MODE;
  new_msg->development_mode = parsed_rpt.DEVELOPMENT_MODE;
  new_msg->development_mode_allowed = parsed_rpt.DEVELOPMENT_MODE_ALLOWED;

  return new_msg;
}

std::shared_ptr<void> Dbc13Api::ParseGnssTime(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::GnssTime>();

  GNSS_TIME_t parsed_rpt;
  Unpack_GNSS_TIME_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->gnss_timestamp = parsed_rpt.GNSS_TIMESTAMP;

  return new_msg;
}

std::shared_ptr<void> Dbc13Api::ParseHeadlightAuxRpt(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::HeadlightAuxRpt>();

  HEADLIGHT_AUX_RPT_t parsed_rpt;
  Unpack_HEADLIGHT_AUX_RPT_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->headlights_on = parsed_rpt.HEADLIGHTS_ON;
  new_msg->headlights_on_bright = parsed_rpt.HEADLIGHTS_ON_BRIGHT;
  new_msg->fog_lights_on = parsed_rpt.FOG_LIGHTS_ON;
  new_msg->headlights_mode = parsed_rpt.HEADLIGHTS_MODE;
  new_msg->headlights_on_avail = parsed_rpt.HEADLIGHTS_ON_AVAIL;
  new_msg->headlights_on_bright_avail = parsed_rpt.HEADLIGHTS_ON_BRIGHT_AVAIL;
  new_msg->fog_lights_on_avail = parsed_rpt.FOG_LIGHTS_ON_AVAIL;
  new_msg->headlights_mode_avail = parsed_rpt.HEADLIGHTS_MODE_AVAIL;

  return new_msg;
}

std::shared_ptr<void> Dbc13Api::ParseInteriorLightsRpt(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::InteriorLightsRpt>();

  INTERIOR_LIGHTS_RPT_t parsed_rpt;
  Unpack_INTERIOR_LIGHTS_RPT_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->front_dome_lights_on = parsed_rpt.FRONT_DOME_LIGHTS_ON;
  new_msg->front_dome_lights_on_avail = parsed_rpt.FRONT_DOME_LIGHTS_ON_AVAIL;
  new_msg->rear_dome_lights_on = parsed_rpt.REAR_DOME_LIGHTS_ON;
  new_msg->rear_dome_lights_on_avail = parsed_rpt.REAR_DOME_LIGHTS_ON_AVAIL;
  new_msg->mood_lights_on = parsed_rpt.MOOD_LIGHTS_ON;
  new_msg->mood_lights_on_avail = parsed_rpt.MOOD_LIGHTS_ON_AVAIL;
  new_msg->dim_level = parsed_rpt.DIM_LEVEL;
  new_msg->dim_level_avail = parsed_rpt.DIM_LEVEL_AVAIL;
  new_msg->ambient_light_sensor = parsed_rpt.AMBIENT_LIGHT_SENSOR;
  new_msg->ambient_light_sensor_avail = parsed_rpt.AMBIENT_LIGHT_SENSOR_AVAIL;

  return new_msg;
}

std::shared_ptr<void> Dbc13Api::ParseLatLonHeadingRpt(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::LatLonHeadingRpt>();

  LAT_LON_HEADING_RPT_t parsed_rpt;
  Unpack_LAT_LON_HEADING_RPT_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->latitude_degrees = parsed_rpt.LATITUDE_DEGREES;
  new_msg->latitude_minutes = parsed_rpt.LATITUDE_MINUTES;
  new_msg->latitude_seconds = parsed_rpt.LATITUDE_SECONDS;
  new_msg->longitude_degrees = parsed_rpt.LONGITUDE_DEGREES;
  new_msg->longitude_minutes = parsed_rpt.LONGITUDE_MINUTES;
  new_msg->longitude_seconds = parsed_rpt.LONGITUDE_SECONDS;
  new_msg->heading = parsed_rpt.HEADING_phys;

  return new_msg;
}

std::shared_ptr<void> Dbc13Api::ParseLinearAccelRpt(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::LinearAccelRpt>();

  LINEAR_ACCEL_RPT_t parsed_rpt;
  Unpack_LINEAR_ACCEL_RPT_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->lateral_new_data_rx = parsed_rpt.LATERAL_NEW_DATA_RX;
  new_msg->longitudinal_new_data_rx = parsed_rpt.LONGITUDINAL_NEW_DATA_RX;
  new_msg->vertical_new_data_rx = parsed_rpt.VERTICAL_NEW_DATA_RX;

  new_msg->lateral_valid = parsed_rpt.LATERAL_VALID;
  new_msg->longitudinal_valid = parsed_rpt.LONGITUDINAL_VALID;
  new_msg->vertical_valid = parsed_rpt.VERTICAL_VALID;

  new_msg->lateral_accel = parsed_rpt.LATERAL_ACCEL_phys;
  new_msg->longitudinal_accel = parsed_rpt.LONGITUDINAL_ACCEL_phys;
  new_msg->vertical_accel = parsed_rpt.VERTICAL_ACCEL_phys;

  return new_msg;
}

std::shared_ptr<void> Dbc13Api::ParseMotorRpt1(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::MotorRpt1>();

  BRAKE_MOTOR_RPT_1_t parsed_rpt;
  Unpack_BRAKE_MOTOR_RPT_1_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->current = parsed_rpt.MOTOR_CURRENT_phys;
  new_msg->position = parsed_rpt.SHAFT_POSITION_phys;

  return new_msg;
}

std::shared_ptr<void> Dbc13Api::ParseMotorRpt2(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::MotorRpt2>();

  BRAKE_MOTOR_RPT_2_t parsed_rpt;
  Unpack_BRAKE_MOTOR_RPT_2_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->encoder_temp = parsed_rpt.ENCODER_TEMPERATURE;
  new_msg->motor_temp = parsed_rpt.MOTOR_TEMPERATURE;
  new_msg->angular_speed = parsed_rpt.ANGULAR_SPEED_phys;

  return new_msg;
}

std::shared_ptr<void> Dbc13Api::ParseMotorRpt3(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::MotorRpt3>();

  BRAKE_MOTOR_RPT_3_t parsed_rpt;
  Unpack_BRAKE_MOTOR_RPT_3_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->torque_output = parsed_rpt.TORQUE_OUTPUT_phys;
  new_msg->torque_input = parsed_rpt.TORQUE_INPUT_phys;

  return new_msg;
}

std::shared_ptr<void> Dbc13Api::ParseNotificationRpt(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::NotificationRpt>();

  NOTIFICATION_RPT_t parsed_rpt;
  Unpack_NOTIFICATION_RPT_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->light_color = parsed_rpt.LIGHT_COLOR;
  new_msg->light_status = parsed_rpt.LIGHT_STATUS;
  new_msg->buzzer_status = parsed_rpt.BUZZER_STATUS;

  return new_msg;
}

std::shared_ptr<void> Dbc13Api::ParseOccupancyRpt(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::OccupancyRpt>();

  OCCUPANCY_RPT_t parsed_rpt;
  Unpack_OCCUPANCY_RPT_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->driver_seat_occupied = parsed_rpt.DRIVER_SEAT_OCCUPIED;
  new_msg->driver_seat_occupied_avail = parsed_rpt.DRIVER_SEAT_OCCUPIED_AVAIL;
  new_msg->passenger_seat_occupied = parsed_rpt.PASS_SEAT_OCCUPIED;
  new_msg->passenger_seat_occupied_avail = parsed_rpt.PASS_SEAT_OCCUPIED_AVAIL;
  new_msg->rear_seat_occupied = parsed_rpt.REAR_SEAT_OCCUPIED;
  new_msg->rear_seat_occupied_avail = parsed_rpt.REAR_SEAT_OCCUPIED_AVAIL;
  new_msg->driver_seatbelt_buckled = parsed_rpt.DRIVER_SEATBELT_BUCKLED;
  new_msg->driver_seatbelt_buckled_avail = parsed_rpt.DRIVER_SEATBELT_BUCKLED_AVAIL;
  new_msg->passenger_seatbelt_buckled = parsed_rpt.PASS_SEATBELT_BUCKLED;
  new_msg->passenger_seatbelt_buckled_avail = parsed_rpt.PASS_SEATBELT_BUCKLED_AVAIL;

  new_msg->center_rear_seatbelt_buckled = parsed_rpt.CTR_REAR_SEATBELT_BUCKLED;
  new_msg->center_rear_seatbelt_buckled_avail = parsed_rpt.CTR_REAR_SEATBELT_BUCKLED_AVAIL;

  new_msg->driver_rear_seatbelt_buckled = parsed_rpt.DRIVER_SEATBELT_BUCKLED;
  new_msg->driver_rear_seatbelt_buckled_avail = parsed_rpt.DRIVER_SEATBELT_BUCKLED_AVAIL;
  new_msg->pass_rear_seatbelt_buckled = parsed_rpt.PASS_REAR_SEATBELT_BUCKLED;
  new_msg->pass_rear_seatbelt_buckled_avail = parsed_rpt.PASS_REAR_SEATBELT_BUCKLED_AVAIL;

  return new_msg;
}

std::shared_ptr<void> Dbc13Api::ParseParkingBrakeAuxRpt(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::ParkingBrakeAuxRpt>();

  PARKING_BRAKE_AUX_RPT_t parsed_rpt;
  Unpack_PARKING_BRAKE_AUX_RPT_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->parking_brake_status = parsed_rpt.PARKING_BRAKE_STATUS;
  new_msg->parking_brake_status_avail = parsed_rpt.PARKING_BRAKE_STATUS_AVAIL;

  return new_msg;
}


std::shared_ptr<void> Dbc13Api::ParseSystemRptBoolWithControlStatus(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::SystemRptBoolWithControlStatus>();

  PARKING_BRAKE_RPT_t parsed_rpt;
  Unpack_PARKING_BRAKE_RPT_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->enabled = parsed_rpt.ENABLED;
  new_msg->override_active = parsed_rpt.OVERRIDE_ACTIVE;
  new_msg->command_output_fault = parsed_rpt.COMMAND_OUTPUT_FAULT;
  new_msg->input_output_fault = parsed_rpt.INPUT_OUTPUT_FAULT;
  new_msg->output_reported_fault = parsed_rpt.OUTPUT_REPORTED_FAULT;
  new_msg->pacmod_fault = parsed_rpt.PACMOD_FAULT;
  new_msg->vehicle_fault = parsed_rpt.VEHICLE_FAULT;
  new_msg->command_timeout = parsed_rpt.COMMAND_TIMEOUT;
  new_msg->manual_input = parsed_rpt.MANUAL_INPUT;
  new_msg->command = parsed_rpt.COMMANDED_VALUE;
  new_msg->output = parsed_rpt.OUTPUT_VALUE;
  new_msg->control_status = parsed_rpt.CONTROL_STATUS;
  new_msg->controlling_systems = parsed_rpt.CONTROLLING_SYSTEMS;
  new_msg->performance_status = parsed_rpt.PERFORMANCE_STATUS;

  return new_msg;
}

std::shared_ptr<void> Dbc13Api::ParseRearLightsRpt(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::RearLightsRpt>();

  REAR_LIGHTS_RPT_t parsed_rpt;
  Unpack_REAR_LIGHTS_RPT_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->brake_lights_on = parsed_rpt.BRAKE_LIGHTS_ON;
  new_msg->brake_lights_on_avail = parsed_rpt.BRAKE_LIGHTS_ON_AVAIL;
  new_msg->reverse_lights_on = parsed_rpt.REVERSE_LIGHTS_ON;
  new_msg->reverse_lights_on_avail = parsed_rpt.REVERSE_LIGHTS_ON_AVAIL;

  return new_msg;
}

std::shared_ptr<void> Dbc13Api::ParseRemoteStopRpt(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::RemoteStopRpt>();

  REMOTE_STOP_RPT_t parsed_rpt;
  Unpack_REMOTE_STOP_RPT_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->remote_stop_command = parsed_rpt.REMOTE_STOP_COMMAND;
  new_msg->remote_stop_sender_id = parsed_rpt.REMOTE_STOP_SENDER_ID;
  new_msg->remote_stop_timeout = parsed_rpt.REMOTE_STOP_TIMEOUT;
  new_msg->remote_stop_fault = parsed_rpt.REMOTE_STOP_FAULT;
  new_msg->remote_stop_mode_change_allowed = parsed_rpt.REMOTE_STOP_MODE_CHANGE_ALLOWED;

  return new_msg;
}

std::shared_ptr<void> Dbc13Api::ParseSafetyFuncCriticalStopRpt(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::SafetyFuncCriticalStopRpt>();

  SAFETY_FUNC_CRITICAL_STOP_RPT_t parsed_rpt;
  Unpack_SAFETY_FUNC_CRITICAL_STOP_RPT_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->automaual_opctrl_fault = parsed_rpt.AUTOMS_MAN_OPCTRL_FAULT;
  new_msg->remote_stop_fault = parsed_rpt.REMOTE_STOP_FAULT;
  new_msg->safety_brake_opctrl_off = parsed_rpt.SAFETY_BRAKE_OPCTRL_OFF;
  new_msg->safety_brake_cmd_timeout = parsed_rpt.SAFETY_BRAKE_CMD_TIMEOUT;
  new_msg->safety_func_cmd_timeout = parsed_rpt.SAFETY_FUNC_CMD_TIMEOUT;
  new_msg->safety_func_critical_stop_1_cmd = parsed_rpt.SAFETY_FUNC_CRITICAL_STOP_1_CMD;
  new_msg->safety_func_critical_stop_2_cmd = parsed_rpt.SAFETY_FUNC_CRITICAL_STOP_2_CMD;
  new_msg->safety_func_none_cmd = parsed_rpt.SAFETY_FUNC_NONE_CMD;
  new_msg->pacmod_system_timeout = parsed_rpt.PACMOD_SYSTEM_TIMEOUT;
  new_msg->pacmod_system_fault = parsed_rpt.PACMOD_SYSTEM_FAULT;
  new_msg->pacmod_system_not_active = parsed_rpt.PACMOD_SYSTEM_NOT_ACTIVE;
  new_msg->vehicle_report_timeout = parsed_rpt.VEHICLE_REPORT_TIMEOUT;
  new_msg->vehicle_report_fault = parsed_rpt.VEHICLE_REPORT_FAULT;
  new_msg->low_engine_rpm = parsed_rpt.LOW_ENGINE_RPM;
  new_msg->pri_safety_brake_signal_1_fault = parsed_rpt.PRI_SAFETY_BRAKE_SIGNAL_1_FAULT;
  new_msg->pri_safety_brake_signal_2_fault = parsed_rpt.PRI_SAFETY_BRAKE_SIGNAL_2_FAULT;
  new_msg->sec_safety_brake_signal_1_fault = parsed_rpt.SEC_SAFETY_BRAKE_SIGNAL_1_FAULT;
  new_msg->sec_safety_brake_signal_2_fault = parsed_rpt.SEC_SAFETY_BRAKE_SIGNAL_2_FAULT;
  new_msg->primary_processor_fault = parsed_rpt.PRIMARY_PROCESSOR_FAULT;
  new_msg->secondary_processor_fault = parsed_rpt.SECONDARY_PROCESSOR_FAULT;
  new_msg->remote_stop_cmd = parsed_rpt.REMOTE_STOP_CMD;
  new_msg->pri_safety_brake_pressure_fault = parsed_rpt.PRI_SAFETY_BRAKE_PRESSURE_FAULT;

  return new_msg;
}

std::shared_ptr<void> Dbc13Api::ParseSafetyFuncRpt(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::SafetyFuncRpt>();

  SAFETY_FUNC_RPT_t parsed_rpt;
  Unpack_SAFETY_FUNC_RPT_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->commanded_val = parsed_rpt.COMMANDED_VALUE;
  new_msg->state = parsed_rpt.STATE;
  new_msg->automanual_opctrl = parsed_rpt.AUTOMS_MAN_OPCTRL;
  new_msg->cabin_safety_brake_opctrl = parsed_rpt.CABIN_SAFETY_BRAKE_OPCTRL;
  new_msg->remote_stop_status = parsed_rpt.REMOTE_STOP_STATUS;
  new_msg->engine_status = parsed_rpt.ENGINE_STATUS;
  new_msg->pacmod_system_status = parsed_rpt.PACMOD_SYSTEM_STATUS;
  new_msg->user_pc_fault = parsed_rpt.USER_PC_FAULT;
  new_msg->pacmod_system_fault = parsed_rpt.PACMOD_SYSTEM_FAULT;
  new_msg->manual_state_obtainable = parsed_rpt.MANUAL_READY_STATE_OBTAINABLE;
  new_msg->auto_ready_state_obtainable = parsed_rpt.AUTOMS_READY_STATE_OBTAINABLE;
  new_msg->auto_state_obtainable = parsed_rpt.AUTOMS_STATE_OBTAINABLE;
  new_msg->manual_ready_state_obtainable = parsed_rpt.MANUAL_READY_STATE_OBTAINABLE;
  new_msg->critical_stop1_state_obtainable = parsed_rpt.CRITICAL_STOP1_STATE_OBTAINABLE;
  new_msg->critical_stop2_state_obtainable = parsed_rpt.CRITICAL_STOP2_STATE_OBTAINABLE;

  return new_msg;
}

std::shared_ptr<void> Dbc13Api::ParseSafetyFuncRpt2(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::SafetyFuncRpt2>();

  SAFETY_FUNC_RPT_2_t parsed_rpt;
  Unpack_SAFETY_FUNC_RPT_2_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->commanded_state = parsed_rpt.COMMANDED_STATE;
  new_msg->state = parsed_rpt.STATE;
  new_msg->manual_state_obtainable = parsed_rpt.MANUAL_STATE_OBTAINABLE;
  new_msg->auto_ready_state_obtainable = parsed_rpt.AUTOMS_READY_STATE_OBTAINABLE;
  new_msg->auto_state_obtainable = parsed_rpt.AUTOMS_STATE_OBTAINABLE;
  new_msg->manual_ready_state_obtainable = parsed_rpt.MANUAL_READY_STATE_OBTAINABLE;

  return new_msg;
}

std::shared_ptr<void> Dbc13Api::ParseSafetyResponseRpt(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::SafetyResponseRpt>();

  SAFETY_RESPONSE_RPT_t parsed_rpt;
  Unpack_SAFETY_RESPONSE_RPT_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->safety_response = parsed_rpt.SAFETY_RESPONSE;

  return new_msg;
}

std::shared_ptr<void> Dbc13Api::ParseShiftAuxRpt(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::ShiftAuxRpt>();

  SHIFT_AUX_RPT_t parsed_rpt;
  Unpack_SHIFT_AUX_RPT_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->between_gears = parsed_rpt.BETWEEN_GEARS;
  new_msg->stay_in_neutral_mode = parsed_rpt.STAY_IN_NEUTRAL_MODE;
  new_msg->brake_interlock_active = parsed_rpt.BRAKE_INTERLOCK_ACTIVE;
  new_msg->speed_interlock_active = parsed_rpt.SPEED_INTERLOCK_ACTIVE;
  new_msg->write_to_config = parsed_rpt.WRITE_TO_CONFIG;
  new_msg->calibration_status = parsed_rpt.CALIBRATION_STATUS;
  new_msg->between_gears_avail = parsed_rpt.BETWEEN_GEARS_AVAIL;
  new_msg->stay_in_neutral_mode_avail = parsed_rpt.STAY_IN_NEUTRAL_MODE_AVAIL;
  new_msg->brake_interlock_active_avail = parsed_rpt.BRAKE_INTERLOCK_ACTIVE_AVAIL;
  new_msg->speed_interlock_active_avail = parsed_rpt.SPEED_INTERLOCK_ACTIVE_AVAIL;
  new_msg->write_to_config_is_valid = parsed_rpt.WRITE_TO_CONFIG_IS_VALID;
  new_msg->gear_number_avail = parsed_rpt.GEAR_NUMBER_AVAIL;
  new_msg->shift_mode_avail = parsed_rpt.SHIFT_MODE_AVAIL;
  new_msg->shift_in_progress_avail = parsed_rpt.SHIFT_IN_PROGRESS_AVAIL;
  new_msg->gear_number = parsed_rpt.GEAR_NUMBER;
  new_msg->shift_mode = parsed_rpt.SHIFT_MODE;
  new_msg->shift_in_progress = parsed_rpt.SHIFT_IN_PROGRESS;
  new_msg->driveline_engaged = parsed_rpt.DRIVELINE_ENGAGED;
  new_msg->actual_gear_ratio = parsed_rpt.ACTUAL_GEAR_RATIO_phys;
  new_msg->driveline_engaged_avail = parsed_rpt.DRIVELINE_ENGAGED_AVAIL;
  new_msg->actual_gear_ratio_avail = parsed_rpt.ACTUAL_GEAR_RATIO_AVAIL;

  return new_msg;
}

std::shared_ptr<void> Dbc13Api::ParseSoftwareVersionRpt(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::SoftwareVersionRpt>();

  SOFTWARE_VERSION_RPT_00_t parsed_rpt;
  Unpack_SOFTWARE_VERSION_RPT_00_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->mjr = parsed_rpt.MAJOR;
  new_msg->mnr = parsed_rpt.MINOR;
  new_msg->patch = parsed_rpt.PATCH;
  new_msg->build0 = parsed_rpt.BUILD0;
  new_msg->build1 = parsed_rpt.BUILD1;
  new_msg->build2 = parsed_rpt.BUILD2;
  new_msg->build3 = parsed_rpt.BUILD3;

  return new_msg;
}

std::shared_ptr<void> Dbc13Api::ParseSteeringAuxRpt(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::SteeringAuxRpt>();

  STEERING_AUX_RPT_t parsed_rpt;
  Unpack_STEERING_AUX_RPT_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->steering_fluid_pressure = parsed_rpt.STEERING_FLUID_PRESSURE_phys;
  new_msg->steering_torque = parsed_rpt.STEERING_TORQUE_phys;
  new_msg->rotation_rate = parsed_rpt.ROTATION_RATE_phys;
  new_msg->operator_interaction = parsed_rpt.OPERATOR_INTERACTION;
  new_msg->rotation_rate_sign = parsed_rpt.ROTATION_RATE_SIGN;
  new_msg->vehicle_angle_calib_status = parsed_rpt.VEHICLE_ANGLE_CALIB_STATUS;
  new_msg->steering_limiting_active = parsed_rpt.STEERING_LIMITING_ACTIVE;
  new_msg->calibration_status = parsed_rpt.CALIBRATION_STATUS;
  new_msg->steering_controller_type = parsed_rpt.STEERING_CONTROLLER_TYPE;
  new_msg->steering_torque_avail = parsed_rpt.STEERING_TORQUE_AVAIL;
  new_msg->rotation_rate_avail = parsed_rpt.ROTATION_RATE_AVAIL;
  new_msg->operator_interaction_avail = parsed_rpt.OPERATOR_INTERACTION_AVAIL;
  new_msg->vehicle_angle_calib_status_avail = parsed_rpt.VEHICLE_ANGLE_CALIB_STATUS_AVAIL;
  new_msg->steering_limiting_active_avail = parsed_rpt.STEERING_LIMITING_ACTIVE_AVAIL;
  new_msg->steering_controller_type_avail = parsed_rpt.STEERING_CONTROLLER_TYPE_AVAIL;

  return new_msg;
}

std::shared_ptr<void> Dbc13Api::ParseSteeringCmdLimitRpt(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::SteeringCmdLimitRpt>();

  STEERING_CMD_LIMIT_RPT_t parsed_rpt;
  Unpack_STEERING_CMD_LIMIT_RPT_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->pos_cmd_limit = parsed_rpt.POSITION_CMD_LIMIT_phys;
  new_msg->limited_pos_cmd = parsed_rpt.LIMITED_POSITION_CMD_phys;
  new_msg->rotation_rate_cmd_limit = parsed_rpt.ROTATION_RATE_CMD_LIMIT_phys;
  new_msg->limited_rotation_rate_cmd = parsed_rpt.LIMITED_ROTATION_RATE_phys;

  return new_msg;
}

std::shared_ptr<void> Dbc13Api::ParseSteeringRpt(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::SystemRptFloat>();

  STEERING_RPT_t parsed_rpt;
  Unpack_STEERING_RPT_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->enabled = parsed_rpt.ENABLED;
  new_msg->override_active = parsed_rpt.OVERRIDE_ACTIVE;
  new_msg->command_output_fault = parsed_rpt.COMMAND_OUTPUT_FAULT;
  new_msg->input_output_fault = parsed_rpt.INPUT_OUTPUT_FAULT;
  new_msg->output_reported_fault = parsed_rpt.OUTPUT_REPORTED_FAULT;
  new_msg->pacmod_fault = parsed_rpt.PACMOD_FAULT;
  new_msg->vehicle_fault = parsed_rpt.VEHICLE_FAULT;
  new_msg->command_timeout = parsed_rpt.COMMAND_TIMEOUT;
  new_msg->manual_input = parsed_rpt.MANUAL_INPUT_phys;
  new_msg->command = parsed_rpt.COMMANDED_VALUE_phys;
  new_msg->output = parsed_rpt.OUTPUT_VALUE_phys;
  new_msg->control_status = parsed_rpt.CONTROL_STATUS;
  new_msg->controlling_systems = parsed_rpt.CONTROLLING_SYSTEMS;
  new_msg->performance_status = parsed_rpt.PERFORMANCE_STATUS;

  return new_msg;
}

std::shared_ptr<void> Dbc13Api::ParseSupervisoryCtrl(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::SupervisoryCtrl>();

  SUPERVISORY_CTRL_t parsed_rpt;
  Unpack_SUPERVISORY_CTRL_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->enable = parsed_rpt.ENABLE;
  new_msg->counter = parsed_rpt.COUNTER;
  new_msg->complement = parsed_rpt.COMPLEMENT;

  return new_msg;
}

std::shared_ptr<void> Dbc13Api::ParseSystemRptBool(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::SystemRptBool>();

  HORN_RPT_t parsed_rpt;
  Unpack_HORN_RPT_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->enabled = parsed_rpt.ENABLED;
  new_msg->override_active = parsed_rpt.OVERRIDE_ACTIVE;
  new_msg->command_output_fault = parsed_rpt.COMMAND_OUTPUT_FAULT;
  new_msg->input_output_fault = parsed_rpt.INPUT_OUTPUT_FAULT;
  new_msg->output_reported_fault = parsed_rpt.OUTPUT_REPORTED_FAULT;
  new_msg->pacmod_fault = parsed_rpt.PACMOD_FAULT;
  new_msg->vehicle_fault = parsed_rpt.VEHICLE_FAULT;
  new_msg->command_timeout = parsed_rpt.COMMAND_TIMEOUT;
  new_msg->manual_input = parsed_rpt.MANUAL_INPUT;
  new_msg->command = parsed_rpt.COMMANDED_VALUE;
  new_msg->output = parsed_rpt.OUTPUT_VALUE;

  return new_msg;
}

std::shared_ptr<void> Dbc13Api::ParseSystemRptFloat(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::SystemRptFloat>();

  ACCEL_RPT_t parsed_rpt;
  Unpack_ACCEL_RPT_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->enabled = parsed_rpt.ENABLED;
  new_msg->override_active = parsed_rpt.OVERRIDE_ACTIVE;
  new_msg->command_output_fault = parsed_rpt.COMMAND_OUTPUT_FAULT;
  new_msg->input_output_fault = parsed_rpt.INPUT_OUTPUT_FAULT;
  new_msg->output_reported_fault = parsed_rpt.OUTPUT_REPORTED_FAULT;
  new_msg->pacmod_fault = parsed_rpt.PACMOD_FAULT;
  new_msg->vehicle_fault = parsed_rpt.VEHICLE_FAULT;
  new_msg->command_timeout = parsed_rpt.COMMAND_TIMEOUT;
  new_msg->manual_input = parsed_rpt.MANUAL_INPUT_phys;
  new_msg->command = parsed_rpt.COMMANDED_VALUE_phys;
  new_msg->output = parsed_rpt.OUTPUT_VALUE_phys;
  new_msg->control_status = parsed_rpt.CONTROL_STATUS;
  new_msg->controlling_systems = parsed_rpt.CONTROLLING_SYSTEMS;
  new_msg->performance_status = parsed_rpt.PERFORMANCE_STATUS;

  return new_msg;
}

std::shared_ptr<void> Dbc13Api::ParseSystemRptInt(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::SystemRptInt>();

  SHIFT_RPT_t parsed_rpt;
  Unpack_SHIFT_RPT_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->enabled = parsed_rpt.ENABLED;
  new_msg->override_active = parsed_rpt.OVERRIDE_ACTIVE;
  new_msg->command_output_fault = parsed_rpt.COMMAND_OUTPUT_FAULT;
  new_msg->input_output_fault = parsed_rpt.INPUT_OUTPUT_FAULT;
  new_msg->output_reported_fault = parsed_rpt.OUTPUT_REPORTED_FAULT;
  new_msg->pacmod_fault = parsed_rpt.PACMOD_FAULT;
  new_msg->vehicle_fault = parsed_rpt.VEHICLE_FAULT;
  new_msg->command_timeout = parsed_rpt.COMMAND_TIMEOUT;
  new_msg->manual_input = parsed_rpt.MANUAL_INPUT;
  new_msg->command = parsed_rpt.COMMANDED_VALUE;
  new_msg->output = parsed_rpt.OUTPUT_VALUE;

  return new_msg;
}

std::shared_ptr<void> Dbc13Api::ParseSystemRptIntWithControlStatus(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::SystemRptIntWithControlStatus>();

  SHIFT_RPT_t parsed_rpt;
  Unpack_SHIFT_RPT_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->enabled = parsed_rpt.ENABLED;
  new_msg->override_active = parsed_rpt.OVERRIDE_ACTIVE;
  new_msg->command_output_fault = parsed_rpt.COMMAND_OUTPUT_FAULT;
  new_msg->input_output_fault = parsed_rpt.INPUT_OUTPUT_FAULT;
  new_msg->output_reported_fault = parsed_rpt.OUTPUT_REPORTED_FAULT;
  new_msg->pacmod_fault = parsed_rpt.PACMOD_FAULT;
  new_msg->vehicle_fault = parsed_rpt.VEHICLE_FAULT;
  new_msg->command_timeout = parsed_rpt.COMMAND_TIMEOUT;
  new_msg->manual_input = parsed_rpt.MANUAL_INPUT;
  new_msg->command = parsed_rpt.COMMANDED_VALUE;
  new_msg->output = parsed_rpt.OUTPUT_VALUE;
  new_msg->control_status = parsed_rpt.CONTROL_STATUS;
  new_msg->controlling_systems = parsed_rpt.CONTROLLING_SYSTEMS;
  new_msg->performance_status = parsed_rpt.PERFORMANCE_STATUS;

  return new_msg;
}

std::shared_ptr<void> Dbc13Api::ParseTipperBodyAuxRpt(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::TipperBodyAuxRpt>();

  TIPPER_BODY_AUX_RPT_00_t parsed_rpt;
  Unpack_TIPPER_BODY_AUX_RPT_00_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->roll_angle = parsed_rpt.ROLL_ANGLE_phys;
  new_msg->pitch_angle   = parsed_rpt.PITCH_ANGLE_phys;
  new_msg->door_fully_open = parsed_rpt.DOOR_FULLY_OPEN;
  new_msg->door_fully_closed = parsed_rpt.DOOR_FULLY_CLOSED;
  new_msg->body_fully_lowered = parsed_rpt.BODY_FULLY_LOWERED;
  new_msg->body_fully_raised = parsed_rpt.BODY_FULLY_RAISED;

  return new_msg;
}

std::shared_ptr<void> Dbc13Api::ParseTirePressureExtendedRpt(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::TirePressureExtendedRpt>();

  TIRE_PRESSURE_EXTENDED_RPT_t parsed_rpt;
  Unpack_TIRE_PRESSURE_EXTENDED_RPT_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->tire_chassis_index_1 = parsed_rpt.TIRE_CHASSIS_INDEX_1;
  new_msg->tire_chassis_index_2 = parsed_rpt.TIRE_CHASSIS_INDEX_2;
  new_msg->tire_chassis_index_3 = parsed_rpt.TIRE_CHASSIS_INDEX_3;
  new_msg->tire_axle_1 = parsed_rpt.TIRE_AXLE_1;
  new_msg->tire_axle_2 = parsed_rpt.TIRE_AXLE_2;
  new_msg->tire_axle_3 = parsed_rpt.TIRE_AXLE_3;
  new_msg->tire_position_1 = parsed_rpt.TIRE_POSITION_1;
  new_msg->tire_position_2 = parsed_rpt.TIRE_POSITION_2;
  new_msg->tire_position_3 = parsed_rpt.TIRE_POSITION_3;
  new_msg->tire_pressure_1 = parsed_rpt.TIRE_PRESSURE_1_phys;
  new_msg->tire_pressure_2 = parsed_rpt.TIRE_PRESSURE_2_phys;
  new_msg->tire_pressure_3 = parsed_rpt.TIRE_PRESSURE_3_phys;

  return new_msg;
}

std::shared_ptr<void> Dbc13Api::ParseTirePressureRpt(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::TirePressureRpt>();

  TIRE_PRESSURE_RPT_t parsed_rpt;
  Unpack_TIRE_PRESSURE_RPT_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->front_left_tire_pressure = parsed_rpt.FRONT_LEFT_TIRE_PRESSURE_phys;
  new_msg->front_right_tire_pressure = parsed_rpt.FRONT_RIGHT_TIRE_PRESSURE_phys;
  new_msg->rear_left_tire_pressure = parsed_rpt.REAR_LEFT_TIRE_PRESSURE_phys;
  new_msg->rear_right_tire_pressure = parsed_rpt.REAR_RIGHT_TIRE_PRESSURE_phys;

  return new_msg;
}

std::shared_ptr<void> Dbc13Api::ParseTireTemperatureExtendedRpt(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::TireTemperatureExtendedRpt>();

  TIRE_TEMPERATURE_EXTENDED_RPT_t parsed_rpt;
  Unpack_TIRE_TEMPERATURE_EXTENDED_RPT_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->tire_chassis_index_1 = parsed_rpt.TIRE_CHASSIS_INDEX_1;
  new_msg->tire_chassis_index_2 = parsed_rpt.TIRE_CHASSIS_INDEX_2;
  new_msg->tire_chassis_index_3 = parsed_rpt.TIRE_CHASSIS_INDEX_3;
  new_msg->tire_axle_1 = parsed_rpt.TIRE_AXLE_1;
  new_msg->tire_axle_2 = parsed_rpt.TIRE_AXLE_2;
  new_msg->tire_axle_3 = parsed_rpt.TIRE_AXLE_3;
  new_msg->tire_position_1 = parsed_rpt.TIRE_POSITION_1;
  new_msg->tire_position_2 = parsed_rpt.TIRE_POSITION_2;
  new_msg->tire_position_3 = parsed_rpt.TIRE_POSITION_3;
  new_msg->tire_temperature_1 = parsed_rpt.TIRE_TEMPERATURE_1_phys;
  new_msg->tire_temperature_2 = parsed_rpt.TIRE_TEMPERATURE_2_phys;
  new_msg->tire_temperature_3 = parsed_rpt.TIRE_TEMPERATURE_3_phys;

  return new_msg;
}

std::shared_ptr<void> Dbc13Api::ParseTrailerBrakePressureRpt(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::TrailerBrakePressureRpt>();

  TRAILER_BRAKE_PRESSURE_RPT_00_t parsed_rpt;
  Unpack_TRAILER_BRAKE_PRESSURE_RPT_00_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->brake_pressure_axle_1_left = parsed_rpt.BRAKE_PRESSURE_AXLE_1_LEFT_phys;
  new_msg->brake_pressure_axle_1_right = parsed_rpt.BRAKE_PRESSURE_AXLE_1_RIGHT_phys;
  new_msg->brake_pressure_axle_2_left = parsed_rpt.BRAKE_PRESSURE_AXLE_2_LEFT_phys;
  new_msg->brake_pressure_axle_2_right = parsed_rpt.BRAKE_PRESSURE_AXLE_2_RIGHT_phys;
  new_msg->brake_pressure_axle_3_left = parsed_rpt.BRAKE_PRESSURE_AXLE_3_LEFT_phys;
  new_msg->brake_pressure_axle_3_right = parsed_rpt.BRAKE_PRESSURE_AXLE_3_RIGHT_phys;
  new_msg->pneumatic_supply_pressure = parsed_rpt.PNEUMATIC_SUPPLY_PRESSURE_phys;

  return new_msg;
}

std::shared_ptr<void> Dbc13Api::ParseTrailerFaultRpt(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::TrailerFaultRpt>();

  TRAILER_FAULT_RPT_00_t parsed_rpt;
  Unpack_TRAILER_FAULT_RPT_00_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->service_brake_status = parsed_rpt.SERVICE_BRAKE_STATUS;
  new_msg->abs_status = parsed_rpt.ABS_STATUS;
  new_msg->electrical_supply_status = parsed_rpt.ELECTRICAL_SUPPLY_STATUS;
  new_msg->pneumatic_supply_status = parsed_rpt.PNEUMATIC_SUPPLY_STATUS;
  new_msg->amber_warning_lamp_request = parsed_rpt.AMBER_WARNING_LAMP_REQUEST;
  new_msg->red_warning_lamp_request = parsed_rpt.RED_WARNING_LAMP_REQUEST;

  return new_msg;
}

std::shared_ptr<void> Dbc13Api::ParseTrailerPayloadRpt(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::TrailerPayloadRpt>();

  TRAILER_PAYLOAD_RPT_00_t parsed_rpt;
  Unpack_TRAILER_PAYLOAD_RPT_00_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->payload_00 = parsed_rpt.PAYLOAD_00_phys;
  new_msg->tare_00 = parsed_rpt.TARE_00_phys;
  new_msg->payload_01 = parsed_rpt.PAYLOAD_01_phys;
  new_msg->tare_01 = parsed_rpt.TARE_01_phys;

  return new_msg;
}
std::shared_ptr<void> Dbc13Api::ParseTurnAuxRpt(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::TurnAuxRpt>();

  TURN_AUX_RPT_t parsed_rpt;
  Unpack_TURN_AUX_RPT_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->driver_blinker_bulb_on = parsed_rpt.DRIVER_BLINKER_BULB_ON;
  new_msg->passenger_blinker_bulb_on = parsed_rpt.PASS_BLINKER_BULB_ON;
  new_msg->driver_blinker_bulb_on_avail = parsed_rpt.DRIVER_BLINKER_BULB_ON_AVAIL;
  new_msg->passenger_blinker_bulb_on_avail = parsed_rpt.PASS_BLINKER_BULB_ON_AVAIL;

  return new_msg;
}

std::shared_ptr<void> Dbc13Api::ParseVehicleDynamicsRpt(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::VehicleDynamicsRpt>();

  VEH_DYNAMICS_RPT_t parsed_rpt;
  Unpack_VEH_DYNAMICS_RPT_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->veh_g_forces = parsed_rpt.VEH_G_FORCES_phys;

  return new_msg;
}

std::shared_ptr<void> Dbc13Api::ParseVehicleFaultRpt(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::VehicleFaultRpt>();

  VEHICLE_FAULT_RPT_t parsed_rpt;
  Unpack_VEHICLE_FAULT_RPT_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->engine_check_light = parsed_rpt.ENGINE_CHECK_LIGHT;
  new_msg->engine_check_light_avail = parsed_rpt.ENGINE_CHECK_LIGHT_AVAIL;
  new_msg->trc_fault_light = parsed_rpt.TRC_FAULT_LIGHT;
  new_msg->trc_fault_light_avail = parsed_rpt.TRC_FAULT_LIGHT_AVAIL;
  new_msg->trc_off_fault_light = parsed_rpt.TRC_OFF_FAULT_LIGHT;
  new_msg->trc_off_fault_light_avail = parsed_rpt.TRC_OFF_FAULT_LIGHT_AVAIL;
  new_msg->antilock_brake_fault_light = parsed_rpt.ANTILOCK_BRAKE_FAULT_LIGHT;
  new_msg->antilock_brake_fault_light_avail = parsed_rpt.ANTILOCK_BRAKE_FAULT_LIGHT_AVAIL;
  new_msg->tire_fault_light = parsed_rpt.TIRE_FAULT_LIGHT;
  new_msg->tire_fault_light_avail = parsed_rpt.TIRE_FAULT_LIGHT_AVAIL;
  new_msg->air_bags_fault_light = parsed_rpt.AIR_BAGS_FAULT_LIGHT;
  new_msg->air_bags_fault_light_avail = parsed_rpt.AIR_BAGS_FAULT_LIGHT_AVAIL;
  new_msg->low_engine_oil_pressure = parsed_rpt.LOW_ENGINE_OIL_PRESSURE;
  new_msg->low_engine_oil_pressure_avail = parsed_rpt.LOW_ENGINE_OIL_PRESSURE_AVAIL;
  new_msg->brake_fault = parsed_rpt.BRAKE_FAULT;
  new_msg->brake_fault_avail = parsed_rpt.BRAKE_FAULT_AVAIL;
  new_msg->brk_applied_power_reduced = parsed_rpt.BRK_APPLIED_POWER_REDUCED;
  new_msg->brk_applied_power_reduced_avail = parsed_rpt.BRK_APPLIED_POWER_REDUCED_AVAIL;
  new_msg->steering_loss_stop_safely = parsed_rpt.STEERING_LOSS_STOP_SAFELY;
  new_msg->steering_loss_stop_safely_avail = parsed_rpt.STEERING_LOSS_STOP_SAFELY_AVAIL;
  new_msg->steering_fault_service_now = parsed_rpt.STEERING_FAULT_SERVICE_NOW;
  new_msg->steering_fault_service_now_avail = parsed_rpt.STEERING_FAULT_SERVICE_NOW_AVAIL;
  new_msg->xmsn_fault_service_now = parsed_rpt.XMSN_FAULT_SERVICE_NOW;
  new_msg->xmsn_fault_service_now_avail = parsed_rpt.XMSN_FAULT_SERVICE_NOW_AVAIL;
  new_msg->xmsn_over_temp_stop_safely = parsed_rpt.XMSN_OVER_TEMP_STOP_SAFELY;
  new_msg->xmsn_over_temp_stop_safely_avail = parsed_rpt.XMSN_OVER_TEMP_STOP_SAFELY_AVAIL;
  new_msg->low_battery_features_off = parsed_rpt.LOW_BATTERY_FEATURES_OFF;
  new_msg->low_battery_features_off_avail = parsed_rpt.LOW_BATTERY_FEATURES_OFF_AVAIL;
  new_msg->charging_system_fault = parsed_rpt.CHARGING_SYSTEM_FAULT;
  new_msg->charging_system_fault_avail = parsed_rpt.CHARGING_SYSTEM_FAULT_AVAIL;

  return new_msg;
}

std::shared_ptr<void> Dbc13Api::ParseVehicleFaultRpt2(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::VehicleFaultRpt2>();

  VEHICLE_FAULT_RPT_2_t parsed_rpt;
  Unpack_VEHICLE_FAULT_RPT_2_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->brakes_amber_lamp = parsed_rpt.BRAKES_AMBER_LAMP;
  new_msg->brakes_red_lamp = parsed_rpt.BRAKES_RED_LAMP;
  new_msg->chassis_amber_lamp = parsed_rpt.CHASSIS_AMBER_LAMP;
  new_msg->chassis_red_lamp = parsed_rpt.CHASSIS_RED_LAMP;
  new_msg->electronic_brakes_amber_lamp = parsed_rpt.ELECTRONIC_BRAKES_AMBER_LAMP;
  new_msg->electronic_brakes_red_lamp = parsed_rpt.ELECTRONIC_BRAKES_RED_LAMP;
  new_msg->engine_amber_lamp = parsed_rpt.ENGINE_AMBER_LAMP;
  new_msg->engine_red_lamp = parsed_rpt.ENGINE_RED_LAMP;
  new_msg->engine_brake_amber_lamp = parsed_rpt.ENGINE_BRAKE_AMBER_LAMP;
  new_msg->engine_brake_red_lamp = parsed_rpt.ENGINE_BRAKE_RED_LAMP;
  new_msg->management_ecu_amber_lamp = parsed_rpt.MANAGEMENT_ECU_AMBER_LAMP;
  new_msg->management_ecu_red_lamp = parsed_rpt.MANAGEMENT_ECU_RED_LAMP;
  new_msg->pneumatic_amber_lamp = parsed_rpt.PNEUMATIC_AMBER_LAMP;
  new_msg->pneumatic_red_lamp = parsed_rpt.PNEUMATIC_RED_LAMP;
  new_msg->restraints_amber_lamp = parsed_rpt.RESTRAINTS_AMBER_LAMP;
  new_msg->restraints_red_lamp = parsed_rpt.RESTRAINTS_RED_LAMP;
  new_msg->trailer_abs_amber_lamp = parsed_rpt.TRAILER_ABS_AMBER_LAMP;
  new_msg->trailer_abs_red_lamp = parsed_rpt.TRAILER_ABS_RED_LAMP;
  new_msg->transmission_amber_lamp = parsed_rpt.TRANSMISSION_AMBER_LAMP;
  new_msg->transmission_red_lamp = parsed_rpt.TRANSMISSION_RED_LAMP;

  return new_msg;
}

std::shared_ptr<void> Dbc13Api::ParseVehicleSpeedRpt(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::VehicleSpeedRpt>();

  VEHICLE_SPEED_RPT_t parsed_rpt;
  Unpack_VEHICLE_SPEED_RPT_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->vehicle_speed = parsed_rpt.VEHICLE_SPEED_phys;
  new_msg->vehicle_speed_valid = parsed_rpt.VEHICLE_SPEED_VALID;

  return new_msg;
}

std::shared_ptr<void> Dbc13Api::ParseVinRpt(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::VinRpt>();

  VIN_RPT_t parsed_rpt;
  Unpack_VIN_RPT_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->mfg_code = parsed_rpt.VEH_MFG_CODE;
  new_msg->my_code = parsed_rpt.VEH_MY_CODE;
  new_msg->serial = parsed_rpt.VEH_SERIAL;

  return new_msg;
}

std::shared_ptr<void> Dbc13Api::ParseWheelSpeedRpt(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::WheelSpeedRpt>();

  WHEEL_SPEED_RPT_t parsed_rpt;
  Unpack_WHEEL_SPEED_RPT_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->axle_1_left_wheel_speed = parsed_rpt.WHEEL_SPD_AXLE_1_LEFT_phys;
  new_msg->axle_1_right_wheel_speed = parsed_rpt.WHEEL_SPD_AXLE_1_RIGHT_phys;
  new_msg->axle_2_left_wheel_speed = parsed_rpt.WHEEL_SPD_AXLE_2_LEFT_phys;
  new_msg->axle_2_right_wheel_speed = parsed_rpt.WHEEL_SPD_AXLE_2_RIGHT_phys;

  return new_msg;
}

std::shared_ptr<void> Dbc13Api::ParseWiperAuxRpt(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::WiperAuxRpt>();

  WIPER_AUX_RPT_t parsed_rpt;
  Unpack_WIPER_AUX_RPT_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->front_wiping = parsed_rpt.FRONT_WIPING;
  new_msg->front_spraying = parsed_rpt.FRONT_SPRAYING;
  new_msg->rear_wiping = parsed_rpt.REAR_WIPING;
  new_msg->rear_spraying = parsed_rpt.REAR_SPRAYING;
  new_msg->spray_near_empty = parsed_rpt.SPRAY_NEAR_EMPTY;
  new_msg->spray_empty = parsed_rpt.SPRAY_EMPTY;

  new_msg->front_wiping_avail = parsed_rpt.FRONT_WIPING_AVAIL;
  new_msg->front_spraying_avail = parsed_rpt.FRONT_SPRAYING_AVAIL;
  new_msg->rear_wiping_avail = parsed_rpt.REAR_WIPING_AVAIL;
  new_msg->rear_spraying_avail = parsed_rpt.REAR_SPRAYING_AVAIL;
  new_msg->spray_near_empty_avail = parsed_rpt.SPRAY_NEAR_EMPTY_AVAIL;
  new_msg->spray_empty_avail = parsed_rpt.SPRAY_EMPTY_AVAIL;

  return new_msg;
}

std::shared_ptr<void> Dbc13Api::ParseYawRateRpt(const cn_msgs::Frame& can_msg)
{
  auto new_msg = std::make_shared<pm_msgs::YawRateRpt>();

  YAW_RATE_RPT_t parsed_rpt;
  Unpack_YAW_RATE_RPT_pacmod13(&parsed_rpt, static_cast<const uint8_t*>(&can_msg.data[0]), static_cast<uint8_t>(can_msg.dlc));

  new_msg->yaw_rate = parsed_rpt.YAW_RATE_phys;

  return new_msg;
}

// Message Encoding
cn_msgs::Frame Dbc13Api::EncodeCmd(const pm_msgs::BrakeDecelCmd& msg)
{
  cn_msgs::Frame packed_frame;

  BRAKE_DECEL_CMD_t unpacked_cmd;
  unpacked_cmd.ENABLE = msg.enable;
  unpacked_cmd.IGNORE_OVERRIDES = msg.ignore_overrides;
  unpacked_cmd.CLEAR_OVERRIDE = msg.clear_override;
  unpacked_cmd.BRAKE_DECEL_CMD_phys = msg.brake_decel_command;
  unpacked_cmd.XBR_EBI_MODE = msg.xbr_ebi_mode;
  unpacked_cmd.XBR_PRIORITY = msg.xbr_priority;
  unpacked_cmd.XBR_CONTROL_MODE = msg.xbr_control_mode;

  uint8_t unused_ide;
  Pack_BRAKE_DECEL_CMD_pacmod13(&unpacked_cmd, static_cast<uint8_t*>(&packed_frame.data[0]), static_cast<uint8_t*>(&packed_frame.dlc), &unused_ide);

  return packed_frame;
}

cn_msgs::Frame Dbc13Api::EncodeCmd(const pm_msgs::CabinClimateCmd& msg)
{
  cn_msgs::Frame packed_frame;

  CABIN_CLIMATE_CMD_t unpacked_cmd;
  unpacked_cmd.ENABLE = msg.enable;
  unpacked_cmd.IGNORE_OVERRIDES = msg.ignore_overrides;
  unpacked_cmd.CLEAR_OVERRIDE = msg.clear_override;
  unpacked_cmd.CMD_AC_OFF_ON = msg.cmd_ac_off_on;
  unpacked_cmd.CMD_MAX_AC_OFF_ON = msg.cmd_max_ac_off_on;
  unpacked_cmd.CMD_DEFROST_OFF_ON = msg.cmd_defrost_off_on;
  unpacked_cmd.CMD_MAX_DEFROST_OFF_ON = msg.cmd_max_defrost_off_on;
  unpacked_cmd.CMD_DIR_UP_OFF_ON = msg.cmd_dir_up_off_on;
  unpacked_cmd.CMD_DIR_DOWN_OFF_ON = msg.cmd_dir_down_off_on;

  uint8_t unused_ide;
  Pack_CABIN_CLIMATE_CMD_pacmod13(&unpacked_cmd, static_cast<uint8_t*>(&packed_frame.data[0]), static_cast<uint8_t*>(&packed_frame.dlc), &unused_ide);

  return packed_frame;
}

cn_msgs::Frame Dbc13Api::EncodeCmd(const pm_msgs::DifferentialLocksCmd& msg)
{
  cn_msgs::Frame packed_frame;

  DIFFERENTIAL_LOCKS_CMD_t unpacked_cmd;
  unpacked_cmd.ENABLE = msg.enable;
  unpacked_cmd.CLEAR_OVERRIDE = msg.clear_override;
  unpacked_cmd.IGNORE_OVERRIDES = msg.ignore_overrides;
  unpacked_cmd.FRONT_LOCK_CMD = msg.front_lock_cmd;
  unpacked_cmd.CENTER_LOCK_CMD = msg.center_lock_cmd;
  unpacked_cmd.REAR_LOCK_CMD = msg.rear_lock_cmd;

  uint8_t unused_ide;
  Pack_DIFFERENTIAL_LOCKS_CMD_pacmod13(&unpacked_cmd, static_cast<uint8_t*>(&packed_frame.data[0]), static_cast<uint8_t*>(&packed_frame.dlc), &unused_ide);

  return packed_frame;
}

cn_msgs::Frame Dbc13Api::EncodeCmd(const pm_msgs::EngineBrakeCmd& msg)
{
  cn_msgs::Frame packed_frame;

  ENGINE_BRAKE_CMD_t unpacked_cmd;
  unpacked_cmd.ENABLE = msg.enable;
  unpacked_cmd.CLEAR_OVERRIDE = msg.clear_override;
  unpacked_cmd.IGNORE_OVERRIDES = msg.ignore_overrides;
  unpacked_cmd.ENGINE_BRAKE_CMD = msg.engine_brake_cmd;
  unpacked_cmd.AUTO_CMD = msg.auto_cmd;

  uint8_t unused_ide;
  Pack_ENGINE_BRAKE_CMD_pacmod13(&unpacked_cmd, static_cast<uint8_t*>(&packed_frame.data[0]), static_cast<uint8_t*>(&packed_frame.dlc), &unused_ide);

  return packed_frame;
}

cn_msgs::Frame Dbc13Api::EncodeCmd(const pm_msgs::GlobalCmd& msg)
{
  cn_msgs::Frame packed_frame;

  GLOBAL_CMD_t unpacked_cmd;
  unpacked_cmd.CLEAR_FAULTS = msg.clear_faults;
  unpacked_cmd.SANITY_CHECK_REQUIRED = msg.sanity_check_required;
  unpacked_cmd.CLEAR_OVERRIDES = msg.clear_overrides;
  unpacked_cmd.DEVELOPMENT_MODE_REQUEST = msg.development_mode_request;
  unpacked_cmd.COUNTER = msg.counter;
  unpacked_cmd.COMPLEMENT = msg.complement;

  uint8_t unused_ide;
  Pack_GLOBAL_CMD_pacmod13(&unpacked_cmd, static_cast<uint8_t*>(&packed_frame.data[0]), static_cast<uint8_t*>(&packed_frame.dlc), &unused_ide);

  return packed_frame;
}

cn_msgs::Frame Dbc13Api::EncodeCmd(const pm_msgs::NotificationCmd& msg)
{
  cn_msgs::Frame packed_frame;

  NOTIFICATION_CMD_t unpacked_cmd;
  unpacked_cmd.BUZZER_MUTE = msg.buzzer_mute;
  unpacked_cmd.UNDERDASH_LIGHTS_WHITE = msg.underdash_lights_white;

  uint8_t unused_ide;
  Pack_NOTIFICATION_CMD_pacmod13(&unpacked_cmd, static_cast<uint8_t*>(&packed_frame.data[0]), static_cast<uint8_t*>(&packed_frame.dlc), &unused_ide);

  return packed_frame;
}

cn_msgs::Frame Dbc13Api::EncodeCmd(const pm_msgs::SafetyBrakeCmd& msg)
{
  cn_msgs::Frame packed_frame;

  SAFETY_BRAKE_CMD_t unpacked_cmd;
  unpacked_cmd.SAFETY_BRAKE_CMD = msg.safety_brake_cmd;

  uint8_t unused_ide;
  Pack_SAFETY_BRAKE_CMD_pacmod13(&unpacked_cmd, static_cast<uint8_t*>(&packed_frame.data[0]), static_cast<uint8_t*>(&packed_frame.dlc), &unused_ide);

  return packed_frame;
}

cn_msgs::Frame Dbc13Api::EncodeCmd(const pm_msgs::SafetyFuncCmd& msg)
{
  cn_msgs::Frame packed_frame;

  SAFETY_FUNC_CMD_t unpacked_cmd;
  unpacked_cmd.COMMAND = msg.command;

  uint8_t unused_ide;
  Pack_SAFETY_FUNC_CMD_pacmod13(&unpacked_cmd, static_cast<uint8_t*>(&packed_frame.data[0]), static_cast<uint8_t*>(&packed_frame.dlc), &unused_ide);

  return packed_frame;
}

cn_msgs::Frame Dbc13Api::EncodeCmd(const pm_msgs::SteeringCmd& msg)
{
  cn_msgs::Frame packed_frame;

  STEERING_CMD_t unpacked_cmd;
  unpacked_cmd.ENABLE = msg.enable;
  unpacked_cmd.IGNORE_OVERRIDES = msg.ignore_overrides;
  unpacked_cmd.CLEAR_OVERRIDE = msg.clear_override;
  unpacked_cmd.POSITION_phys = msg.command;
  unpacked_cmd.ROTATION_RATE_phys = msg.rotation_rate;

  uint8_t unused_ide;
  Pack_STEERING_CMD_pacmod13(&unpacked_cmd, static_cast<uint8_t*>(&packed_frame.data[0]), static_cast<uint8_t*>(&packed_frame.dlc), &unused_ide);

  return packed_frame;
}

cn_msgs::Frame Dbc13Api::EncodeCmd(const pm_msgs::SystemCmdBool& msg)
{
  cn_msgs::Frame packed_frame;

  HORN_CMD_t unpacked_cmd;
  unpacked_cmd.ENABLE = msg.enable;
  unpacked_cmd.IGNORE_OVERRIDES = msg.ignore_overrides;
  unpacked_cmd.CLEAR_OVERRIDE = msg.clear_override;
  unpacked_cmd.HORN_CMD = msg.command;

  uint8_t unused_ide;
  Pack_HORN_CMD_pacmod13(&unpacked_cmd, static_cast<uint8_t*>(&packed_frame.data[0]), static_cast<uint8_t*>(&packed_frame.dlc), &unused_ide);

  return packed_frame;
}

cn_msgs::Frame Dbc13Api::EncodeCmd(const pm_msgs::SystemCmdFloat& msg)
{
  cn_msgs::Frame packed_frame;

  ACCEL_CMD_t unpacked_cmd;
  unpacked_cmd.ENABLE = msg.enable;
  unpacked_cmd.IGNORE_OVERRIDES = msg.ignore_overrides;
  unpacked_cmd.CLEAR_OVERRIDE = msg.clear_override;
  unpacked_cmd.ACCEL_CMD_phys = msg.command;

  uint8_t unused_ide;
  Pack_ACCEL_CMD_pacmod13(&unpacked_cmd, static_cast<uint8_t*>(&packed_frame.data[0]), static_cast<uint8_t*>(&packed_frame.dlc), &unused_ide);

  return packed_frame;
}

cn_msgs::Frame Dbc13Api::EncodeCmd(const pm_msgs::SystemCmdInt& msg)
{
  cn_msgs::Frame packed_frame;

  SHIFT_CMD_t unpacked_cmd;
  unpacked_cmd.ENABLE = msg.enable;
  unpacked_cmd.IGNORE_OVERRIDES = msg.ignore_overrides;
  unpacked_cmd.CLEAR_OVERRIDE = msg.clear_override;
  unpacked_cmd.SHIFT_CMD = msg.command;

  uint8_t unused_ide;
  Pack_SHIFT_CMD_pacmod13(&unpacked_cmd, static_cast<uint8_t*>(&packed_frame.data[0]), static_cast<uint8_t*>(&packed_frame.dlc), &unused_ide);

  return packed_frame;
}

cn_msgs::Frame Dbc13Api::EncodeCmd(const pm_msgs::UserPcHealthRpt& msg)
{
  cn_msgs::Frame packed_frame;

  USER_PC_HEALTH_RPT_00_t unpacked_cmd;
  unpacked_cmd.COUNTER = msg.counter;
  unpacked_cmd.CHECKSUM = msg.checksum;
  unpacked_cmd.USER_PC_APPLICATION_FAULT = msg.user_pc_application_fault;

  uint8_t unused_ide;
  Pack_USER_PC_HEALTH_RPT_00_pacmod13(&unpacked_cmd, static_cast<uint8_t*>(&packed_frame.data[0]), static_cast<uint8_t*>(&packed_frame.dlc), &unused_ide);

  return packed_frame;
}
}  // namespace pacmod4_common

