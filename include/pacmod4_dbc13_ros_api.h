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

#ifndef PACMOD4_DBC3_ROS_API_H
#define PACMOD4_DBC3_ROS_API_H

#include "pacmod4_dbc_ros_api.h"

#include <string>
#include <vector>
#include <memory>
#include <mutex>

namespace pacmod4_common
{

class Dbc13Api : public DbcApi
{
public:
  explicit Dbc13Api(uint32_t version = 13):DbcApi(version){};
  virtual ~Dbc13Api() = default;

  std::shared_ptr<void> ParseAccelAuxRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseAirPressureRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseAngVelRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseAutomsManSwitchRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseBatteryVoltageLevelRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseBrakeAuxRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseBrakeDecelAuxRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseCabinClimateRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseCmdLimitRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseComponentRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseDateTimeRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseDifferentialLocksRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseDetectedObjectRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseDoorRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseDrivetrainFeatureRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseEStopRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseEngineAuxRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseEngineAuxRpt2(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseEngineLoadFactorRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseEngineBrakeAuxRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseEngineBrakeRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseFireSuppressionRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseGlobalRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseGlobalRpt2(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseGnssTime(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseHeadlightAuxRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseInteriorLightsRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseLatLonHeadingRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseLinearAccelRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseMotorRpt1(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseMotorRpt2(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseMotorRpt3(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseNotificationRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseOccupancyRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseParkingBrakeAuxRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseSystemRptBoolWithControlStatus(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseRearLightsRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseRemoteStopRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseSafetyFuncCriticalStopRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseSafetyFuncRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseSafetyFuncRpt2(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseSafetyResponseRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseShiftAuxRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseSoftwareVersionRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseSteeringAuxRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseSteeringCmdLimitRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseSteeringRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseSupervisoryCtrl(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseSystemRptBool(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseSystemRptFloat(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseSystemRptInt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseSystemRptIntWithControlStatus(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseTipperBodyAuxRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseTirePressureExtendedRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseTirePressureRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseTireTemperatureExtendedRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseTrailerBrakePressureRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseTrailerFaultRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseTrailerPayloadRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseTurnAuxRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseVehicleDynamicsRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseVehicleFaultRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseVehicleFaultRpt2(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseVehicleSpeedRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseVinRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseWheelSpeedRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseWiperAuxRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseYawRateRpt(const cn_msgs::Frame& can_msg) override;

  cn_msgs::Frame EncodeCmd(const pm_msgs::BrakeDecelCmd& msg) override;
  cn_msgs::Frame EncodeCmd(const pm_msgs::CabinClimateCmd& msg) override;
  cn_msgs::Frame EncodeCmd(const pm_msgs::DifferentialLocksCmd& msg) override;
  cn_msgs::Frame EncodeCmd(const pm_msgs::EngineBrakeCmd& msg) override;
  cn_msgs::Frame EncodeCmd(const pm_msgs::GlobalCmd& msg) override;
  cn_msgs::Frame EncodeCmd(const pm_msgs::NotificationCmd& msg) override;
  cn_msgs::Frame EncodeCmd(const pm_msgs::SteeringCmd& msg) override;
  cn_msgs::Frame EncodeCmd(const pm_msgs::SafetyBrakeCmd& msg) override;
  cn_msgs::Frame EncodeCmd(const pm_msgs::SafetyFuncCmd& msg) override;
  cn_msgs::Frame EncodeCmd(const pm_msgs::SystemCmdBool& msg) override;
  cn_msgs::Frame EncodeCmd(const pm_msgs::SystemCmdFloat& msg) override;
  cn_msgs::Frame EncodeCmd(const pm_msgs::SystemCmdInt& msg) override;
  cn_msgs::Frame EncodeCmd(const pm_msgs::UserPcHealthRpt& msg) override;
};
}  // namespace pacmod4_common

#endif  // PACMOD4_DBC3_ROS_API_H
