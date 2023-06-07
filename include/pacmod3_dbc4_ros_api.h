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

#ifndef PACMOD3_DBC4_ROS_API_H
#define PACMOD3_DBC4_ROS_API_H

#include "pacmod3_dbc3_ros_api.h"

#include <string>
#include <vector>
#include <memory>
#include <mutex>

namespace pacmod3_common
{

// Derived from previous DBC API version
// The only overridden functions that exist here are due to changes to those msg types relative to the previous DBC version.
class Dbc4Api : public Dbc3Api
{
public:
  explicit Dbc4Api(uint32_t version = 4):Dbc3Api(version){};
  virtual ~Dbc4Api() = default;

  std::shared_ptr<void> ParseAngVelRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseComponentRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseLinearAccelRpt(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseSystemRptBool(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseSystemRptFloat(const cn_msgs::Frame& can_msg) override;
  std::shared_ptr<void> ParseSystemRptInt(const cn_msgs::Frame& can_msg) override;

  cn_msgs::Frame EncodeCmd(const pm_msgs::GlobalCmd& msg) override;
  cn_msgs::Frame EncodeCmd(const pm_msgs::NotificationCmd& msg) override;
  cn_msgs::Frame EncodeCmd(const pm_msgs::SystemCmdBool& msg) override;
  cn_msgs::Frame EncodeCmd(const pm_msgs::SystemCmdFloat& msg) override;
  cn_msgs::Frame EncodeCmd(const pm_msgs::SystemCmdInt& msg) override;
  cn_msgs::Frame EncodeCmd(const pm_msgs::SteeringCmd& msg) override;
};
}  // namespace pacmod3_common

#endif  // PACMOD3_DBC4_ROS_API_H
