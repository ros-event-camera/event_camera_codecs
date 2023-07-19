// -*-c++-*--------------------------------------------------------------------
// Copyright 2023 Bernd Pfrommer <bernd.pfrommer@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef EVENT_CAMERA_CODECS__EVENT_PACKET_H_
#define EVENT_CAMERA_CODECS__EVENT_PACKET_H_

#include <event_camera_codecs/ros1_ros2_compat.h>

#ifdef USING_ROS_1

#include <event_camera_msgs/EventPacket.h>
namespace event_camera_codecs
{
using EventPacket = event_camera_msgs::EventPacket;
using EventPacketConstSharedPtr = event_camera_msgs::EventPacket::MSG_CONST_SHARED_PTR;
}  // namespace event_camera_codecs

#else

#include <event_camera_msgs/msg/event_packet.hpp>
namespace event_camera_codecs
{
using EventPacket = event_camera_msgs::msg::EventPacket;
using EventPacketConstSharedPtr = event_camera_msgs::msg::EventPacket::MSG_CONST_SHARED_PTR;
}  // namespace event_camera_codecs

#endif

#endif  // EVENT_CAMERA_CODECS__EVENT_PACKET_H_
