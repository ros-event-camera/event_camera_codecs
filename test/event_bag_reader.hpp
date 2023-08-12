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

#ifndef EVENT_BAG_READER_HPP_
#define EVENT_BAG_READER_HPP_

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>

class EventBagReader
{
public:
  using EventPacket = event_camera_msgs::msg::EventPacket;
  explicit EventBagReader(const std::string & bagName) { reader_.open(bagName); }
  EventPacket::ConstSharedPtr next()
  {
    if (reader_.has_next()) {
      auto msg = reader_.read_next();
      rclcpp::SerializedMessage serializedMsg(*msg->serialized_data);
      auto m = std::make_shared<EventPacket>();
      serialization_.deserialize_message(&serializedMsg, m.get());
      return (m);
    }
    return (nullptr);
  }

private:
  rosbag2_cpp::Reader reader_;
  rclcpp::Serialization<EventPacket> serialization_;
};

#endif  // EVENT_BAG_READER_HPP_
