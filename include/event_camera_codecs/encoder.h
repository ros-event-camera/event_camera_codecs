// -*-c++-*--------------------------------------------------------------------
// Copyright 2021 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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

#ifndef EVENT_CAMERA_CODECS__ENCODER_H_
#define EVENT_CAMERA_CODECS__ENCODER_H_

#include <stdint.h>

#include <memory>
#include <string>
#include <vector>

namespace event_camera_codecs
{
class Encoder
{
public:
  virtual ~Encoder() {}
  // ---- interface methods
  virtual void setBuffer(std::vector<uint8_t> * buf) = 0;
  virtual void setSensorTime(uint64_t sensorTime) = 0;
  // time difference dt is in nanoseconds since last setSensorTime
  virtual void encodeCD(int32_t dt, uint16_t x, uint16_t y, uint8_t p) = 0;
  virtual void encodeExtTrigger(int32_t dt, uint8_t edge, uint8_t id) = 0;
  virtual void flush() = 0;
  // ----- static methods
  // factory method to create new instance
  static std::shared_ptr<Encoder> newInstance(const std::string & codec);
};

}  // namespace event_camera_codecs
#endif  // EVENT_CAMERA_CODECS__ENCODER_H_
