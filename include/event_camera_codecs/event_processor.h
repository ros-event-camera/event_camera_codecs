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

#ifndef EVENT_CAMERA_CODECS__EVENT_PROCESSOR_H_
#define EVENT_CAMERA_CODECS__EVENT_PROCESSOR_H_

#include <cstddef>
#include <cstdint>

namespace event_camera_codecs
{
class EventProcessor
{
public:
  virtual ~EventProcessor() {}
  virtual void eventCD(uint64_t sensor_time, uint16_t ex, uint16_t ey, uint8_t polarity) = 0;
  virtual void eventExtTrigger(uint64_t sensor_time, uint8_t edge, uint8_t id) = 0;
  virtual void finished() = 0;
  virtual void rawData(const char * data, size_t len) = 0;
};
}  // namespace event_camera_codecs
#endif  // EVENT_CAMERA_CODECS__EVENT_PROCESSOR_H_
