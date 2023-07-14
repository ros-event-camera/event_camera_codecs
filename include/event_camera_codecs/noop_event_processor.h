// -*-c++-*--------------------------------------------------------------------
// Copyright 2022 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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

#ifndef EVENT_CAMERA_CODECS__NOOP_EVENT_PROCESSOR_H_
#define EVENT_CAMERA_CODECS__NOOP_EVENT_PROCESSOR_H_

#include <event_camera_codecs/event_processor.h>

#include <cstddef>
#include <cstdint>
namespace event_camera_codecs
{
class NoOpEventProcessor : public EventProcessor
{
public:
  void eventCD(uint64_t, uint16_t, uint16_t, uint8_t) override {}
  void eventExtTrigger(uint64_t, uint8_t, uint8_t) override {}
  void finished() override {}
  void rawData(const char *, size_t) override {}
};
}  // namespace event_camera_codecs
#endif  // EVENT_CAMERA_CODECS__NOOP_EVENT_PROCESSOR_H_
