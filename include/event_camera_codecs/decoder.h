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

#ifndef EVENT_CAMERA_CODECS__DECODER_H_
#define EVENT_CAMERA_CODECS__DECODER_H_

#include <event_camera_codecs/noop_event_processor.h>
#include <event_camera_codecs/ros1_ros2_compat.h>
#include <stdint.h>

#include <memory>
#include <string>

namespace event_camera_codecs
{
template <class MsgT, class EventProcT = NoOpEventProcessor>
class Decoder
{
public:
  virtual ~Decoder() {}

  // Decodes entire message, produces callbacks to processor
  void decode(const MsgT & msg, EventProcT * processor)
  {
    setTimeBase(msg.time_base);
    decode(msg.events.data(), msg.events.size(), processor);
  }

  // Decodes messages up to, but not including, timeLimit. Will return true if
  // the time limit has been reached, false otherwise. If the time limit has been
  // reached, decode() must be called again with the same message pointer
  // to process the remaining events in the message.
  // Only if the time limit has been reached (!), nextTime will be set to the time
  // following the last decoded event.
  bool decodeUntil(
    const MsgT & msg, EventProcT * processor, uint64_t timeLimit, uint64_t * nextTime)
  {
    setTimeBase(msg.time_base);
    size_t bytesConsumed = decodeUntil(
      msg.events.data() + bytesUsed_, msg.events.size() - bytesUsed_, processor, timeLimit,
      nextTime);
    bytesUsed_ += bytesConsumed;
    const bool reachedTimeLimit = (bytesUsed_ < msg.events.size());
    if (reachedTimeLimit) {
      bytesUsed_ = 0;  // reached end-of-message, reset pointer for next message
    }
    return (reachedTimeLimit);
  }

  // Summarizes message statistics
  bool summarize(const MsgT & msg, uint64_t * firstTS, uint64_t * lastTS, size_t * numEventsOnOff)
  {
    return (summarize(msg.events.data(), msg.events.size(), firstTS, lastTS, numEventsOnOff));
  }

  // ---- interface methods
  // (deprecated version of typed message)
  virtual void decode(const uint8_t * buf, size_t bufSize, EventProcT * processor) = 0;
  // (deprecated version of typed message), returns bytes consumed
  virtual size_t decodeUntil(
    const uint8_t * buf, size_t bufSize, EventProcT * processor, uint64_t timeLimit,
    uint64_t * nextTime) = 0;
  virtual bool summarize(
    const uint8_t * buf, size_t size, uint64_t * firstTS, uint64_t * lastTS,
    size_t * numEventsOnOff) = 0;
  virtual void setTimeBase(const uint64_t timeBase) = 0;
  virtual bool findFirstSensorTime(const uint8_t * buf, size_t size, uint64_t * firstTS) = 0;
  virtual bool findFirstSensorTime(const MsgT & msg, uint64_t * firstTS) = 0;
  virtual void setTimeMultiplier(uint32_t mult) = 0;
  virtual void setGeometry(uint16_t width, uint16_t height) = 0;

  // ----------- variables
  size_t bytesUsed_{0};
};

}  // namespace event_camera_codecs
#endif  // EVENT_CAMERA_CODECS__DECODER_H_
