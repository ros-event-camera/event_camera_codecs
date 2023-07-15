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

#ifndef EVENT_CAMERA_CODECS__TRIGGER_DECODER_H_
#define EVENT_CAMERA_CODECS__TRIGGER_DECODER_H_

#include <stdint.h>

#include <memory>
#include <string>

#include "event_camera_codecs/decoder.h"
#include "event_camera_codecs/event_processor.h"

namespace event_camera_codecs
{
namespace trigger
{
template <class MsgT, class EventProcT>
class Decoder : public event_camera_codecs::Decoder<MsgT, EventProcT>
{
public:
  using timestamp_t = uint64_t;

  void decode(const uint8_t * buf, size_t bufSize, EventProcT * processor) override
  {
    for (const uint8_t * p_u8 = buf; p_u8 < buf + bufSize; p_u8 += 8) {
      const uint64_t & p = *reinterpret_cast<const uint64_t *>(p_u8);
      processor->eventExtTrigger(
        timeBase_ + static_cast<uint32_t>(p & 0xFFFFFFFFULL),
        static_cast<bool>(p & ~0x7FFFFFFFFFFFFFFFULL), static_cast<uint8_t>((p >> 48) & 0x7FFFULL));
    }
    processor->finished();
  }

  size_t decodeUntil(
    const uint8_t * buf, size_t bufSize, EventProcT * processor, uint64_t timeLimit,
    uint64_t * nextTime) override
  {
    for (const uint8_t * p_u8 = buf; p_u8 < buf + bufSize; p_u8 += 8) {
      const uint64_t & p = *reinterpret_cast<const uint64_t *>(p_u8);
      const uint64_t t = timeBase_ + static_cast<uint32_t>(p & 0xFFFFFFFFULL);
      if (t >= timeLimit) {
        processor->finished();
        *nextTime = t;
        return (p_u8 - buf);
      }
      processor->eventExtTrigger(
        t, static_cast<bool>(p & ~0x7FFFFFFFFFFFFFFFULL),
        static_cast<uint8_t>((p >> 48) & 0x7FFFULL));
    }
    processor->finished();
    return (bufSize);
  }

  bool summarize(
    const uint8_t * buf, size_t size, uint64_t * firstTS, uint64_t * lastTS,
    size_t * numEventsOnOff) override
  {
    bool hasValidTime(false);
    if (size >= 8) {
      *firstTS = ((*reinterpret_cast<const uint64_t *>(buf)) & 0xFFFFFFFFULL) + timeBase_;
      *lastTS = ((*reinterpret_cast<const uint64_t *>(buf + size - 8)) & 0xFFFFFFFFULL) + timeBase_;
      hasValidTime = true;
    }
    if (numEventsOnOff) {
      for (const uint8_t * p_u8 = buf; p_u8 < buf + size; p_u8 += 8) {
        const uint64_t & p = *reinterpret_cast<const uint64_t *>(p_u8);
        numEventsOnOff[(p & ~0x7FFFFFFFFFFFFFFFULL) != 0]++;
      }
    }
    return (hasValidTime);
  }

  void setTimeBase(uint64_t t) override { timeBase_ = t; }

  bool findFirstSensorTime(const uint8_t * buf, size_t size, uint64_t * firstTS) override
  {
    if (size >= 8) {
      *firstTS = ((*reinterpret_cast<const uint64_t *>(buf)) & 0xFFFFFFFFULL) + timeBase_;
      return (true);
    }
    return (false);
  }
  void setTimeMultiplier(uint32_t) override
  { /* ignored */
  }
  void setGeometry(uint16_t, uint16_t) override
  { /* ignored */
  }

private:
  // --------------------- variables
  uint64_t timeBase_{0};
};
}  // namespace trigger
}  // namespace event_camera_codecs
#endif  // EVENT_CAMERA_CODECS__TRIGGER_DECODER_H_
