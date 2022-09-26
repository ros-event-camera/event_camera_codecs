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

#ifndef EVENT_ARRAY_CODECS__EVT3_DECODER_H_
#define EVENT_ARRAY_CODECS__EVT3_DECODER_H_

#include <stdint.h>

#include <memory>
#include <string>

#include "event_array_codecs/decoder.h"
#include "event_array_codecs/evt3_types.h"

namespace event_array_codecs
{
namespace evt3
{
class Decoder : public event_array_codecs::Decoder
{
public:
  using timestamp_t = uint64_t;

  void decode(const uint8_t * buf, size_t bufSize, EventProcessor * processor) override;
  bool summarize(
    const uint8_t * buf, size_t size, uint64_t * firstTS, uint64_t * lastTS,
    size_t * numEventsOnOff) override;
  void setTimeBase(uint64_t) override
  {
    // no-op
  }
  bool findFirstSensorTime(const uint8_t * buf, size_t size, uint64_t * firstTS) override;

private:
  inline static timestamp_t update_high_time(uint16_t t, timestamp_t timeHigh)
  {
    // shift right and remove rollover bits to get last time_high
    const timestamp_t lastHigh = (timeHigh >> 12) & ((1ULL << 12) - 1);
    // sometimes the high stamp goes back a little without a rollover really happening
    const timestamp_t MIN_DISTANCE = 10;
    if (t < lastHigh && lastHigh - t > MIN_DISTANCE) {
      // new high time is smaller than old high time, bump high time bits
      // printf("%x %lx %lx\n", t, lastHigh, lastHigh - t);
      // std::cout << "rollover detected: new " << t << " old: " << lastHigh << std::endl;
      timeHigh += (1 << 24);  // add to rollover bits
    } else if (t < lastHigh) {
      // std::cout << "rollover averted: new " << t << " old: " << lastHigh << std::endl;
    }
    // wipe out lower 24 bits of timeHigh (leaving only rollover bits)
    // and replace non-rollover bits with new time base
    timeHigh = (timeHigh & (~((1ULL << 24) - 1))) | (static_cast<timestamp_t>(t) << 12);
    return (timeHigh);
  }

  inline size_t findValidTime(const Event * buffer, size_t numRead)
  {
    size_t i = 0;
    bool hasValidHighTime(false);
    for (; !hasValidTime_ && i < numRead; i++) {
      switch (buffer[i].code) {
        case Code::TIME_LOW: {
          const TimeLow * e = reinterpret_cast<const TimeLow *>(&buffer[i]);
          timeLow_ = e->t;
          if (hasValidHighTime) {
            hasValidTime_ = true;  // will break out of loop
            break;
          }
        } break;
        case Code::TIME_HIGH: {
          const TimeHigh * e = reinterpret_cast<const TimeHigh *>(&buffer[i]);
          timeHigh_ = update_high_time(e->t, timeHigh_);
          hasValidHighTime = true;
        } break;
        default:  // ignore all but time codes
          break;
      }
    }
    return (i);
  }

  // --------------------- variables
  size_t numEvents_{0};
  uint16_t ey_{0};              // current y coordinate
  uint16_t timeLow_{0};         // time stamp low
  timestamp_t timeHigh_{0};     // time stamp high + rollover bits
  uint8_t currentPolarity_{0};  // polarity for vector event
  uint16_t currentBaseX_{0};    // X coordinate basis for vector event
  bool hasValidTime_{false};
};
}  // namespace evt3
}  // namespace event_array_codecs
#endif  // EVENT_ARRAY_CODECS__EVT3_DECODER_H_
