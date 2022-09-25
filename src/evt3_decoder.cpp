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

#include "event_array_codecs/evt3_decoder.h"

#include <bitset>

#include "event_array_codecs/event_processor.h"
#include "event_array_codecs/evt3_types.h"

namespace event_array_codecs
{
namespace evt3
{
using timestamp_t = Decoder::timestamp_t;

inline static timestamp_t make_time(timestamp_t high, uint16_t low) { return (high | low); }

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

// #define DEBUG

void Decoder::decode(const uint8_t * buf, size_t bufSize, EventProcessor * processor)
{
  const size_t numRead = bufSize / sizeof(Event);
  const Event * buffer = reinterpret_cast<const Event *>(buf);
  for (size_t i = 0; i < numRead; i++) {
    switch (buffer[i].code) {
      case Code::ADDR_X: {
        const AddrX * e = reinterpret_cast<const AddrX *>(&buffer[i]);
        processor->eventCD(timeHigh_ | timeLow_, e->x, ey_, e->polarity);
        numEvents_++;
      } break;
      case Code::ADDR_Y: {
        const AddrY * e = reinterpret_cast<const AddrY *>(&buffer[i]);
        ey_ = e->y;  // save for later
      } break;
      case Code::TIME_LOW: {
        const TimeLow * e = reinterpret_cast<const TimeLow *>(&buffer[i]);
        timeLow_ = e->t;
      } break;
      case Code::TIME_HIGH: {
        const TimeHigh * e = reinterpret_cast<const TimeHigh *>(&buffer[i]);
        timeHigh_ = update_high_time(e->t, timeHigh_);
        lastHigh_ = e->t;
      } break;
      case Code::VECT_BASE_X: {
        const VectBaseX * b = reinterpret_cast<const VectBaseX *>(&buffer[i]);
        currentPolarity_ = b->pol;
        currentBaseX_ = b->x;
        break;
      }
      case Code::VECT_8: {
        const Vect8 * b = reinterpret_cast<const Vect8 *>(&buffer[i]);
        for (int i = 0; i < 8; i++) {
          if (b->valid & (1 << i)) {
            processor->eventCD(
              make_time(timeHigh_, timeLow_), currentBaseX_ + i, ey_, currentPolarity_);
            numEvents_++;
          }
        }
        currentBaseX_ += 8;
        break;
      }
      case Code::VECT_12: {
        const Vect12 * b = reinterpret_cast<const Vect12 *>(&buffer[i]);
        for (int i = 0; i < 12; i++) {
          if (b->valid & (1 << i)) {
            processor->eventCD(
              make_time(timeHigh_, timeLow_), currentBaseX_ + i, ey_, currentPolarity_);
            numEvents_++;
          }
        }
        currentBaseX_ += 12;
        break;
      }
      case Code::EXT_TRIGGER: {
        const ExtTrigger * e = reinterpret_cast<const ExtTrigger *>(&buffer[i]);
        processor->eventExtTrigger(make_time(timeHigh_, timeLow_), e->edge, e->id);
        break;
      }
      case Code::OTHERS: {
#if 0
              const Others * e = reinterpret_cast<const Others *>(&buffer[i]);
              const SubType subtype = static_cast<SubType>(e->subtype);
              if (subtype != SubType::MASTER_END_OF_FRAME) {
                std::cout << "ignoring OTHERS code: " << toString(subtype) << std::endl;
              }
#endif
      } break;
        // ------- the CONTINUED codes are used in conjunction with
        // the OTHERS code, so ignore as well
      case Code::CONTINUED_4:
      case Code::CONTINUED_12: {
      } break;
      default:
        // ------- all the vector codes are not generated
        // by the Gen3 sensor I have....
        std::cout << "got unsupported code: " << static_cast<int>(buffer[i].code) << std::endl;
        throw std::runtime_error("got unsupported code!");
        break;
    }
  }
  processor->finished();
}

void Decoder::summarize(
  const uint8_t * buf, size_t bufSize, uint64_t * firstTS, uint64_t * lastTS,
  size_t * numEventsOnOff)
{
  const size_t numRead = bufSize / sizeof(Event);
  const Event * buffer = reinterpret_cast<const Event *>(buf);
  int64_t t1(-1), t2(-1);

  for (size_t i = 0; i < numRead; i++) {
    switch (buffer[i].code) {
      case Code::ADDR_X: {
        const AddrX * e = reinterpret_cast<const AddrX *>(&buffer[i]);
        (numEventsOnOff[e->polarity])++;
      } break;
      case Code::TIME_LOW: {
        const TimeLow * e = reinterpret_cast<const TimeLow *>(&buffer[i]);
        timeLow_ = e->t;
        if (timeHigh_ != 0) {
          const timestamp_t t = make_time(timeHigh_, timeLow_);
          if (t1 < 0) {
            t1 = t;
          }
          if (static_cast<int64_t>(t) > t2) {
            t2 = static_cast<int64_t>(t);
          }
        }
      } break;
      case Code::TIME_HIGH: {
        const TimeHigh * e = reinterpret_cast<const TimeHigh *>(&buffer[i]);
        timeHigh_ = update_high_time(e->t, timeHigh_);
      } break;
      case Code::VECT_BASE_X: {
        const VectBaseX * b = reinterpret_cast<const VectBaseX *>(&buffer[i]);
        currentPolarity_ = b->pol;
        break;
      }
      case Code::VECT_8: {
        const Vect8 * b = reinterpret_cast<const Vect8 *>(&buffer[i]);
        numEventsOnOff[currentPolarity_] += std::bitset<8>(b->valid).count();
        break;
      }
      case Code::VECT_12: {
        const Vect12 * b = reinterpret_cast<const Vect12 *>(&buffer[i]);
        numEventsOnOff[currentPolarity_] += std::bitset<12>(b->valid).count();
        break;
      }
      case Code::ADDR_Y:
      case Code::EXT_TRIGGER:
      case Code::OTHERS:
      case Code::CONTINUED_4:
      case Code::CONTINUED_12: {
      } break;
      default:
        // ------- all the vector codes are not generated
        // by the Gen3 sensor I have....
        std::cout << "got unsupported code: " << static_cast<int>(buffer[i].code) << std::endl;
        throw std::runtime_error("got unsupported code!");
        break;
    }
  }
  if (t1 > 0) {
    *firstTS = static_cast<uint64_t>(t1);
  }
  if (t2 > 0) {
    *lastTS = static_cast<uint64_t>(t2);
  }
}

bool Decoder::findFirstSensorTime(const uint8_t * buf, size_t size, uint64_t * firstTS)
{
  const size_t numRead = size / sizeof(Event);
  const Event * buffer = reinterpret_cast<const Event *>(buf);
  bool foundTime(false);
  for (size_t i = 0; i < numRead; i++) {
    switch (buffer[i].code) {
      case Code::TIME_LOW: {
        const TimeLow * e = reinterpret_cast<const TimeLow *>(&buffer[i]);
        timeLow_ = e->t;
        if (timeHigh_ != 0) {
          *firstTS = make_time(timeHigh_, timeLow_);
          foundTime = true;
          // cannot return early, need to update decoder state!
        }
      } break;
      case Code::TIME_HIGH: {
        const TimeHigh * e = reinterpret_cast<const TimeHigh *>(&buffer[i]);
        timeHigh_ = update_high_time(e->t, timeHigh_);
      } break;
      default:
        break;
    }
  }
  return (foundTime);
}

}  // namespace evt3
}  // namespace event_array_codecs
