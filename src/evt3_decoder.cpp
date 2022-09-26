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
#include <limits>

#include "event_array_codecs/event_processor.h"
#include "event_array_codecs/evt3_types.h"

namespace event_array_codecs
{
namespace evt3
{
using timestamp_t = Decoder::timestamp_t;

inline static timestamp_t make_time(timestamp_t high, uint16_t low)
{
  return ((high | low) * 1000);
}

void Decoder::decode(const uint8_t * buf, size_t bufSize, EventProcessor * processor)
{
  const size_t numRead = bufSize / sizeof(Event);
  const Event * buffer = reinterpret_cast<const Event *>(buf);
  for (size_t i = findValidTime(buffer, numRead); i < numRead; i++) {
    switch (buffer[i].code) {
      case Code::ADDR_X: {
        const AddrX * e = reinterpret_cast<const AddrX *>(&buffer[i]);
        processor->eventCD(make_time(timeHigh_, timeLow_), e->x, ey_, e->polarity);
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

bool Decoder::summarize(
  const uint8_t * buf, size_t bufSize, uint64_t * firstTS, uint64_t * lastTS,
  size_t * numEventsOnOff)
{
  const size_t numRead = bufSize / sizeof(Event);
  const Event * buffer = reinterpret_cast<const Event *>(buf);
  // find first valid time stamp
  size_t i = findValidTime(buffer, numRead);
  if (!hasValidTime_) {
    return (hasValidTime_);
  }
  uint64_t t1 = make_time(timeHigh_, timeLow_);
  uint64_t t2 = t1;

  for (; i < numRead; i++) {
    switch (buffer[i].code) {
      case Code::ADDR_X: {
        const AddrX * e = reinterpret_cast<const AddrX *>(&buffer[i]);
        (numEventsOnOff[e->polarity])++;
      } break;
      case Code::TIME_LOW: {
        const TimeLow * e = reinterpret_cast<const TimeLow *>(&buffer[i]);
        timeLow_ = e->t;
        const timestamp_t t = make_time(timeHigh_, timeLow_);
        t1 = std::min(t, t1);
        t2 = std::max(t, t2);
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
  *firstTS = t1;
  *lastTS = t2;
  return (hasValidTime_);
}

bool Decoder::findFirstSensorTime(const uint8_t * buf, size_t size, uint64_t * firstTS)
{
  const size_t numRead = size / sizeof(Event);
  const Event * buffer = reinterpret_cast<const Event *>(buf);
  size_t i = findValidTime(buffer, numRead);
  *firstTS = make_time(timeHigh_, timeLow_);
  // need to still run this loop to update the time state of the decoder
  // and capture rollover etc
  for (; i < numRead; i++) {
    switch (buffer[i].code) {
      case Code::TIME_LOW: {
        const TimeLow * e = reinterpret_cast<const TimeLow *>(&buffer[i]);
        timeLow_ = e->t;
      } break;
      case Code::TIME_HIGH: {
        const TimeHigh * e = reinterpret_cast<const TimeHigh *>(&buffer[i]);
        timeHigh_ = update_high_time(e->t, timeHigh_);
      } break;
      default:
        break;
    }
  }
  return (hasValidTime_);
}

}  // namespace evt3
}  // namespace event_array_codecs
