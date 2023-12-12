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

#ifndef EVENT_CAMERA_CODECS__EVT3_DECODER_H_
#define EVENT_CAMERA_CODECS__EVT3_DECODER_H_

#include <stdint.h>

#include <bitset>
#include <limits>
#include <memory>
#include <string>

#include "event_camera_codecs/decoder.h"
#include "event_camera_codecs/evt3_types.h"

namespace event_camera_codecs
{
namespace evt3
{
template <class MsgT, class EventProcT>
class Decoder : public event_camera_codecs::Decoder<MsgT, EventProcT>
{
public:
  using timestamp_t = uint64_t;

  void decode(const uint8_t * buf, size_t bufSize, EventProcT * processor) override
  {
    struct NoTimeLimit
    {
      static bool isInFuture(uint64_t, uint64_t) { return (false); }
    };
    doDecode<NoTimeLimit>(buf, bufSize, processor, 0, nullptr, nullptr);
  }

  size_t decodeUntil(
    const uint8_t * buf, size_t bufSize, EventProcT * processor, uint64_t timeLimit,
    uint64_t * nextTime) override
  {
    if (hasValidTime_ && makeTime(timeHigh_, timeLow_) >= timeLimit) {
      *nextTime = makeTime(timeHigh_, timeLow_);
      return (0);
    }
    struct TimeLimit
    {
      static bool isInFuture(uint64_t t, uint64_t limit) { return (t >= limit); }
    };
    size_t numConsumed{0};
    doDecode<TimeLimit>(buf, bufSize, processor, timeLimit, &numConsumed, nextTime);
    return (numConsumed);
  }

  template <class TimeLimiterT>
  void doDecode(
    const uint8_t * buf, size_t bufSize, EventProcT * processor, uint64_t timeLimit,
    size_t * numConsumed, uint64_t * nextTime)
  {
    const size_t numRead = bufSize / sizeof(Event);
    const Event * buffer = reinterpret_cast<const Event *>(buf);
    for (size_t i = findValidTime(buffer, numRead); i < numRead; i++) {
      switch (buffer[i].code) {
        case Code::ADDR_X: {
          const AddrX * e = reinterpret_cast<const AddrX *>(&buffer[i]);
          if (e->x < width_ && ey_ < height_) {
            processor->eventCD(makeTime(timeHigh_, timeLow_), e->x, ey_, e->polarity);
            numEvents_++;
          }
        } break;
        case Code::ADDR_Y: {
          const AddrY * e = reinterpret_cast<const AddrY *>(&buffer[i]);
          ey_ = e->y;  // save for later
        } break;
        case Code::TIME_LOW: {
          const TimeLow * e = reinterpret_cast<const TimeLow *>(&buffer[i]);
          timeLow_ = e->t;
          if (TimeLimiterT::isInFuture(makeTime(timeHigh_, timeLow_), timeLimit)) {
            // stopping early because we reached the time limit
            *numConsumed = i * sizeof(Event);
            *nextTime = makeTime(timeHigh_, timeLow_);
            processor->finished();
            return;
          }
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
              const uint16_t ex = currentBaseX_ + i;
              if (ex < width_ && ey_ < height_) {
                processor->eventCD(makeTime(timeHigh_, timeLow_), ex, ey_, currentPolarity_);
                numEvents_++;
              }
            }
          }
          currentBaseX_ += 8;
          break;
        }
        case Code::VECT_12: {
          const Vect12 * b = reinterpret_cast<const Vect12 *>(&buffer[i]);
          for (int i = 0; i < 12; i++) {
            if (b->valid & (1 << i)) {
              const uint16_t ex = currentBaseX_ + i;
              if (ex < width_ && ey_ < height_) {
                processor->eventCD(
                  makeTime(timeHigh_, timeLow_), currentBaseX_ + i, ey_, currentPolarity_);
                numEvents_++;
              }
            }
          }
          currentBaseX_ += 12;
          break;
        }
        case Code::EXT_TRIGGER: {
          const ExtTrigger * e = reinterpret_cast<const ExtTrigger *>(&buffer[i]);
          processor->eventExtTrigger(makeTime(timeHigh_, timeLow_), e->edge, e->id);
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
          std::cout << "evt3 event camera decoder got unsupported code: "
                    << static_cast<int>(buffer[i].code) << std::endl;
          throw std::runtime_error("got unsupported code!");
          break;
      }
    }
    if (numConsumed != nullptr) {
      *numConsumed = bufSize;  // have consumed the entire buffer
    }
    processor->finished();
  }

  bool summarize(
    const uint8_t * buf, size_t bufSize, uint64_t * firstTS, uint64_t * lastTS,
    size_t * numEventsOnOff) override
  {
    const size_t numRead = bufSize / sizeof(Event);
    const Event * buffer = reinterpret_cast<const Event *>(buf);
    // find first valid time stamp
    size_t i = findValidTime(buffer, numRead);
    if (!hasValidTime_) {
      return (hasValidTime_);
    }
    uint64_t t1 = makeTime(timeHigh_, timeLow_);
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
          const timestamp_t t = makeTime(timeHigh_, timeLow_);
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
          std::cout << "evt3 event camera decoder got unsupported code: "
                    << static_cast<int>(buffer[i].code) << std::endl;
          throw std::runtime_error("got unsupported code!");
          break;
      }
    }
    *firstTS = t1;
    *lastTS = t2;
    return (hasValidTime_);
  }
  void setTimeBase(uint64_t) override
  {
    // no-op
  }
  bool findFirstSensorTime(const uint8_t * buf, size_t size, uint64_t * firstTS) override
  {
    const size_t numRead = size / sizeof(Event);
    const Event * buffer = reinterpret_cast<const Event *>(buf);
    size_t i = findValidTime(buffer, numRead);
    *firstTS = makeTime(timeHigh_, timeLow_);
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

  bool findFirstSensorTime(const MsgT & msg, uint64_t * firstTS) override
  {
    return findFirstSensorTime(msg.events.data(), msg.events.size(), firstTS);
  }

  void setTimeMultiplier(uint32_t mult) override { timeMult_ = mult; }
  void setGeometry(uint16_t width, uint16_t height) override
  {
    width_ = width;
    height_ = height;
  }
  uint16_t getWidth() const override { return (width_); }
  uint16_t getHeight() const override { return (height_); }

private:
  inline timestamp_t makeTime(timestamp_t high, uint16_t low) { return ((high | low) * timeMult_); }

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
  uint32_t timeMult_{1000};     // default: time in nanoseconds
  bool hasValidTime_{false};    // false until time is valid
  uint16_t width_{0};           // sensor geometry
  uint16_t height_{0};          // sensor geometry
};
}  // namespace evt3
}  // namespace event_camera_codecs
#endif  // EVENT_CAMERA_CODECS__EVT3_DECODER_H_
