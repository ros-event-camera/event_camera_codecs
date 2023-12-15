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

#ifndef EVENT_CAMERA_CODECS__LIBCAER_CMP_DECODER_H_
#define EVENT_CAMERA_CODECS__LIBCAER_CMP_DECODER_H_

#include <event_camera_codecs/decoder.h>
#include <event_camera_codecs/libcaer_cmp_types.h>
#include <stdint.h>

#include <bitset>
#include <limits>
#include <memory>
#include <string>

namespace event_camera_codecs
{
namespace libcaer_cmp
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
    if (timeBase_ >= timeLimit) {
      *nextTime = timeBase_;
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
    timeLow_ = 0;
    timeHigh_ = 0;
    for (size_t i = 0; i < numRead; i++) {
      switch (buffer[i].code) {
        case Code::ADDR_Y: {
          const AddrY * e = reinterpret_cast<const AddrY *>(&buffer[i]);
          if (ex_ < width_ && e->y < height_) {
            processor->eventCD(makeTime(timeHigh_, timeLow_), ex_, e->y, e->polarity);
            numEvents_++;
          }
        } break;
        case Code::ADDR_X: {
          const AddrX * e = reinterpret_cast<const AddrX *>(&buffer[i]);
          ex_ = e->x;  // save for later
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
        case Code::VECT_BASE_Y: {
          const VectBaseY * b = reinterpret_cast<const VectBaseY *>(&buffer[i]);
          currentPolarity_ = b->polarity;
          currentBaseY_ = b->y;
          break;
        }
        case Code::VECT_8: {
          const Vect8 * b = reinterpret_cast<const Vect8 *>(&buffer[i]);
          for (int i = 0; i < 8; i++) {
            if (b->valid & (1 << i)) {
              const uint16_t ey = currentBaseY_ + i;
              if (ex_ < width_ && ey < height_) {
                processor->eventCD(makeTime(timeHigh_, timeLow_), ex_, ey, currentPolarity_);
                numEvents_++;
              }
            }
          }
          currentBaseY_ += 8;  // TODO(Bernd): use this optimization on the encoder side!
          break;
        }
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
    timestamp_t timeHigh{0};
    uint16_t timeLow{0};
    uint8_t currentPolarity{0};

    const size_t numRead = bufSize / sizeof(Event);
    const Event * buffer = reinterpret_cast<const Event *>(buf);
    uint64_t t1 = timeBase_;
    uint64_t t2 = t1;

    for (size_t i = 0; i < numRead; i++) {
      switch (buffer[i].code) {
        case Code::ADDR_Y: {
          const AddrY * e = reinterpret_cast<const AddrY *>(&buffer[i]);
          (numEventsOnOff[e->polarity])++;
        } break;
        case Code::TIME_LOW: {
          const TimeLow * e = reinterpret_cast<const TimeLow *>(&buffer[i]);
          timeLow = e->t;
          const timestamp_t t = makeTime(timeHigh, timeLow);
          t1 = std::min(t, t1);
          t2 = std::max(t, t2);
        } break;
        case Code::TIME_HIGH: {
          const TimeHigh * e = reinterpret_cast<const TimeHigh *>(&buffer[i]);
          timeHigh = update_high_time(e->t, timeHigh);
        } break;
        case Code::VECT_BASE_Y: {
          const VectBaseY * b = reinterpret_cast<const VectBaseY *>(&buffer[i]);
          currentPolarity = b->polarity;
          break;
        }
        case Code::VECT_8: {
          const Vect8 * b = reinterpret_cast<const Vect8 *>(&buffer[i]);
          numEventsOnOff[currentPolarity] += std::bitset<8>(b->valid).count();
          break;
        }
        case Code::ADDR_X:
          break;
        default:
          throw std::runtime_error("got unsupported libcaer_cmp code!");
          break;
      }
    }
    *firstTS = t1;
    *lastTS = t2;
    return (true);
  }
  void setTimeBase(uint64_t t) override { timeBase_ = t; }

  bool findFirstSensorTime(const uint8_t *, size_t, uint64_t * firstTS) override
  {
    *firstTS = timeBase_;
    return (true);
  }

  bool findFirstSensorTime(const MsgT & msg, uint64_t * firstTS) override
  {
    setTimeBase(msg.time_base);
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
  inline timestamp_t makeTime(timestamp_t high, uint16_t low)
  {
    return (((high | low) * 1000 + timeBase_) * timeMult_);
  }

  inline static timestamp_t update_high_time(uint16_t t, timestamp_t)
  {
    return (static_cast<timestamp_t>(t) << 12);
  }

  // --------------------- variables
  size_t numEvents_{0};
  uint16_t ex_{0};              // current x coordinate
  uint16_t timeLow_{0};         // time stamp low
  timestamp_t timeHigh_{0};     // time stamp high + rollover bits
  uint8_t currentPolarity_{0};  // polarity for vector event
  uint16_t currentBaseY_{0};    // Y coordinate basis for vector event
  uint32_t timeMult_{1};        // default: time in nanoseconds
  uint64_t timeBase_{0};        // first sensor time in packet
  uint16_t width_{0};           // sensor geometry
  uint16_t height_{0};          // sensor geometry
};
}  // namespace libcaer_cmp
}  // namespace event_camera_codecs
#endif  // EVENT_CAMERA_CODECS__LIBCAER_CMP_DECODER_H_
