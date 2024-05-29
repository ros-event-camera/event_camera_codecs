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

#ifndef EVENT_CAMERA_CODECS__LIBCAER_DECODER_H_
#define EVENT_CAMERA_CODECS__LIBCAER_DECODER_H_

#include <endian.h>
#include <stdint.h>

#include <iostream>
#include <memory>
#include <string>

#include "event_camera_codecs/decoder.h"
#include "event_camera_codecs/event_processor.h"

namespace event_camera_codecs
{
namespace libcaer
{
template <class MsgT, class EventProcT>
class Decoder : public event_camera_codecs::Decoder<MsgT, EventProcT>
{
public:
  using timestamp_t = uint64_t;

  void decode(const uint8_t * buf, size_t bufSize, EventProcT * processor) override
  {
    struct Decode
    {
      explicit Decode(EventProcT * p) : proc(p) {}
      bool timeExceedsLimit(uint64_t) const { return (false); }
      void eventCD(uint64_t t, uint32_t x, uint32_t y, uint32_t polarity)
      {
        proc->eventCD(
          t, static_cast<uint16_t>(x), static_cast<uint16_t>(y), static_cast<uint8_t>(polarity));
      }
      void finished() { proc->finished(); }
      void setTime(uint64_t) {}  // do nothing
      void setSize(size_t) {}    // do nothing
      EventProcT * proc{nullptr};
      size_t bytesProcessed{0};
    } decoder(processor);
    loopOverTime<Decode>(buf, bufSize, decoder);
  }

  size_t decodeUntil(
    const uint8_t * buf, size_t bufSize, EventProcT * processor, uint64_t timeLimit,
    uint64_t * nextTime) override
  {
    struct DecodeUntil
    {
      explicit DecodeUntil(EventProcT * p, uint64_t tL) : proc(p), tLimit(tL) {}
      bool timeExceedsLimit(uint64_t t) const { return (t >= tLimit); }
      void eventCD(uint64_t t, uint32_t x, uint32_t y, uint32_t polarity)
      {
        proc->eventCD(
          t, static_cast<uint16_t>(x), static_cast<uint16_t>(y), static_cast<uint8_t>(polarity));
      }
      void finished() { proc->finished(); }
      void setTime(uint64_t t) { nextTime = t; }
      void setSize(size_t n) { bytesProcessed = n; }
      EventProcT * proc{nullptr};
      uint64_t tLimit{0};
      uint64_t nextTime{0};
      size_t bytesProcessed{0};
    } decodeUntil(processor, timeLimit);

    loopOverTime<DecodeUntil>(buf, bufSize, decodeUntil);
    *nextTime = decodeUntil.nextTime;
    return (decodeUntil.bytesProcessed);
  }

  bool summarize(
    const uint8_t * buf, size_t size, uint64_t * firstTS, uint64_t * lastTS,
    size_t * numEventsOnOff) override
  {
    // get start and stop times
    bool hasValidTime(false);
    if (size >= sizeof(PolarityEvent)) {
      const PolarityEvent & peStart = *reinterpret_cast<const PolarityEvent *>(buf);
      const auto tsLowStart = getTS(peStart);
      *firstTS = combineTS(timeBase_, tsLowStart);
      const auto tsLowEnd = getTS(*(reinterpret_cast<const PolarityEvent *>(buf + size) - 1));
      // The following line makes the assumption that there is only one overflow in the packet, i.e.
      // that a packet does not span more than about 35 mins.
      *lastTS = combineTS(
        timeBase_ + ((tsLowEnd < tsLowStart) ? ((1ULL << TS_OVERFLOW_SHIFT) * 1000) : 0), tsLowEnd);
      hasValidTime = true;
    }
    // get number of ON/OFF events
    struct EventCounter
    {
      bool timeExceedsLimit(uint64_t) const { return (false); }
      void eventCD(uint64_t, uint32_t, uint32_t, uint32_t polarity) { count[polarity]++; }
      void finished() {}         // do nothing
      void setTime(uint64_t) {}  // do nothing
      void setSize(size_t) {}    // do nothing
      size_t count[2]{0, 0};
    } eventCounter;

    if (numEventsOnOff) {
      loopOverTime<EventCounter>(buf, size, eventCounter);
      numEventsOnOff[0] = eventCounter.count[0];
      numEventsOnOff[1] = eventCounter.count[1];
    }
    return (hasValidTime);
  }

  void setTimeBase(uint64_t t) override { timeBase_ = t; }

  bool findFirstSensorTime(const uint8_t * buf, size_t size, uint64_t * firstTS) override
  {
    if (size >= sizeof(EventPacket)) {
      const PolarityEvent & peStart = *reinterpret_cast<const PolarityEvent *>(buf);
      const auto tsLowStart = getTS(peStart);
      *firstTS = combineTS(timeBase_, tsLowStart);
      return (true);
    }
    return (false);
  }

  bool findFirstSensorTime(const MsgT & msg, uint64_t * firstTS) override
  {
    setTimeBase(msg.time_base);
    return findFirstSensorTime(msg.events.data(), msg.events.size(), firstTS);
  }

  void setTimeMultiplier(uint32_t) override {}
  void setGeometry(uint16_t, uint16_t) override {}
  uint16_t getWidth() const override { return (0); }
  uint16_t getHeight() const override { return (0); }
  uint32_t getTimeMultiplier() const final { return (1); }
  bool hasSensorTimeSinceEpoch() const final { return (true); }

private:
  struct __attribute__((__packed__)) PolarityEvent
  {
    uint32_t data;
    int32_t timestamp;
  };

  static inline uint64_t combineTS(uint64_t tsHigh, uint64_t tsLow)
  {
    return (tsHigh + tsLow * 1000);
  }

  static inline uint64_t getTS(const PolarityEvent & pe)
  {
    return (
      static_cast<uint64_t>(static_cast<int32_t>(le32toh(static_cast<uint32_t>(pe.timestamp)))));
  }

  static inline uint32_t getField(uint32_t d, uint32_t shift, uint32_t mask)
  {
    return ((le32toh(d) >> shift) & mask);
  };

  template <class EventHandler>
  void loopOverTime(const uint8_t * buf, size_t bufSize, EventHandler & eh)
  {
    // For the decoding logic see libcaer's "polarity.h"
    // timeBase_ has the host (ROS) time when the driver started up, increased
    // by the high bits of the sensor time:
    // timeBase_ = t0_ + high_bits(elapsed sensor time)

    uint64_t ts_high = timeBase_;
    uint64_t ts_low_last = 0;  // only care about rollovers if they happen *within* packet
    for (const uint8_t * p_u8 = buf; p_u8 < buf + bufSize; p_u8 += 8) {
      const PolarityEvent & pe = *reinterpret_cast<const PolarityEvent *>(p_u8);
      const uint64_t ts_low = getTS(pe);
      if (ts_low < ts_low_last) {
        // roll over the high bits by one if the low bits roll over. Since the low bits
        // are in microseconds, but the time stamp is in nanoseconds, we need to multiply
        // by a factor of 1000
        ts_high += (1ULL << TS_OVERFLOW_SHIFT) * 1000;
      }
      const uint64_t t = combineTS(ts_high, ts_low);
      if (eh.timeExceedsLimit(t)) {
        eh.finished();
        eh.setTime(t);
        eh.setSize(p_u8 - buf);
        return;
      }
      eh.eventCD(
        t, getField(pe.data, X_SHIFT, X_MASK), getField(pe.data, Y_SHIFT, Y_MASK),
        getField(pe.data, POLARITY_SHIFT, POLARITY_MASK));
      ts_low_last = ts_low;
    }
    eh.finished();
    eh.setSize(bufSize);
  }

  // --------------------- variables
  uint64_t timeBase_{0};
  static constexpr uint32_t TS_OVERFLOW_SHIFT = 31;
  static constexpr uint32_t POLARITY_SHIFT = 1;
  static constexpr uint32_t POLARITY_MASK = 1;
  static constexpr uint32_t Y_SHIFT = 2;
  static constexpr uint32_t Y_MASK = 0x7FFF;
  static constexpr uint32_t X_SHIFT = 17;
  static constexpr uint32_t X_MASK = 0x7FFF;
};
}  // namespace libcaer
}  // namespace event_camera_codecs
#endif  // EVENT_CAMERA_CODECS__LIBCAER_DECODER_H_
