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

#ifndef EVENT_ARRAY_CODECS__EVT3_ENCODER_H_
#define EVENT_ARRAY_CODECS__EVT3_ENCODER_H_

#include <event_array_codecs/evt3_types.h>
#include <stdint.h>

#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace event_array_codecs
{
namespace evt3
{
class Encoder : public event_array_codecs::Encoder
{
public:
  // ---- inherited from Encoder
  void setBuffer(std::vector<uint8_t> * buf) override { bufferPtr_ = buf; }
  void setSensorTime(uint64_t t) override
  {
    timeBase_ = t;
    const uint64_t sensorTime = t / 1000;  // sensor time in usec
    const uint32_t evtTime = evt_from_sensor_time(sensorTime);
    updateEvtTime<TimeHigh>(time_high_from_evt(evtTime), &evtTimeHigh_);
    updateEvtTime<TimeLow>(time_low_from_evt(evtTime), &evtTimeLow_);
  }
  inline void encodeCD(int32_t dt, uint16_t x, uint16_t y, uint8_t p) override
  {
    // sorry, totally unoptimized encoding for now!
    updateTime(dt);
    if (y != y_) {
      write<AddrY>(AddrY(y, 0));
      y_ = y;
    }
    write<AddrX>(AddrX(x, p));
  }

  inline void encodeExtTrigger(int32_t dt, uint8_t edge, uint8_t id) override
  {
    updateTime(dt);
    write<ExtTrigger>(ExtTrigger(edge, id));
  }
  inline void flush() override
  {
    // no need to flush buffer
  }

private:
  inline void updateTime(uint32_t dt)
  {
    const uint64_t newSensorTime = (timeBase_ + dt) / 1000;  // sensor time in usec
    const uint32_t newEvtTime = evt_from_sensor_time(newSensorTime);

    const uint32_t newEvtTimeHigh = time_high_from_evt(newEvtTime);
    if (newEvtTimeHigh != evtTimeHigh_) {
      updateEvtTime<TimeHigh>(newEvtTimeHigh, &evtTimeHigh_);
    }

    const uint32_t newEvtTimeLow = time_low_from_evt(newEvtTime);
    if (newEvtTimeLow != evtTimeLow_) {
      updateEvtTime<TimeLow>(newEvtTimeLow, &evtTimeLow_);
    }
  }

  template <class T>
  inline void write(const T & data)
  {
    bufferPtr_->resize(bufferPtr_->size() + sizeof(T));
    std::memcpy(
      &((*bufferPtr_)[0]) + bufferPtr_->size() - sizeof(T),
      reinterpret_cast<const uint8_t *>(&data), sizeof(T));
  }

  template <class T>
  inline void updateEvtTime(uint32_t time, uint32_t * target)
  {
    T t(time);
    write<T>(t);
    *target = time;
  }

  static inline uint32_t evt_from_sensor_time(uint64_t t)
  {
    return (t & ((1 << 24) - 1));  // just lower 24 bits
  }

  static inline uint32_t time_high_from_evt(uint32_t t)
  {
    return (t >> 12);  // just bits higher than 12
  }

  static inline uint32_t time_low_from_evt(uint32_t t)
  {
    return (t & ((1 << 12) - 1));  // just lower 12 bits
  }

  // -------------------------------- variables -------------------
  std::vector<uint8_t> * bufferPtr_;
  uint64_t timeBase_{0};
  uint32_t evtTimeHigh_{0};
  uint32_t evtTimeLow_{0};
  uint16_t y_{(1 << 16) - 1};
};

}  // namespace evt3
}  // namespace event_array_codecs
#endif  // EVENT_ARRAY_CODECS__EVT3_ENCODER_H_
