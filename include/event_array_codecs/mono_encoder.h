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

#ifndef EVENT_ARRAY_CODECS__MONO_ENCODER_H_
#define EVENT_ARRAY_CODECS__MONO_ENCODER_H_

#include <stdint.h>

#include <memory>
#include <string>
#include <vector>

namespace event_array_codecs
{
namespace mono
{
class Encoder : public event_array_codecs::Encoder
{
public:
  const size_t bytes_per_event = 8;
  // ---- inherited from Encoder
  void setBuffer(std::vector<uint8_t> * buf) override { bufferPtr_ = buf; }
  void setSensorTime(uint64_t) override
  {
    // mono encoding relies on sensor time being provided by message header
  }

  inline void encodeCD(int32_t dt, uint16_t x, uint16_t y, uint8_t p) override
  {
    dt = std::max(dt, 0);
    const auto old_size = bufferPtr_->size();
    bufferPtr_->resize(old_size + bytes_per_event);
    uint64_t * packed = reinterpret_cast<uint64_t *>(&(*bufferPtr_)[old_size]);
    *packed = static_cast<uint64_t>(p) << 63 | static_cast<uint64_t>(y) << 48 |
              static_cast<uint64_t>(x) << 32 | static_cast<uint64_t>(dt);
  }
  inline void encodeExtTrigger(int32_t dt, uint8_t edge, uint8_t id) override
  {
    dt = std::max(dt, 0);
    const auto old_size = bufferPtr_->size();
    bufferPtr_->resize(old_size + bytes_per_event);
    uint64_t * packed = reinterpret_cast<uint64_t *>(&(*bufferPtr_)[old_size]);
    *packed = static_cast<uint64_t>(edge) << 63 | static_cast<uint64_t>(id) << 48 |
              static_cast<uint64_t>(dt);
  }
  inline void flush() override
  {
    // no need to flush buffer for mono encoding
  }

private:
  std::vector<uint8_t> * bufferPtr_;
};

}  // namespace mono
}  // namespace event_array_codecs
#endif  // EVENT_ARRAY_CODECS__MONO_ENCODER_H_
