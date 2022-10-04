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

#ifndef EVENT_ARRAY_CODECS__DECODER_H_
#define EVENT_ARRAY_CODECS__DECODER_H_

#include <stdint.h>

#include <memory>
#include <string>

namespace event_array_codecs
{
template <class EventProcT>
class Decoder
{
public:
  virtual ~Decoder() {}
  // ---- interface methods
  virtual void decode(const uint8_t * buf, size_t bufSize, EventProcT * processor) = 0;
  virtual bool summarize(
    const uint8_t * buf, size_t size, uint64_t * firstTS, uint64_t * lastTS,
    size_t * numEventsOnOff) = 0;
  virtual void setTimeBase(const uint64_t timeBase) = 0;
  virtual bool findFirstSensorTime(const uint8_t * buf, size_t size, uint64_t * firstTS) = 0;
};

}  // namespace event_array_codecs
#endif  // EVENT_ARRAY_CODECS__DECODER_H_
