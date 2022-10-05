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

#ifndef EVENT_ARRAY_CODECS__DECODER_FACTORY_H_
#define EVENT_ARRAY_CODECS__DECODER_FACTORY_H_

#include <event_array_codecs/decoder.h>
#include <event_array_codecs/evt3_decoder.h>
#include <event_array_codecs/mono_decoder.h>
#include <event_array_codecs/noop_event_processor.h>
#include <event_array_codecs/trigger_decoder.h>
#include <stdint.h>

#include <memory>
#include <string>
#include <unordered_map>

namespace event_array_codecs
{
template <class EventProcT = NoOpEventProcessor>
class DecoderFactory
{
public:
  // factory method to create new instance
  std::shared_ptr<Decoder<EventProcT>> newInstance(const std::string & codec)
  {
    if (codec == "evt3") {
      return (std::make_shared<evt3::Decoder<EventProcT>>());
    } else if (codec == "mono") {
      return (std::make_shared<mono::Decoder<EventProcT>>());
    } else if (codec == "trigger") {
      return (std::make_shared<trigger::Decoder<EventProcT>>());
    }
    // return null pointer if codec not found
    return (std::shared_ptr<mono::Decoder<EventProcT>>());
  }
  // factory method to get decoder from shared pool
  std::shared_ptr<Decoder<EventProcT>> getInstance(const std::string & codec)
  {
    auto it = decoderMap_.find(codec);
    if (it == decoderMap_.end()) {
      auto elem = decoderMap_.insert({codec, newInstance(codec)});
      return (elem.first->second);
    }
    return (it->second);
  }

private:
  std::unordered_map<std::string, std::shared_ptr<Decoder<EventProcT>>> decoderMap_;
};

}  // namespace event_array_codecs
#endif  // EVENT_ARRAY_CODECS__DECODER_FACTORY_H_
