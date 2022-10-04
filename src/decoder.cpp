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

#include "event_array_codecs/decoder.h"

#include <unordered_map>

#include "event_array_codecs/evt3_decoder.h"
#include "event_array_codecs/mono_decoder.h"
#include "event_array_codecs/trigger_decoder.h"

namespace event_array_codecs
{
std::shared_ptr<Decoder> Decoder::newInstance(const std::string & codec)
{
  if (codec == "evt3") {
    return (std::make_shared<evt3::Decoder>());
  } else if (codec == "mono") {
    return (std::make_shared<mono::Decoder>());
  } else if (codec == "trigger") {
    return (std::make_shared<trigger::Decoder>());
  }
  return (std::shared_ptr<Decoder>());
}

static std::unordered_map<std::string, std::shared_ptr<Decoder>> decoder_map;

std::shared_ptr<Decoder> Decoder::getInstance(const std::string & codec)
{
  auto it = decoder_map.find(codec);
  if (it == decoder_map.end()) {
    auto elem = decoder_map.insert({codec, newInstance(codec)});
    return (elem.first->second);
  }
  return (it->second);
}
}  // namespace event_array_codecs
