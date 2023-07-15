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

#ifndef EVENT_CAMERA_CODECS__DECODER_FACTORY_H_
#define EVENT_CAMERA_CODECS__DECODER_FACTORY_H_

#include <event_camera_codecs/decoder.h>
#include <event_camera_codecs/evt3_decoder.h>
#include <event_camera_codecs/mono_decoder.h>
#include <event_camera_codecs/noop_event_processor.h>
#include <event_camera_codecs/trigger_decoder.h>
#include <stdint.h>

#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace event_camera_codecs
{
template <class MsgT, class EventProcT = NoOpEventProcessor>
class DecoderFactory
{
public:
  // factory method to create new instance
  std::shared_ptr<Decoder<MsgT, EventProcT>> newInstance(const std::string &)
  {
    throw std::runtime_error("no decoder available for this message type!");
  }

  // factory method to get decoder from shared pool
  Decoder<MsgT, EventProcT> * getInstance(
    const std::string & codec, uint16_t width, uint16_t height)
  {
    throw std::runtime_error("no decoder available for this message type!");
  }
};

template <class EventProcT>
class DecoderFactory<EventPacket, EventProcT>
{
public:
  std::shared_ptr<Decoder<EventPacket, EventProcT>> newInstance(const std::string & codec)
  {
    if (codec == "evt3") {
      return (std::make_shared<evt3::Decoder<EventPacket, EventProcT>>());
    } else if (codec == "mono") {
      return (std::make_shared<mono::Decoder<EventPacket, EventProcT>>());
    } else if (codec == "trigger") {
      return (std::make_shared<trigger::Decoder<EventPacket, EventProcT>>());
    }
    // return null pointer if codec not found
    return (nullptr);
  }

  Decoder<EventPacket, EventProcT> * getInstance(
    const std::string & codec, uint16_t width, uint16_t height)
  {
    auto it = decoderMap_.find(codec);
    if (it == decoderMap_.end()) {
      auto elem = decoderMap_.insert({codec, newInstance(codec)});
      elem.first->second->setGeometry(width, height);
      return (elem.first->second.get());
    }
    return (it->second.get());
  }

private:
  std::unordered_map<std::string, std::shared_ptr<Decoder<EventPacket, EventProcT>>> decoderMap_;
};

}  // namespace event_camera_codecs
#endif  // EVENT_CAMERA_CODECS__DECODER_FACTORY_H_
