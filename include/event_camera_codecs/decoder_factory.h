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
#include <event_camera_codecs/event_packet.h>
#include <event_camera_codecs/evt3_decoder.h>
#include <event_camera_codecs/libcaer_cmp_decoder.h>
#include <event_camera_codecs/libcaer_decoder.h>
#include <event_camera_codecs/mono_decoder.h>
#include <event_camera_codecs/noop_event_processor.h>
#include <event_camera_codecs/ros1_ros2_compat.h>
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
  Decoder<MsgT, EventProcT> * getInstance(const MsgT & msg)
  {
    return (getInstance(msg.encoding, msg.width, msg.height));
  }

  // (deprecated) factory method to get decoder from shared pool
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
    } else if (codec == "libcaer") {
      return (std::make_shared<libcaer::Decoder<EventPacket, EventProcT>>());
    } else if (codec == "libcaer_cmp") {
      return (std::make_shared<libcaer_cmp::Decoder<EventPacket, EventProcT>>());
    }
    // return null pointer if codec not found
    return (nullptr);
  }

  // factory method to get decoder from shared pool
  Decoder<EventPacket, EventProcT> * getInstance(const EventPacket & msg)
  {
    return (getInstance(msg.encoding, msg.width, msg.height));
  }

  // (deprecated) factory method to get decoder from shared pool
  Decoder<EventPacket, EventProcT> * getInstance(
    const std::string & codec, uint16_t width, uint16_t height)
  {
    const DecoderKey key(DecoderKey(codec, width, height));
    auto it = decoderMap_.find(key);
    if (it == decoderMap_.end()) {
      auto c = newInstance(codec);
      if (c != 0) {
        auto elem = decoderMap_.insert({key, newInstance(codec)});
        elem.first->second->setGeometry(width, height);
        return (elem.first->second.get());
      } else {
        return (nullptr);
      }
    }
    return (it->second.get());
  }

private:
  class DecoderKey
  {
  public:
    DecoderKey(const std::string & enc, uint16_t w, uint16_t h) : enc_(enc), w_(w), h_(h) {}
    bool operator==(const DecoderKey & k) const
    {
      return (enc_ == k.enc_ && w_ == k.w_ && h_ == k.h_);
    }
    const std::string & getEncoding() const { return (enc_); }
    uint16_t getWidth() const { return (w_); }
    uint16_t getHeight() const { return (h_); }

  private:
    std::string enc_;
    uint16_t w_{0};
    uint16_t h_{0};
  };
  struct hash_fn
  {
    size_t operator()(const DecoderKey & k) const
    {
      return (std::hash<std::string>()(k.getEncoding()) + k.getWidth() + k.getHeight());
    }
  };
  std::unordered_map<DecoderKey, std::shared_ptr<Decoder<EventPacket, EventProcT>>, hash_fn>
    decoderMap_;
};  // namespace event_camera_codecs

}  // namespace event_camera_codecs
#endif  // EVENT_CAMERA_CODECS__DECODER_FACTORY_H_
