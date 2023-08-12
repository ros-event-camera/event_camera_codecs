// -*-c++-*--------------------------------------------------------------------
// Copyright 2023 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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

#include <event_camera_codecs/decoder.h>
#include <event_camera_codecs/decoder_factory.h>
#include <gtest/gtest.h>

#include "event_bag_reader.hpp"

using event_camera_codecs::EventPacket;

class CheckSumProcessor : public event_camera_codecs::EventProcessor
{
public:
  inline void eventCD(uint64_t t, uint16_t ex, uint16_t ey, uint8_t polarity) override
  {
    checkSumCD_t_ += t;
    checkSumCD_x_ += ex;
    checkSumCD_y_ += ey;
    checkSumCD_p_ += polarity;
  }
  void eventExtTrigger(uint64_t t, uint8_t edge, uint8_t id) override
  {
    checkSumTrigger_t_ += t;
    checkSumTrigger_edge_ += edge;
    checkSumTrigger_id_ += id;
  }
  void finished() override{};
  void rawData(const char *, size_t) override{};
  // --- own methods
  void verifyCheckSumCD(uint64_t t, uint64_t x, uint64_t y, uint64_t p)
  {
    ASSERT_EQ(checkSumCD_t_, t);
    ASSERT_EQ(checkSumCD_x_, x);
    ASSERT_EQ(checkSumCD_y_, y);
    ASSERT_EQ(checkSumCD_p_, p);
  }
  void verifyCheckSumTrigger(uint64_t t, uint64_t edge, uint64_t id)
  {
    ASSERT_EQ(checkSumTrigger_t_, t);
    ASSERT_EQ(checkSumTrigger_edge_, edge);
    ASSERT_EQ(checkSumTrigger_id_, id);
  }

private:
  uint64_t checkSumCD_t_{0};
  uint64_t checkSumCD_x_{0};
  uint64_t checkSumCD_y_{0};
  uint64_t checkSumCD_p_{0};
  uint64_t checkSumTrigger_t_{0};
  uint64_t checkSumTrigger_edge_{0};
  uint64_t checkSumTrigger_id_{0};
};

//
//  test that decoding of "mono" is working
//
TEST(event_camera_codecs, mono_cd_test)
{
  CheckSumProcessor proc;

  event_camera_codecs::DecoderFactory<EventPacket, CheckSumProcessor> decoderFactory;
  EventBagReader ebr("test_data_mono_cd");
  while (const auto msgPtr = ebr.next()) {
    auto decoder = decoderFactory.getInstance(*msgPtr);
    EXPECT_TRUE(decoder != nullptr);
    decoder->decode(*msgPtr, &proc);
  }
  proc.verifyCheckSumCD(4950000ULL, 34450ULL, 26450ULL, 50ULL);
}

//
//  test that decoding of "evt3" (including trigger events) is working
//
TEST(event_camera_codecs, evt3_trigger)
{
  CheckSumProcessor proc;

  event_camera_codecs::DecoderFactory<EventPacket, CheckSumProcessor> decoderFactory;
  EventBagReader ebr("test_data_evt3_trigger");
  while (const auto msgPtr = ebr.next()) {
    auto decoder = decoderFactory.getInstance(*msgPtr);
    EXPECT_TRUE(decoder != nullptr);
    decoder->decode(*msgPtr, &proc);
  }
  proc.verifyCheckSumCD(1222326831810000ULL, 42078718ULL, 31656267ULL, 41439ULL);
  proc.verifyCheckSumTrigger(29096982000ULL, 1ULL, 0ULL);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
