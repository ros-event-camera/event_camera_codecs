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

#include <rclcpp/rclcpp.hpp>

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
    if (debug_ && t < lastTime_) {
      std::cout << "t going backwards: last time: " << lastTime_ << ", t: " << t << std::endl;
    }
    EXPECT_TRUE(t >= lastTime_);
    lastTime_ = t;
  }
  void eventExtTrigger(uint64_t t, uint8_t edge, uint8_t id) override
  {
    checkSumTrigger_t_ += t;
    checkSumTrigger_edge_ += edge;
    checkSumTrigger_id_ += id;
    EXPECT_TRUE(t >= lastTime_);
    lastTime_ = t;
  }
  void finished() override{};
  void rawData(const char *, size_t) override{};
  // --- own methods
  void setDebug(bool b) { debug_ = b; }
  bool getDebug() const { return (debug_); }
  void incNumMessages() { numMessages_++; }
  void incNumDecodedCompletely(bool b) { numDecodedCompletely_ += b; }
  size_t getNumMessages() const { return (numMessages_); }
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

  void verifyLastTime(uint64_t t) { ASSERT_EQ(lastTime_, t); }
  void verifyNumDecodedCompletely(size_t n) { ASSERT_EQ(numDecodedCompletely_, n); }
  void verifyNumMessages(size_t n) { ASSERT_EQ(numMessages_, n); }

  void printCheckSums()
  {
    std::cout << "checkSumCD_t: " << checkSumCD_t_ << std::endl;
    std::cout << "checkSumCD_x: " << checkSumCD_x_ << std::endl;
    std::cout << "checkSumCD_y: " << checkSumCD_y_ << std::endl;
    std::cout << "checkSumCD_p: " << checkSumCD_p_ << std::endl;
    std::cout << "checkSumTrigger_t: " << checkSumTrigger_t_ << std::endl;
    std::cout << "checkSumTrigger_edge: " << checkSumTrigger_edge_ << std::endl;
    std::cout << "checkSumTrigger_id: " << checkSumTrigger_id_ << std::endl;
    std::cout << "lastTime_: " << lastTime_ << std::endl;
    std::cout << "numDecodedCompletely_: " << numDecodedCompletely_ << std::endl;
    std::cout << "numMessages_: " << numMessages_ << std::endl;
  }

private:
  uint64_t checkSumCD_t_{0};
  uint64_t checkSumCD_x_{0};
  uint64_t checkSumCD_y_{0};
  uint64_t checkSumCD_p_{0};
  uint64_t checkSumTrigger_t_{0};
  uint64_t checkSumTrigger_edge_{0};
  uint64_t checkSumTrigger_id_{0};
  uint64_t lastTime_{0};
  size_t numDecodedCompletely_{0};
  size_t numMessages_{0};
  bool debug_{false};
};

uint64_t test_decode_until(uint64_t untilTime, CheckSumProcessor * proc, const std::string & bag)
{
  uint64_t nextTime{0};
  event_camera_codecs::DecoderFactory<EventPacket, CheckSumProcessor> decoderFactory;
  EventBagReader ebr(bag);
  while (const auto msgPtr = ebr.next()) {
    if (proc->getDebug()) {
      std::cout << "---- start message ----: " << std::endl;
      std::cout << "msg stamp: " << rclcpp::Time(msgPtr->header.stamp).nanoseconds() << std::endl;
      std::cout << "time base: " << msgPtr->time_base << std::endl;
    }
    auto decoder = decoderFactory.getInstance(*msgPtr);
    EXPECT_TRUE(decoder != nullptr);
    // Technically the API demands that we call decodeUntil() again until it returns false.
    // We get around that by calling setTimeBase(). It's a hack really since the
    // state of the decoder gets messed up.
    decoder->setTimeBase(msgPtr->time_base);
    const bool dc = !decoder->decodeUntil(*msgPtr, proc, untilTime, &nextTime);
    proc->incNumDecodedCompletely(dc);
    proc->incNumMessages();
    if (proc->getDebug()) {
      std::cout << "---- finished message ----: " << proc->getNumMessages() << " "
                << static_cast<int>(dc) << std::endl;
    }
  }
  return (nextTime);
}

//
//  EVT3 tests(including trigger events)
//
TEST(event_camera_codecs, evt3_decode)
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

TEST(event_camera_codecs, evt3_decode_until_first)
{
  CheckSumProcessor proc;
  uint64_t untilTime{14252072000ULL};  // beginning of next packet
  (void)test_decode_until(untilTime, &proc, "test_data_evt3_trigger");
  proc.verifyCheckSumCD(29004181874000ULL, 897009ULL, 773502ULL, 1028ULL);
  proc.verifyLastTime(14252063000ULL);
  proc.verifyNumDecodedCompletely(3ULL);
  proc.verifyNumMessages(105ULL);
}

TEST(event_camera_codecs, evt3_decode_until_middle)
{
  CheckSumProcessor proc;
  uint64_t untilTime{14251634000ULL};  // middle of packet
  (void)test_decode_until(untilTime, &proc, "test_data_evt3_trigger");
  proc.verifyCheckSumCD(28491115518000ULL, 880131ULL, 758865ULL, 1006ULL);
  proc.verifyLastTime(14251630000ULL);
  proc.verifyNumDecodedCompletely(3ULL);
  proc.verifyNumMessages(105ULL);
}

TEST(event_camera_codecs, evt3_decode_until_last)
{
  CheckSumProcessor proc;
  uint64_t untilTime{14252063000ULL};  // exactly end of packet
  (void)test_decode_until(untilTime, &proc, "test_data_evt3_trigger");
  proc.verifyCheckSumCD(28989929811000ULL, 896982ULL, 773451ULL, 1027ULL);
  proc.verifyLastTime(14252047000);
  proc.verifyNumDecodedCompletely(3);
  proc.verifyNumMessages(105ULL);
}

//
//  ---------- mono tests
//
TEST(event_camera_codecs, mono_cd_decode)
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

TEST(event_camera_codecs, mono_cd_decode_until_first)
{
  CheckSumProcessor proc;
  uint64_t untilTime{50000ULL};  // first event in packet
  (void)test_decode_until(untilTime, &proc, "test_data_mono_cd");
  proc.verifyCheckSumCD(1225000ULL, 17225ULL, 13225ULL, 25ULL);
  proc.verifyLastTime(49000ULL);
  proc.verifyNumDecodedCompletely(1ULL);
  proc.verifyNumMessages(2ULL);
}

TEST(event_camera_codecs, mono_cd_decode_until_middle)
{
  CheckSumProcessor proc;
  uint64_t untilTime{80000ULL};  // event in middle of packet
  (void)test_decode_until(untilTime, &proc, "test_data_mono_cd");
  proc.verifyCheckSumCD(3160000ULL, 27260ULL, 20860ULL, 40ULL);
  proc.verifyLastTime(79000);
  proc.verifyNumDecodedCompletely(1ULL);
  proc.verifyNumMessages(2ULL);
}

TEST(event_camera_codecs, mono_cd_decode_until_last)
{
  CheckSumProcessor proc;
  uint64_t untilTime{49000ULL};  // last event in previous packet
  (void)test_decode_until(untilTime, &proc, "test_data_mono_cd");
  proc.verifyCheckSumCD(1176000ULL, 16856ULL, 12936ULL, 24ULL);
  proc.verifyLastTime(48000ULL);
  proc.verifyNumDecodedCompletely(0ULL);
  proc.verifyNumMessages(2ULL);
}

//
// ------------------- libcaer tests
//
TEST(event_camera_codecs, libcaer_decode)
{
  CheckSumProcessor proc;
  event_camera_codecs::DecoderFactory<EventPacket, CheckSumProcessor> decoderFactory;
  EventBagReader ebr("test_data_libcaer_rollover");
  while (const auto msgPtr = ebr.next()) {
    auto decoder = decoderFactory.getInstance(*msgPtr);
    EXPECT_TRUE(decoder != nullptr);
    decoder->decode(*msgPtr, &proc);
  }
  proc.verifyCheckSumCD(15762826572202229126ULL, 111651612ULL, 82801007ULL, 181908ULL);
}

TEST(event_camera_codecs, libcaer_decode_until_very_first)
{
  CheckSumProcessor proc;
  uint64_t untilTime{1702309120762236018ULL};  // first event in first packet
  (void)test_decode_until(untilTime, &proc, "test_data_libcaer_rollover");
  proc.verifyCheckSumCD(0ULL, 0ULL, 0ULL, 0ULL);
  proc.verifyLastTime(0ULL);
  proc.verifyNumDecodedCompletely(0ULL);
  proc.verifyNumMessages(201ULL);
}

TEST(event_camera_codecs, libcaer_decode_until_first)
{
  CheckSumProcessor proc;
  uint64_t untilTime{1702309120772229018ULL};  // first event in second packet
  (void)test_decode_until(untilTime, &proc, "test_data_libcaer_rollover");
  proc.verifyCheckSumCD(15706488888233371866ULL, 533572ULL, 401707ULL, 872ULL);
  proc.verifyLastTime(1702309120772213018ULL);
  proc.verifyNumDecodedCompletely(1ULL);
  proc.verifyNumMessages(201ULL);
}

TEST(event_camera_codecs, libcaer_decode_until_middle)
{
  CheckSumProcessor proc;
  uint64_t untilTime{1702309120773239018ULL};  // event in middle of second packet
  (void)test_decode_until(untilTime, &proc, "test_data_libcaer_rollover");
  proc.verifyCheckSumCD(3141897757159270998ULL, 582593ULL, 442271ULL, 951ULL);
  proc.verifyLastTime(1702309120773233018ULL);
  proc.verifyNumDecodedCompletely(1ULL);
  proc.verifyNumMessages(201ULL);
}

TEST(event_camera_codecs, libcaer_decode_until_last)
{
  CheckSumProcessor proc;
  uint64_t untilTime{1702309120782222018ULL};  // last event of second packet
  (void)test_decode_until(untilTime, &proc, "test_data_libcaer_rollover");
  proc.verifyCheckSumCD(12099893362926800844ULL, 1068301ULL, 808043ULL, 1788ULL);
  proc.verifyLastTime(1702309120782216018ULL);
  proc.verifyNumDecodedCompletely(2ULL);
  proc.verifyNumMessages(201ULL);
}

TEST(event_camera_codecs, libcaer_summarize)
{
  event_camera_codecs::DecoderFactory<EventPacket, CheckSumProcessor> decoderFactory;
  EventBagReader ebr("test_data_libcaer_rollover");
  const auto msgPtr = ebr.next();
  EXPECT_TRUE(msgPtr != nullptr);
  auto decoder = decoderFactory.getInstance(*msgPtr);
  EXPECT_TRUE(decoder != nullptr);
  uint64_t firstTS{0};
  uint64_t lastTS{0};
  size_t numEventsOnOff[2]{0, 0};
  decoder->summarize(*msgPtr, &firstTS, &lastTS, numEventsOnOff);
  ASSERT_EQ(firstTS, 1702309120762236018ULL);
  ASSERT_EQ(lastTS, 1702309120772213018ULL);
  ASSERT_EQ(numEventsOnOff[0], 741ULL);
  ASSERT_EQ(numEventsOnOff[1], 872ULL);
}

TEST(event_camera_codecs, libcaer_find_first)
{
  event_camera_codecs::DecoderFactory<EventPacket, CheckSumProcessor> decoderFactory;
  EventBagReader ebr("test_data_libcaer_rollover");
  const auto msgPtr = ebr.next();
  EXPECT_TRUE(msgPtr != nullptr);
  auto decoder = decoderFactory.getInstance(*msgPtr);
  EXPECT_TRUE(decoder != nullptr);
  uint64_t firstTS{0};
  decoder->findFirstSensorTime(*msgPtr, &firstTS);
  ASSERT_EQ(firstTS, 1702309120762236018ULL);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
