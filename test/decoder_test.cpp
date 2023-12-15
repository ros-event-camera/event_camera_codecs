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
    ASSERT_LT(ex, width_);
    ASSERT_LT(ey, height_);
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
  void setGeometry(uint16_t w, uint16_t h)
  {
    width_ = w;
    height_ = h;
  }
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
  uint16_t width_{0};
  uint16_t height_{0};
  size_t numDecodedCompletely_{0};
  size_t numMessages_{0};
  bool debug_{false};
};

class SummaryTester
{
public:
  void setFirstTS(uint64_t t) { firstTS_ = t; }
  void setLastTS(uint64_t t) { lastTS_ = t; }
  void setNumOff(size_t n) { numEventsOnOff_[0] = n; }
  void setNumOn(size_t n) { numEventsOnOff_[1] = n; }
  void test(uint64_t firstTS, uint64_t lastTS, size_t numOff, size_t numOn)
  {
    ASSERT_GE(lastTS_, firstTS_);
    ASSERT_EQ(firstTS_, firstTS);
    ASSERT_EQ(lastTS_, lastTS);
    ASSERT_EQ(numEventsOnOff_[0], numOff);
    ASSERT_EQ(numEventsOnOff_[1], numOn);
  }
  void print() const
  {
    std::cout << firstTS_ << ", " << lastTS_ << ", " << numEventsOnOff_[0] << ", "
              << numEventsOnOff_[1] << std::endl;
  }

private:
  uint64_t firstTS_{0};
  uint64_t lastTS_{0};
  size_t numEventsOnOff_[2]{0, 0};
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
    proc->setGeometry(msgPtr->width, msgPtr->height);
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

void test_decode(CheckSumProcessor * proc, const std::string & bag)
{
  event_camera_codecs::DecoderFactory<EventPacket, CheckSumProcessor> decoderFactory;
  EventBagReader ebr(bag);
  while (const auto msgPtr = ebr.next()) {
    proc->setGeometry(msgPtr->width, msgPtr->height);
    auto decoder = decoderFactory.getInstance(*msgPtr);
    EXPECT_TRUE(decoder != nullptr);
    decoder->decode(*msgPtr, proc);
  }
}

void test_summarize(SummaryTester * st, const std::string & bag)
{
  event_camera_codecs::DecoderFactory<EventPacket, CheckSumProcessor> decoderFactory;
  EventBagReader ebr(bag);
  const auto msgPtr = ebr.next();
  EXPECT_TRUE(msgPtr != nullptr);
  auto decoder = decoderFactory.getInstance(*msgPtr);
  EXPECT_TRUE(decoder != nullptr);
  uint64_t firstTS{0};
  uint64_t lastTS{0};
  size_t numOnOff[2]{0, 0};
  decoder->summarize(*msgPtr, &firstTS, &lastTS, numOnOff);
  st->setFirstTS(firstTS);
  st->setLastTS(lastTS);
  st->setNumOff(numOnOff[0]);
  st->setNumOn(numOnOff[1]);
}

uint64_t test_find_first(const std::string & bag)
{
  event_camera_codecs::DecoderFactory<EventPacket, CheckSumProcessor> decoderFactory;
  EventBagReader ebr(bag);
  auto msgPtr = ebr.next();
  EXPECT_TRUE(msgPtr != nullptr);
  msgPtr = ebr.next();  // skip first packet
  EXPECT_TRUE(msgPtr != nullptr);
  auto decoder = decoderFactory.getInstance(*msgPtr);
  EXPECT_TRUE(decoder != nullptr);
  uint64_t firstTS{0};
  decoder->findFirstSensorTime(*msgPtr, &firstTS);
  return (firstTS);
}

//
//  EVT3 tests(including trigger events)
//
TEST(event_camera_codecs, evt3_decode)
{
  CheckSumProcessor proc;
  test_decode(&proc, "test_data_evt3_trigger");
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
  test_decode(&proc, "test_data_mono_cd");
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
  test_decode(&proc, "test_data_libcaer_rollover");
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
  SummaryTester st;
  test_summarize(&st, "test_data_libcaer_rollover");
  st.test(1702309120762236018ULL, 1702309120772213018ULL, 741ULL, 872ULL);
}

TEST(event_camera_codecs, libcaer_find_first)
{
  ASSERT_EQ(1702309120772229018ULL, test_find_first("test_data_libcaer_rollover"));
}

//
// ------------------- libcaer_cmp tests
//
TEST(event_camera_codecs, libcaer_cmp_decode)
{
  CheckSumProcessor proc;
  test_decode(&proc, "test_data_libcaer_cmp");
  proc.verifyCheckSumCD(14023744309793358664ULL, 269115819ULL, 200213171ULL, 491741ULL);
}

TEST(event_camera_codecs, libcaer_cmp_summarize)
{
  SummaryTester st;
  test_summarize(&st, "test_data_libcaer_cmp");
  st.test(1702669729854452000ULL, 1702669729864451000ULL, 1690ULL, 2627ULL);
}

TEST(event_camera_codecs, libcaer_cmp_find_first)
{
  ASSERT_EQ(1702669729864452000ULL, test_find_first("test_data_libcaer_cmp"));
}

TEST(event_camera_codecs, libcaer_cmp_decode_until_first)
{
  CheckSumProcessor proc;
  uint64_t untilTime{1702669729864452000ULL};  // first event of second packet
  (void)test_decode_until(untilTime, &proc, "test_data_libcaer_cmp");
  proc.printCheckSums();
  proc.verifyCheckSumCD(8621082466655168832ULL, 1397945ULL, 1059167ULL, 2627ULL);
  proc.verifyLastTime(1702669729864451000);
  proc.verifyNumDecodedCompletely(1ULL);
  proc.verifyNumMessages(191ULL);
}

TEST(event_camera_codecs, libcaer_cmp_decode_until_middle)
{
  CheckSumProcessor proc;
  uint64_t untilTime{1702669729874411000ULL};  // event middle of second packet
  (void)test_decode_until(untilTime, &proc, "test_data_libcaer_cmp");
  proc.printCheckSums();

  proc.verifyCheckSumCD(7308769552432234048ULL, 2811188ULL, 2123354ULL, 5224ULL);
  proc.verifyLastTime(1702669729874406000ULL);

  proc.verifyNumDecodedCompletely(2ULL);
  proc.verifyNumMessages(191ULL);
}

TEST(event_camera_codecs, libcaer_cmp_decode_until_last)
{
  CheckSumProcessor proc;
  uint64_t untilTime{1702669729874450000ULL};  // last event of second packet
  (void)test_decode_until(untilTime, &proc, "test_data_libcaer_cmp");
  proc.printCheckSums();
  proc.verifyCheckSumCD(2766006272627320816ULL, 2819158ULL, 2127214ULL, 5235ULL);
  proc.verifyLastTime(1702669729874446000ULL);
  proc.verifyNumDecodedCompletely(2ULL);
  proc.verifyNumMessages(191ULL);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
