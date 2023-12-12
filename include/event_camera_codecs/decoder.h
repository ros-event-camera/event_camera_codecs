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

#ifndef EVENT_CAMERA_CODECS__DECODER_H_
#define EVENT_CAMERA_CODECS__DECODER_H_

#include <event_camera_codecs/noop_event_processor.h>
#include <event_camera_codecs/ros1_ros2_compat.h>
#include <stdint.h>

#include <memory>
#include <string>

namespace event_camera_codecs
{
/*!
\warning This API is unstable and may change in the near future.

\brief Base class of decoders for different encodings.

The Decoder class is templated by message type so it can leverage all fields
of a message. It is also templated by the event processor which permits
for efficient inlining of the processing code, i.e. a decoded event
can be processed immediately without incurring the cost of storing
the event in memory.

\tparam MsgT ROS message type to decode, e.g. EventPacket
\tparam EventProcT EventProcessor class that gets called once an event is decoded.
*/
template <class MsgT, class EventProcT = NoOpEventProcessor>
class Decoder
{
public:
  virtual ~Decoder() = default;

  /*!
  \brief Decodes entire message, produces callbacks to \p processor.
  Use this method to feed the decoder with messages in case you
  want the entire message decoded.
  \param msg message to decode
  \param processor event processor to call when events are decoded
  */
  void decode(const MsgT & msg, EventProcT * processor)
  {
    processor->rawData(reinterpret_cast<const char *>(msg.events.data()), msg.events.size());
    setTimeBase(msg.time_base);
    decode(msg.events.data(), msg.events.size(), processor);
  }

  /*!
  \brief  Decodes a message up to, but not including, \p timeLimit .

  If the time limit has been reached (implying not the entire message has been processed),
  decodeUntil() must be called again with the same \p msg argument,
  to process the remaining events in the message. Do *not* use decode() and decodeUntil()
  on the same decoder since the two methods advance the state of the decoder
  differently. Also, you *must* call decodeUntil() until all bytes have been used up,
  i.e until it returns false, even if you don't care about the remaining events of the
  message.
  \param msg message to be decoded
  \param processor event processor to call when events are decoded
  \param timeLimit sensor time limit up to (but not including) which decoding should happen
  \param nextTime time following the last decoded event. NOT VALID WHEN RETURN VALUE IS FALSE!
  \return true if time limit has been reached, i.e. end of \p msg has not yet been reached.
  */
  bool decodeUntil(
    const MsgT & msg, EventProcT * processor, uint64_t timeLimit, uint64_t * nextTime)
  {
    return (decodeUntil(
      msg.events.data(), msg.events.size(), processor, timeLimit, msg.time_base, nextTime));
  }

  /*!
  \brief Provides same functionality as decodeUntil(msg, ...), but without using a message type.
  This is helpful when using languages like Python.
  \param buf pointer to first byte of message. Do not advance the pointer!
  \param bufSize number of bytes in event packet. Set to size of message event buffer.
  \param processor event processor to call when events are decoded
  \param timeLimit sensor time limit up to (but not including) which decoding should happen
  \param timeBase time bases for events in packet (may not be used)
  \param nextTime time following the last decoded event. NOT VALID WHEN RETURN VALUE IS FALSE!
  \return true if time limit has been reached, i.e. end of \p msg has not yet been reached.
  */
  bool decodeUntil(
    const uint8_t * buf, size_t bufSize, EventProcT * processor, uint64_t timeLimit,
    uint64_t timeBase, uint64_t * nextTime)
  {
    if (bytesUsed_ == 0) {
      processor->rawData(reinterpret_cast<const char *>(buf), bufSize);
      setTimeBase(timeBase);  // this should already be done before
    }
    size_t bytesConsumed =
      decodeUntil(buf + bytesUsed_, bufSize - bytesUsed_, processor, timeLimit, nextTime);
    bytesUsed_ += bytesConsumed;
    const bool reachedTimeLimit = (bytesUsed_ < bufSize);
    if (!reachedTimeLimit) {
      bytesUsed_ = 0;  // reached end-of-message, reset pointer for next message
    }
    return (reachedTimeLimit);
  }

  /*!
  \brief Summarizes message statistics. Use this function to peek into
  message without getting processing callbacks.
  \param msg message to summarize.
  \param firstTS first timestamp (will not be set if \p msg does not contain timestamp)
  \param lastTS last timestamp (will not be set if \p msg does not contain timestamp)
  \param numEventsOnOff number of events in message (must be pointer to array of size 2)
  \return true if timestamp has been found (i.e. firstTS and lastTS are valid)
  */

  bool summarize(const MsgT & msg, uint64_t * firstTS, uint64_t * lastTS, size_t * numEventsOnOff)
  {
    setTimeBase(msg.time_base);
    return (summarize(msg.events.data(), msg.events.size(), firstTS, lastTS, numEventsOnOff));
  }

  // ---- interface methods
  /*!
  \brief decode buffer without using message type. Use this method if the message is
  not available, e.g. when calling from python. 
  */
  virtual void decode(const uint8_t * buf, size_t bufSize, EventProcT * processor) = 0;
  /*!
  \brief decode buffer until reaching time limit, without using message type. Use
  this method if the message is not available, e.g. when calling from python.
  */
  virtual size_t decodeUntil(
    const uint8_t * buf, size_t bufSize, EventProcT * processor, uint64_t timeLimit,
    uint64_t * nextTime) = 0;
  /*!
  See summarize()
  */
  virtual bool summarize(
    const uint8_t * buf, size_t size, uint64_t * firstTS, uint64_t * lastTS,
    size_t * numEventsOnOff) = 0;
  /*!
  \brief Sets time base, i.e the reference time to which the event times refer to. For
  some codecs (like evt3) this has no effect.
  \param timeBase typically time in nanoseconds since epoch.
  */
  virtual void setTimeBase(const uint64_t timeBase) = 0;
  /*!
  \brief Finds first sensor time in event packet. This can be much faster than
    decoding the entire packet.
  \param buf buffer with event data
  \param size size of buffer with event data (bytes)
  \param firstTS first time stamp (only valid if "true" is returned!)
  \return true if sensor time has been found
  */
  virtual bool findFirstSensorTime(const uint8_t * buf, size_t size, uint64_t * firstTS) = 0;
  /*!
  \brief Finds first sensor time in event packet. This can be much faster than
  decoding the entire packet.
  \param msg event packet message
  \param firstTS first time stamp (only valid if "true" is returned!)
  \return true if sensor time has been found
  */
  virtual bool findFirstSensorTime(const MsgT & msg, uint64_t * firstTS) = 0;
  /*!
  \brief Sets the time multiplier
  \param mult Time multiplier. If the sensor time is natively in usec and \p mult is set to 1000,
   then the decoded time will be in nanoseconds.
  */
  virtual void setTimeMultiplier(uint32_t mult) = 0;

  /*!
  \brief Sets sensor geometry. Must be called before first call to "decode". Necessary for sanity
  checks.
  \param width sensor width in pixels
  \param height sensor height in pixels
  */
  virtual void setGeometry(uint16_t width, uint16_t height) = 0;
  /*!
  \brief Gets sensor width.
  \return width of sensor
  */
  virtual uint16_t getWidth() const = 0;
  /*!
  \brief Gets sensor height.
  \return height of sensor
  */
  virtual uint16_t getHeight() const = 0;

protected:
  uint16_t width_{0};   // sensor geometry
  uint16_t height_{0};  // sensor geometry

private:
  // ----------- variables
  size_t bytesUsed_{0};  //! used to keep track of how far a message has already been decoded
};

}  // namespace event_camera_codecs
#endif  // EVENT_CAMERA_CODECS__DECODER_H_
