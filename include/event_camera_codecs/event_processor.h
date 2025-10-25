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

#ifndef EVENT_CAMERA_CODECS__EVENT_PROCESSOR_H_
#define EVENT_CAMERA_CODECS__EVENT_PROCESSOR_H_

#include <cstddef>
#include <cstdint>

namespace event_camera_codecs
{
class EventProcessor
{
public:
  /**
   * \brief Constructor
   */
  virtual ~EventProcessor() {}

  /**
   * \brief invoked for a change-of-contrast event
   * \param sensor_time  time (usually in nanoseconds, depends on decoder configuration)
   * \param ex image x-coordinate of event
   * \param ey image y-coordinate (from top right)
   * \param polarity 0 if OFF event (intensity decreased), 1 if ON event (intensity increased)
   */
  virtual void eventCD(uint64_t sensor_time, uint16_t ex, uint16_t ey, uint8_t polarity) = 0;
  /**
   * \brief invoked for an external trigger event
   * \param sensor_time time (usually in nanoseconds, depends on decoder configuration)
   * \param edge indicator if rising or falling edge of trigger pulse
   * \param id identifier of trigger source (e.g. which pin the pulse arrived on)
   */
  virtual void eventExtTrigger(uint64_t sensor_time, uint8_t edge, uint8_t id) = 0;
  /**
   * \brief called after the processing of the packet has been completed, or
   *        the time limit (until_time) has been reached.
   */
  virtual void finished() = 0;
  /**
   * \brief passes raw data through from the decoder to the event processor.
   * This can be useful for .e.g. storing the raw data.
   * This method is called once for each processed packet, before eventCD() and
   * eventExtTrigger() are invoked. It is called *only* when messages are fed
   * into the decoder, *not* when raw data is already given to the decoder!
   * \param data pointer to buffer with raw data
   * \param len  size of buffer in bytes
   */

  virtual void rawData(const char * data, size_t len) = 0;
};
}  // namespace event_camera_codecs
#endif  // EVENT_CAMERA_CODECS__EVENT_PROCESSOR_H_
