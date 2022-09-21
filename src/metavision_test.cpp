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

#include <metavision/hal/device/device.h>
#include <metavision/hal/device/device_discovery.h>
#include <metavision/hal/facilities/i_decoder.h>
#include <metavision/hal/facilities/i_event_decoder.h>
#include <metavision/hal/facilities/i_events_stream.h>
#include <metavision/sdk/base/events/event_cd.h>
#include <unistd.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <vector>

#include "event_array_codecs/decoder.h"
#include "event_array_codecs/event_processor.h"

void usage()
{
  std::cout << "usage:" << std::endl;
  std::cout << "metavision_test -i name_of_raw_file -c codec (e.g. evt3)" << std::endl;
}

struct EventCounter : public event_array_codecs::EventProcessor
{
  void process_events(const Metavision::EventCD * begin, const Metavision::EventCD * end)
  {
    for (auto e = begin; e < end; e++) {
      lastMVTime = e->t;
    }
  }

  void eventCD(uint64_t t, uint16_t, uint16_t, uint8_t)
  {
    if (t != lastMVTime) {
      std::cout << " ERROR: " << (t - lastMVTime) << std::endl;
    }
    numEvents++;
  }
  void eventExtTrigger(uint64_t t, uint8_t, uint8_t)
  {
    if (t < lastTime || t - lastTime > 1) {
      if (t < lastTime) {
        std::cout << "time goes back: " << lastTime << " now: " << t << std::endl;
      } else {
        std::cout << "time jump: " << lastTime << " now: " << t << " dt: " << (t - lastTime)
                  << std::endl;
      }
    }
    lastTime = t;
    numEvents++;
  }
  void finished() {}
  void rawData(const char *, size_t) {}

  size_t numEvents{0};
  uint64_t lastTime{0};
  uint64_t lastMVTime{0};
};

int main(int argc, char ** argv)
{
  int opt;

  std::string inFile;
  std::string codec("evt3");
  while ((opt = getopt(argc, argv, "i:c:h")) != -1) {
    switch (opt) {
      case 'i':
        inFile = optarg;
        break;
      case 'c':
        codec = optarg;
        break;
      case 'h':
      default:
        std::cout << "unknown option: " << opt << std::endl;
        usage();
        return (-1);
        break;
    }
  }
  if (inFile.empty()) {
    std::cout << "missing input file name!" << std::endl;
    usage();
    return (-1);
  }
  EventCounter counter;

  Metavision::RawFileConfig cfg;
  cfg.n_events_to_read_ = 1;      // one event at a time
  cfg.n_read_buffers_ = 3;        // must leave at 3
  cfg.do_time_shifting_ = false;  // no mucking with timestamps

  std::cout << "opening file: " << inFile << std::endl;
  std::unique_ptr<Metavision::Device> device;
  try {
    device = Metavision::DeviceDiscovery::open_raw_file(inFile, cfg);
  } catch (Metavision::HalException & e) {
    std::cout << "cannot open file: " << e.what() << std::endl;
    return (-1);
  }

  if (!device) {
    std::cerr << "file opening failed." << std::endl;
    return (-1);
  }

  Metavision::I_EventDecoder<Metavision::EventCD> * mv_decoder =
    device->get_facility<Metavision::I_EventDecoder<Metavision::EventCD>>();
  if (mv_decoder) {
    // Register a lambda function to be called on every CD events
    mv_decoder->add_event_buffer_callback(
      [&counter](const Metavision::EventCD * begin, const Metavision::EventCD * end) {
        counter.process_events(begin, end);
      });
  } else {
    std::cerr << "cannot open decoder!" << std::endl;
  }
  Metavision::I_EventsStream * i_eventsstream = device->get_facility<Metavision::I_EventsStream>();
  Metavision::I_Decoder * i_decoder = device->get_facility<Metavision::I_Decoder>();
  bool stop_decoding = false;

  i_eventsstream->start();
  auto decoder = event_array_codecs::Decoder::newInstance(codec);
  if (!decoder) {
    std::cerr << "unknown codec: " << codec << std::endl;
  }

  while (!stop_decoding) {
    uint16_t ret = i_eventsstream->poll_buffer();
    if (ret < 0) {
      std::cout << "End of file" << std::endl;
      i_eventsstream->stop();
      i_eventsstream->stop_log_raw_data();
      stop_decoding = true;
    } else if (ret == 0) {
      continue;
    }
    int64_t n_bytes;
    uint8_t * raw_data = i_eventsstream->get_latest_raw_data(n_bytes);
    // call both decoders, the "counter" object will compare the time stamps
    i_decoder->decode(raw_data, raw_data + n_bytes);
    decoder->decode(raw_data, n_bytes, &counter);
  }
  return (0);
}
