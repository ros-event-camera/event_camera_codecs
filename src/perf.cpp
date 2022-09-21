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
  std::cout << "perf -c codec -i name_of_raw_file -b buffer_size" << std::endl;
}

// skip header
static void skip_header(std::fstream & in)
{
  int c;
  while ((c = in.peek()) == '%') {
    std::string line;
    std::getline(in, line);
  }
}

struct EventCounter : public event_array_codecs::EventProcessor
{
  void eventCD(uint64_t t, uint16_t, uint16_t, uint8_t)
  {
#ifdef DEBUG
    if (t < lastTime || t - lastTime > 1) {
      if (t < lastTime) {
        std::cout << "time goes back: " << lastTime << " now: " << t << " dt: " << (lastTime - t)
                  << std::endl;
      } else {
        std::cout << "time jump: " << lastTime << " now: " << t << " dt: " << (t - lastTime)
                  << std::endl;
      }
    }
#endif
    lastTime = t;
    numEvents++;
  }

  void eventExtTrigger(uint64_t t, uint8_t, uint8_t)
  {
#ifdef DEBUG
    if (t < lastTime || t - lastTime > 1) {
      if (t < lastTime) {
        std::cout << "time goes back: " << lastTime << " now: " << t << std::endl;
      } else {
        std::cout << "time jump: " << lastTime << " now: " << t << " dt: " << (t - lastTime)
                  << std::endl;
      }
    }
#endif
    lastTime = t;
    numEvents++;
  }
  void finished() {}
  void rawData(const char *, size_t) {}
  // -------------------- variables --------
  size_t numEvents{0};
  uint64_t lastTime{0};
};

int main(int argc, char ** argv)
{
  int opt;

  std::string inFile;
  std::string codec("evt3");
  int bufferSize(2048);
  while ((opt = getopt(argc, argv, "i:c:b:")) != -1) {
    switch (opt) {
      case 'i':
        inFile = optarg;
        break;
      case 'c':
        codec = optarg;
        break;
      case 'b':
        bufferSize = atoi(optarg);
        break;
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
  std::fstream in;
  in.open(inFile, std::ios::in | std::ios::binary);
  skip_header(in);

  std::vector<char> buffer(bufferSize);
  EventCounter counter;
  auto decoder = event_array_codecs::Decoder::newInstance(codec);
  if (!decoder) {
    std::cout << "unknown codec: " << codec << std::endl;
  } else {
    auto start = std::chrono::high_resolution_clock::now();
    while (in.read(&buffer[0], sizeof(buffer))) {
      decoder->decode(reinterpret_cast<uint8_t *>(&buffer[0]), in.gcount(), &counter);
    }
    auto final = std::chrono::high_resolution_clock::now();
    auto total_duration = std::chrono::duration_cast<std::chrono::microseconds>(final - start);
    std::cout << "number of events read: " << counter.numEvents * 1e-6 << " Mev in "
              << total_duration.count() * 1e-6 << " seconds" << std::endl;
    std::cout << "event rate: " << static_cast<double>(counter.numEvents) / total_duration.count()
              << " Mevs" << std::endl;
  }
  return (0);
}
