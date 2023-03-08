# event_array_codecs

This repository holds code for decoding
[event_array_msgs](https://github.com/berndpfrommer/event_array_msgs). It
builds under both ROS1 and ROS2.
You can use this decoder from python via the
[event_array_py repository](https://github.com/berndpfrommer/event_array_py).


## Supported platforms

Currently tested on Ubuntu 20.04 under under ROS Noetic and ROS2 Galactic.


## How to build
Create a workspace (``~/ws``), clone this repo, and use ``wstool``
to pull in the remaining dependencies:

```
pkg=event_array_codecs
mkdir -p ~/$pkg/src
cd ~/ws
git clone https://github.com/berndpfrommer/${pkg}.git src/${pkg}
wstool init src src/${pkg}/${pkg}.rosinstall
# to update an existing space:
# wstool merge -t src src/${pkg}/${pkg}.rosinstall
# wstool update -t src
```

### configure and build on ROS1:

```
catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo  # (optionally add -DCMAKE_EXPORT_COMPILE_COMMANDS=1)
catkin build
```

### configure and build on ROS2:

```
cd ~/ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo  # (optionally add -DCMAKE_EXPORT_COMPILE_COMMANDS=1)
```

## API example

```
#include <event_array_codecs/decoder.h>
#include <event_array_codecs/decoder_factory.h>

class MyProcessor : public event_array_codecs::EventProcessor
{
public:
  inline void eventCD(uint64_t, uint16_t ex, uint16_t ey, uint8_t polarity) override {
    // do useful stuff here
  }
  void eventExtTrigger(uint64_t, uint8_t, uint8_t) override {}
  void finished() override{}; // called after no more events decoded in this packet
  void rawData(const char *, size_t) override{};  // passthrough of raw data
};

MyProcessor processor;

// the decoder factory method is templated on the event processor
// to permit inlining of methods like eventCD() above.

event_array_codecs::DecoderFactory<MyProcessor> decoderFactory;

// to get callbacks into MyProcessor, feed the message buffer
// into the decoder

void eventMsg(const EventArray::ConstPtr & msg) {
  // will create a new decoder on first call, from then on returns existing one
  auto decoder = decoderFactory.getInstance(msg->encoding, msg->width, msg->height);
  if (!decoder) { // msg->encoding was invalid
    return;
  }
  decoder->setTimeBase(msg->time_base); // may not be needed for some encodings but doesn't hurt
  // the decode() will trigger callbacks to processor
  decoder->decode(&(msg->events[0]), msg->events.size(), &processor);
}
```

## Tools

### Performance measurement of decoder

ROS1:
```
# decode a raw file and measure performance
rosrun event_array_codecs perf -i foo.raw
```

ROS2 examples:
```
# decode a raw file and measure performance
ros2 run event_array_codecs perf -i foo.raw
```

## License

This software is issued under the Apache License Version 2.0.
