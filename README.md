# event_camera_codecs

This repository holds code for decoding
[event_camera_msgs](https://github.com/ros-event-camera/event_camera_msgs). It
builds under both ROS1 and ROS2.
You can use this decoder from python via the
[event_camera_py repository](https://github.com/ros-event-camera/event_camera_py).


## Supported platforms

Currently tested on Ubuntu 20.04 under ROS Noetic and ROS2 Galactic
and under Ubuntu 22.04 / ROS2 Humble.


## How to build
Create a workspace, clone this repo, and use ``vcs``
to pull in the remaining dependencies:

```
pkg=event_camera_codecs
mkdir -p ~/$pkg/src
cd ~/ws
git clone https://github.com/ros-event-camera/${pkg}.git src/${pkg}
cd src
vcs import < ${pkg}/${pkg}.repos
cd ..
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

## API example (ROS2)

```cpp
#include <event_camera_codecs/decoder.h>
#include <event_camera_codecs/decoder_factory.h>

using event_camera_codecs::EventPacket;

class MyProcessor : public event_camera_codecs::EventProcessor
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

event_camera_codecs::DecoderFactory<EventPacket, MyProcessor> decoderFactory;

// to get callbacks into MyProcessor, feed the message buffer
// into the decoder

void eventMsg(const EventPacket::ConstSharedPtr & msg) {
  // will create a new decoder on first call, from then on returns existing one
  auto decoder = decoderFactory.getInstance(msg->encoding, msg->width, msg->height);
  if (!decoder) { // msg->encoding was invalid
    return;
  }
  decoder->setTimeBase(msg->time_base); // may not be needed for some encodings but doesn't hurt
  // the decode() will trigger callbacks to processor
  decoder->decode(msg->events.data(), msg->events.size(), &processor);
  /* There is an alternative interface for decoding only up to (but not including)
     a certain time stamp:
  uint64_t timeLimit{whatever}; // (input) time limit, sensor time, in nanoseconds
  uint64_t nextTime{0}; // (output) will contain time stamp (if any) of yet undecoded event
  const size_t bytesDecoded = decoder->decodeUntil(
    msg->events.data(), msg->events.size(), &processor, timeLimit, &nextTime);
  */
}
```

## Tools

### Performance measurement of decoder

ROS1:
```
# decode a raw file and measure performance
rosrun event_camera_codecs perf -i foo.raw
```

ROS2 examples:
```
# decode a raw file and measure performance
ros2 run event_camera_codecs perf -i foo.raw
```

## License

This software is issued under the Apache License Version 2.0.
