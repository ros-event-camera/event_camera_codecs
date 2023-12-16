# event_camera_codecs

This repository holds code for decoding
[event_camera_msgs](https://github.com/ros-event-camera/event_camera_msgs). It
builds under both ROS1 and ROS2.
You can use this decoder from python via the
[event_camera_py repository](https://github.com/ros-event-camera/event_camera_py).


## Supported platforms

Currently tested on:
- ROS1: Ubuntu 20.04 on Noetic
- ROS2: Ubuntu 22.04 on Humble, Iron, Rolling


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

To also build the unit tests, add the cmake argument ``-DEVENT_CAMERA_CODECS_BUILD_TESTS=ON``.

## API example

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
// into the decoder like so

void eventMsg(const event_camera_codecs::EventPacketConstSharedPtr & msg) {
  // will create a new decoder on first call, from then on returns existing one
  auto decoder = decoderFactory.getInstance(*msg);
  if (!decoder) { // msg->encoding was invalid
    return;
  }
  // the decode() will trigger callbacks to processor
  decoder->decode(*msg, &processor);
}

/* To synchronize with frame based sensors it is useful to play back
   until a frame boundary is reached. The interface decodeUntil() is provided
   for this purpose. Do *NOT* use decode() and decodeUntil() on the same decoder!
   In the sample code belowframeTimes is an ordered queue of frame times.
   */

void eventMsg2(const event_camera_codecs::EventPacketConstSharedPtr & msg) {
  auto decoder = decoderFactory.getInstance(*msg);
  uint64_t nextTime{0};
  // The loop will exit when all events in msg have been processed
  // or there are no more frame times available
  decoder->setTimeBase(msg->time_base);
  while (!frameTimes.empty() &&
    decoder->decodeUntil(*msg, &processor, frameTimes.front(), &nextTime)) {
    // use loop in case multiple frames fit inbetween two events
    while (!frameTimes.empty() && frameTimes.front() <= nextTime) {
      // processFrameHere()
      frameTimes.pop();
    }
  }
}
```

## Codecs

The following encodings are supported:
- ``evt3``: Native EVT3 format of the Prophesee (Metavision) line of cameras
- ``libcaer_cmp``: Compressed libcaer format for Inivation Labs line of cameras.
- ``libcaer``: Uncompressed format of libcaer data. Use only when memory bandwidth
   is high and CPU is slow.
- ``mono``: legacy format temporarily used for prophesee cameras. Deprecated.
- ``trigger``: legacy format temporarily used for trigger events. Deprecated.

## Event Time Stamps

All event time stamps returned by the decoder refer to *sensor time*, i.e. the
clock that is built into the device. The *sensor time* is given in nanoseconds
although at the moment no device supports accuracy higher than microseconds.
For certain devices (e.g. libcaer) *sensor time*
is aligned with *host time* when the driver starts up. Since *sensor time* and *host time*
are not synchronized they can drift arbitrarily far from each other.
Some decoders (e.g. ``libcaer``, ``libcaer_cmp``) make use of the ``time_base``
field, which refers to sensor time at the time of arrival of the first data packet
included in the ROS message. For other encodings (e.g. ``evt3``) the sensor time must
be recovered by decoding the messages.

The time stamps encoded in EVT3 format have a few peculiarities worthwhile noting:

- the data field for the time stamps is only 24 bits wide, which leads to wrap-around
  of the time stamps every 2^24usec = 16.77s. This means that if you start decoding
  somewhere in the middle of the data stream (an hour into it!), the first time stamp
  you will get is always between 0 and 16.77s.
  From then on the decoder library keeps track of the
  moment when the 24 bit time stamp wraps around and so gives you the illusion that
  the time stamp is 64bit wide. In reality it is not.
- the device has some dubious bit errors in the time stamps that thwart a simplistic
  detection of the time stamp wrap-around. The decoder has some logic built into it
  to compensate for those errors but if your time stamps are suddenly off by 16.77s,
  that may well have been the problem.
- in theory comparison with the time stamps obtained from the Metavision SDK should be
  possible but especially the starting time usually disagrees for unknown reasons. The
  relative time differences however have always checked out so far.
- because of the time-stamp wrap around, decoding two separate EVT3 event streams from
  two different cameras can be tricky. If the time stamps for one camera have just wrapped
  around before the first packet is decoded, but have not wrapped around for the second
  camera, the sensor times will be off by 16.7s with respect to each other although the
  two cameras are hardware synced.


## Tools

### Performance measurement of EVT3 decoder (for developers only)

ROS1:
```
# decode a raw file and measure performance
rosrun event_camera_codecs codec_perf -i foo.raw
```

ROS2 examples:
```
# decode a raw file and measure performance
ros2 run event_camera_codecs codec_perf -i foo.raw
```

## License

This software is issued under the Apache License Version 2.0.
