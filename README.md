# event_array_codecs

This repository holds code for decoding
[event_array_msgs](https://github.com/berndpfrommer/event_array_msgs). It
builds under both ROS1 and ROS2.

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
