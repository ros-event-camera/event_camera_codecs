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

#ifndef EVENT_CAMERA_CODECS__ROS1_ROS2_COMPAT_H_
#define EVENT_CAMERA_CODECS__ROS1_ROS2_COMPAT_H_

#ifdef USING_ROS_1
#define MSG_CONST_SHARED_PTR ConstPtr
#else
#define MSG_CONST_SHARED_PTR ConstSharedPtr
#endif

#endif  // EVENT_CAMERA_CODECS__ROS1_ROS2_COMPAT_H_
