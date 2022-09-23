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

#include "event_array_codecs/encoder.h"

#include "event_array_codecs/evt3_encoder.h"
#include "event_array_codecs/mono_encoder.h"

namespace event_array_codecs
{
std::shared_ptr<Encoder> Encoder::newInstance(const std::string & codec)
{
  if (codec == "mono" || codec == "trigger") {
    return (std::make_shared<mono::Encoder>());
  }
  return (std::shared_ptr<Encoder>());
}
}  // namespace event_array_codecs
