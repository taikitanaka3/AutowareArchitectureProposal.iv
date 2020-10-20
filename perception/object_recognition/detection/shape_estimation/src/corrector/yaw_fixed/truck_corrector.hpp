/*
 * Copyright 2020 TierIV. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include "shape_estimation/corrector_interface.hpp"

namespace yaw_fixed {
class TruckCorrector : public ShapeEstimationCorrectorInterface
{
public:
  TruckCorrector(){};

  ~TruckCorrector(){};

  bool correct(
    autoware_perception_msgs::msg::Shape & shape_output, geometry_msgs::msg::Pose & pose_output) override;
};
}
