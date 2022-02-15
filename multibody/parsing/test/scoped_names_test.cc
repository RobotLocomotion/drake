// Copyright 2022 Open Source Robotics Foundation, Inc.
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

#include "drake/multibody/parsing/scoped_names.h"

#include <gtest/gtest.h>

#include "drake/common/filesystem.h"
#include "drake/common/find_resource.h"
#include "drake/multibody/parsing/detail_sdf_parser.h"

namespace drake {
namespace multibody {
namespace parsing {
namespace {

using internal::DataSource;

const char* const kTestDir = "drake/multibody/parsing/test";

GTEST_TEST(ScopedNamesTest, GetScopedFrameByName) {
  const std::string full_name =
      FindResourceOrThrow(std::string(kTestDir) + "/scoped_names_model.sdf");

  MultibodyPlant<double> plant(0.0);
  PackageMap package_map;

  const DataSource data_source{&full_name, {}};
  AddModelFromSdf(data_source, "scoped_names_model", package_map, &plant);
  plant.Finalize();

  ASSERT_EQ(
      &GetScopedFrameByName(plant, "scoped_names_model::frame"),
      &plant.GetFrameByName(
          "frame", plant.GetModelInstanceByName("scoped_names_model")));

  ASSERT_EQ(
      &GetScopedFrameByName(
          plant, "scoped_names_model::inner_model::inner_frame"),
      &plant.GetFrameByName(
          "inner_frame",
          plant.GetModelInstanceByName("scoped_names_model::inner_model")));
}

}  // namespace
}  // namespace parsing
}  // namespace multibody
}  // namespace drake
