#pragma once

#include <memory>

#include <gtest/gtest.h>

#include "drake/automotive/maliput/api/road_geometry.h"

namespace drake {
namespace maliput {
namespace multilane {

/// A fixture for tests that use a road that branches and merges.
class BranchAndMergeBasedTest : public ::testing::Test {
 protected:
  BranchAndMergeBasedTest();

  std::unique_ptr<const drake::maliput::api::RoadGeometry> road_geometry_;
  const drake::maliput::api::RoadGeometry::IdIndex& index_;
  const double total_length_;
};

/// A fixture for tests that use a road with multiple loops.
class LoopBasedTest : public ::testing::Test {
 protected:
  LoopBasedTest();

  std::unique_ptr<const drake::maliput::api::RoadGeometry> road_geometry_;
  const drake::maliput::api::RoadGeometry::IdIndex& index_;
};

/// A fixture for tests that use a road that branches from one lane into
/// multiple lanes.
class MultiBranchBasedTest : public ::testing::Test {
 protected:
  MultiBranchBasedTest();

  std::unique_ptr<const drake::maliput::api::RoadGeometry> road_geometry_;
  const drake::maliput::api::RoadGeometry::IdIndex& index_;
};

}  // namespace multilane
}  // namespace maliput
}  // namespace drake
