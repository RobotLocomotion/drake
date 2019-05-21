#include "drake/automotive/maliput/multilane/test_utilities/fixtures.h"

#include <limits>
#include <string>

#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/multilane/builder.h"
#include "drake/automotive/maliput/multilane/loader.h"

using drake::maliput::api::LaneId;
using drake::maliput::multilane::BuilderFactory;
using drake::maliput::multilane::LoadFile;

namespace drake {
namespace maliput {
namespace multilane {

BranchAndMergeBasedTest::BranchAndMergeBasedTest()
    : road_geometry_(
          LoadFile(BuilderFactory(),
                   "automotive/maliput/multilane/branch_and_merge.yaml")),
      index_(road_geometry_->ById()),
      total_length_(index_.GetLane(LaneId("l:1.1_0"))->length() +
                    index_.GetLane(LaneId("l:1.2_0"))->length() +
                    index_.GetLane(LaneId("l:1.3_0"))->length()) {}

LoopBasedTest::LoopBasedTest()
    : road_geometry_(
          LoadFile(BuilderFactory(), "automotive/maliput/multilane/loop.yaml")),
      index_(road_geometry_->ById()) {}

MultiBranchBasedTest::MultiBranchBasedTest()
    : road_geometry_(LoadFile(
          BuilderFactory(), "automotive/maliput/multilane/multi_branch.yaml")),
      index_(road_geometry_->ById()) {}

}  // namespace multilane
}  // namespace maliput
}  // namespace drake
