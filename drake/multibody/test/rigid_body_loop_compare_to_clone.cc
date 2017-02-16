#include "drake/multibody/test/rigid_body_loop_compare_to_clone.h"

#include <memory>

#include "drake/common/text_logging.h"

namespace drake {
namespace multibody {
namespace test {
namespace rigid_body_loop {

bool CompareToClone(const RigidBodyLoop<double>& original,
                  const RigidBodyLoop<double>& other) {
  if (!original.frameA_->CompareToClone(*other.frameA_)) {
    drake::log()->debug("CompareToClone(RigidBodyLoop): FrameA mismatch.");
    return false;
  }
  if (!original.frameB_->CompareToClone(*other.frameB_)) {
    drake::log()->debug("CompareToClone(RigidBodyLoop): FrameB mismatch.");
    return false;
  }
  if (original.axis_ != other.axis_) {
    drake::log()->debug(
        "CompareToClone(RigidBodyLoop): Axes mismatch:\n"
        "  - this: {}\n"
        "  - other: {}",
        original.axis_.transpose(), other.axis_.transpose());
    return false;
  }
  return true;
}

}  // namespace rigid_body_loop
}  // namespace test
}  // namespace multibody
}  // namespace drake
