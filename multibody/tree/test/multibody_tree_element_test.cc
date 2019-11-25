#include "drake/multibody/tree/multibody_tree_element.h"

#include <utility>

#include "drake/multibody/tree/joint.h"

namespace drake {
namespace multibody {

// Make sure we can access the deprecated alias and that it's identical to the
// referent.

using A = MultibodyElement<Joint, double, JointIndex>;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
using B = MultibodyTreeElement<Joint<double>, JointIndex>;
#pragma GCC diagnostic pop

static_assert(std::is_same<A, B>::value, "Yay");

}  // namespace multibody
}  // namespace drake
