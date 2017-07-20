#include "drake/multibody/rigid_contact/rigid_contact_point.h"

#include <unsupported/Eigen/AutoDiff>

#include "drake/common/eigen_autodiff_types.h"

namespace drake {
namespace multibody {
namespace rigid_contact {

template struct RigidContactPoint<double>;
template struct RigidContactPoint<AutoDiffXd>;

}  // namespace rigid_contact
}  // namespace multibody
}  // namespace drake
