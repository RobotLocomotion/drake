#include "drake/geometry/frame_kinematics_vector.h"

#include "drake/common/default_scalars.h"

namespace drake {
namespace geometry {

template class KinematicsVector<FrameId, math::RigidTransform<double>>;
template class KinematicsVector<FrameId, math::RigidTransform<AutoDiffXd>>;
template class KinematicsVector<FrameId,
                                math::RigidTransform<symbolic::Expression>>;

}  // namespace geometry
}  // namespace drake
