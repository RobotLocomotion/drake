#include "drake/common/constants.h"
#include "drake/common/random.h"
#include "drake/common/symbolic.h"
#include "drake/geometry/render/render_engine.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcm/drake_lcm_interface.h"
#include "drake/lcm/drake_mock_lcm.h"
#include "drake/math/autodiff.h"
#include "drake/math/barycentric.h"
#include "drake/math/rotation_matrix.h"
#include "drake/perception/depth_image_to_point_cloud.h"
#include "drake/perception/point_cloud.h"
#include "drake/perception/point_cloud_flags.h"
#include "drake/systems/framework/framework_common.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/system.h"
#include "drake/systems/framework/system_base.h"

template class drake::math::RotationMatrix<double>;
template class drake::math::RotationMatrix<drake::AutoDiffXd>;
template class drake::math::RollPitchYaw<double>;
template class drake::math::RollPitchYaw<drake::AutoDiffXd>;
template class drake::math::BarycentricMesh<double>;
template class drake::systems::LeafSystem<double>;
template class drake::systems::LeafSystem<drake::AutoDiffXd>;
template class drake::systems::LeafSystem<float>;
template class drake::systems::System<double>;
template class drake::systems::System<drake::AutoDiffXd>;
