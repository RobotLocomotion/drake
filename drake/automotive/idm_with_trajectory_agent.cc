// NOLINTNEXTLINE(build/include) False positive on inl file.
#include "drake/automotive/idm_with_trajectory_agent-inl.h"

#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/drake_export.h"
#include "drake/common/symbolic_expression.h"
#include "drake/math/autodiff_overloads.h"

namespace drake {
namespace automotive {

// These instantiations must match the API documentation in
// idm_with_trajectory_agent.h.
template class DRAKE_EXPORT IdmWithTrajectoryAgent<double>;
template class DRAKE_EXPORT IdmWithTrajectoryAgent<drake::TaylorVarXd>;
template class DRAKE_EXPORT IdmWithTrajectoryAgent<drake::symbolic::Expression>;

}  // namespace automotive
}  // namespace drake
