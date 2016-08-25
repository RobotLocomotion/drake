#include "drake/examples/Cars/idm_with_trajectory_agent-inl.h"

#include "drake/common/eigen_autodiff_types.h"

namespace drake {

// Explicitly instantiate all ScalarType-using definitions.
#define DRAKE_INSTANTIATE(ScalarType)                                   \
template class DRAKECARS_EXPORT IdmWithTrajectoryAgent<ScalarType>;

// These instantiations must match the API documentation in
// idm_with_trajectory_agent.h.
DRAKE_INSTANTIATE(double)
DRAKE_INSTANTIATE(drake::TaylorVarXd)

#undef DRAKE_INSTANTIATE

}  // namespace drake
