#include "drake/automotive/idm_with_trajectory_agent-inl.h"

#include "drake/common/eigen_autodiff_types.h"

namespace drake {
namespace cars {

// These instantiations must match the API documentation in
// idm_with_trajectory_agent.h.
template class DRAKECARS_EXPORT IdmWithTrajectoryAgent<double>;
template class DRAKECARS_EXPORT IdmWithTrajectoryAgent<drake::TaylorVarXd>;

}  // namespace cars
}  // namespace drake
