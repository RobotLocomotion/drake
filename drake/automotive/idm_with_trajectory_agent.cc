#include "drake/automotive/idm_with_trajectory_agent-inl.h"

#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/drake_export.h"

namespace drake {
namespace automotive {

// These instantiations must match the API documentation in
// idm_with_trajectory_agent.h.
template class DRAKE_EXPORT IdmWithTrajectoryAgent<double>;
template class DRAKE_EXPORT
IdmWithTrajectoryAgent<drake::TaylorVarXd>;

}  // namespace automotive
}  // namespace drake
