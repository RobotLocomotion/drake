#include "drake/automotive/idm_with_trajectory_agent-inl.h"

#include "drake/common/drake_export.h"
#include "drake/common/symbolic_expression.h"

namespace drake {
namespace automotive {

// This instantiation must match the API documentation in
// idm_with_trajectory_agent.h.
template class DRAKE_EXPORT IdmWithTrajectoryAgent<drake::symbolic::Expression>;

}  // namespace automotive
}  // namespace drake
