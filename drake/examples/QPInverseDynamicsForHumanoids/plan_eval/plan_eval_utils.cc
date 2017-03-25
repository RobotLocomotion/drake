#include "drake/examples/QPInverseDynamicsForHumanoids/plan_eval/plan_eval_utils.h"

namespace drake {
namespace examples {
namespace qp_inverse_dynamics {

template class PiecewiseCartesianTrajectory<double>;
template class PiecewiseCubicTrajectory<double>;

}  // namespace qp_inverse_dynamics
}  // namespace examples
}  // namespace drake
