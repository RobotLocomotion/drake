#include "drake/manipulation/util/trajectory_utils.h"

namespace drake {
namespace manipulation {

template class PiecewiseCartesianTrajectory<double>;
template class PiecewiseCubicTrajectory<double>;
template class SingleSegmentCartesianTrajectory<double>;

}  // namespace manipulation
}  // namespace drake
