#include "drake/manipulation/util/trajectory_utils.h"

namespace drake {
namespace manipulation {
namespace util {

template
class PiecewiseCartesianTrajectory<double>;
template
class PiecewiseCubicTrajectory<double>;

}  // namespace util
}  // namespace manipulation
}  // namespace drake
