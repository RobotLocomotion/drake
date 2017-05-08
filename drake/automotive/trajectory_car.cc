#include "drake/automotive/trajectory_car.h"

// #include "drake/common/autodiff_overloads.h"
#include "drake/common/eigen_autodiff_types.h"

namespace drake {
namespace automotive {

template class TrajectoryCar<double>;
template class TrajectoryCar<AutoDiffXd>;

}  // namespace automotive
}  // namespace drake
