#include "drake/automotive/simple_power_train.h"

#include "drake/common/symbolic_formula.h"

namespace drake {
namespace automotive {

// These instantiations must match the API documentation in
// simple_power_train.h.
template class SimplePowerTrain<double>;
template class SimplePowerTrain<AutoDiffXd>;

}  // namespace automotive
}  // namespace drake
