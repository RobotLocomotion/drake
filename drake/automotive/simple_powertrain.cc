#include "drake/automotive/simple_powertrain.h"

#include "drake/common/symbolic_formula.h"

namespace drake {
namespace automotive {

// These instantiations must match the API documentation in
// simple_powertrain.h.
template class SimplePowertrain<double>;
template class SimplePowertrain<AutoDiffXd>;

}  // namespace automotive
}  // namespace drake
