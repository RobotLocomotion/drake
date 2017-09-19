#include "drake/automotive/simple_powertrain.h"

#include "drake/common/symbolic.h"

namespace drake {
namespace automotive {

// These instantiations must match the API documentation in
// simple_powertrain.h.
template class SimplePowertrain<double>;
template class SimplePowertrain<AutoDiffXd>;
template class SimplePowertrain<symbolic::Expression>;

}  // namespace automotive
}  // namespace drake
