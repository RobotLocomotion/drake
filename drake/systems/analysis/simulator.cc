#include "drake/systems/analysis/simulator.h"

#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/drake_export.h"

namespace drake {
namespace systems {

template class DRAKE_EXPORT Simulator<double>;
template class DRAKE_EXPORT Simulator<AutoDiffXd>;

}  // namespace systems
}  // namespace drake
