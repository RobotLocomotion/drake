#include "drake/systems/analysis/simulator.h"

#include "drake/common/eigen_autodiff_types.h"
#include "drake/drakeSystemAnalysis_export.h"

namespace drake {
namespace systems {

template class DRAKESYSTEMANALYSIS_EXPORT Simulator<double>;
template class DRAKESYSTEMANALYSIS_EXPORT Simulator<AutoDiffXd>;

}  // namespace systems
}  // namespace drake
