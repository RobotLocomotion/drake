// NOLINTNEXTLINE(build/include) False positive on inl file.
#include "drake/examples/Painleve/painleve-inl.h"

#include "drake/common/eigen_autodiff_types.h"

namespace drake {
namespace painleve {

template class Painleve<double>;
// template class Painleve<AutoDiffXd>;

}  // namespace painleve
}  // namespace drake
