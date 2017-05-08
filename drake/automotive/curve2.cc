#include "drake/automotive/curve2.h"

#include "drake/common/eigen_autodiff_types.h"

namespace drake {
namespace automotive {

template class Curve2<double>;
template class Curve2<AutoDiffXd>;

}  // namespace automotive
}  // namespace drake
