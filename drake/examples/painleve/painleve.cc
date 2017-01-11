// NOLINTNEXTLINE(build/include) False positive on inl file.
#include "drake/examples/painleve/painleve-inl.h"

namespace drake {
namespace painleve {

template class Painleve<double>;
template class Painleve<Eigen::AutoDiffScalar<drake::Vector1d>>;

}  // namespace painleve
}  // namespace drake
