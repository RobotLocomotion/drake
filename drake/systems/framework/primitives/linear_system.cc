#include "drake/systems/framework/primitives/linear_system.h"

#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace systems {

template <typename T>
LinearSystem<T>::LinearSystem(const Eigen::Ref<const Eigen::MatrixXd>& A,
                              const Eigen::Ref<const Eigen::MatrixXd>& B,
                              const Eigen::Ref<const Eigen::MatrixXd>& C,
                              const Eigen::Ref<const Eigen::MatrixXd>& D)
    : AffineSystem<T>(A, B, Eigen::VectorXd::Zero(A.rows()), C, D,
                      Eigen::VectorXd::Zero(C.rows())) {}
// TODO(naveenoid): Modify constructor to accommodate 0 dimension systems;
// i.e. in initializing xDot0 and y0 with a zero matrix.
template class DRAKE_EXPORT LinearSystem<double>;
template class DRAKE_EXPORT LinearSystem<AutoDiffXd>;

}  // namespace systems
}  // namespace drake
