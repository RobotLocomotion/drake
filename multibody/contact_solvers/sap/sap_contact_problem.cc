#include "drake/multibody/contact_solvers/sap/sap_contact_problem.h"

#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

template <typename T>
SapContactProblem<T>::SapContactProblem(const T& time_step,
                                        std::vector<MatrixX<T>> A,
                                        VectorX<T> v_star)
    : time_step_(time_step), A_(std::move(A)), v_star_(std::move(v_star)) {
  DRAKE_THROW_UNLESS(time_step > 0.0);
  nv_ = 0;
  for (const auto& Ac : A_) {
    DRAKE_THROW_UNLESS(Ac.size() > 0);
    DRAKE_THROW_UNLESS(Ac.rows() == Ac.cols());
    nv_ += Ac.rows();
  }
  DRAKE_THROW_UNLESS(v_star_.size() == nv_);
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::internal::SapContactProblem)
