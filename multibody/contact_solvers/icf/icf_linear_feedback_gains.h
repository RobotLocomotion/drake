#pragma once

#include "drake/common/default_scalars.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {

/* Represents a linear feedback law of the form
    τ = −K⋅v + b,
where K is non-negative and diagonal.
@tparam_nonsymbolic_scalar */
template <typename T>
struct IcfLinearFeedbackGains {
  VectorX<T> K;
  VectorX<T> b;

  void resize(int nv) {
    K.resize(nv);
    b.resize(nv);
  }
};

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::contact_solvers::icf::internal::
        IcfLinearFeedbackGains);
