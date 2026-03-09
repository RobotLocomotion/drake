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
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(IcfLinearFeedbackGains);
  IcfLinearFeedbackGains();
  ~IcfLinearFeedbackGains();

  /* Resizes this to store the given number of plant velocities. */
  void Resize(int num_velocities) {
    K.resize(num_velocities);
    b.resize(num_velocities);
  }

  VectorX<T> K;
  VectorX<T> b;
};

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    struct ::drake::multibody::contact_solvers::icf::internal::
        IcfLinearFeedbackGains);
