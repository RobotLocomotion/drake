#include "drake/multibody/contact_solvers/icf/icf_linear_feedback_gains.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace internal {

template <typename T>
IcfLinearFeedbackGains<T>::IcfLinearFeedbackGains() = default;

template <typename T>
IcfLinearFeedbackGains<T>::~IcfLinearFeedbackGains() = default;

}  // namespace internal
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    struct ::drake::multibody::contact_solvers::icf::internal::
        IcfLinearFeedbackGains);
