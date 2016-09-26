#include "./time_varying_polynomial_source.h"

#include <stdexcept>
#include <string>

#include "drake/common/drake_assert.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace systems {

template <typename T>
TimeVaryingPolynomialSource<T>::TimeVaryingPolynomialSource(
    const PiecewisePolynomial<double>& pp_traj)
    : kPpTraj(pp_traj) {
  this->DeclareOutputPort(kVectorValued, pp_traj.rows(), kContinuousSampling);
}

template <typename T>
void TimeVaryingPolynomialSource<T>::EvalOutput(const Context<T>& context,
                                                SystemOutput<T>* output) const {
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));
  T time = context.get_time();
  System<T>::GetMutableOutputVector(output, 0) = kPpTraj.value(time);
}

// Explicitly instantiates on the most common scalar types.
template class DRAKESYSTEMFRAMEWORK_EXPORT TimeVaryingPolynomialSource<double>;

}  // namespace systems
}  // namespace drake
