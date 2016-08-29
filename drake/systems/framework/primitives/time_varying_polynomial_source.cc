/// @file
/// Template method implementations for constant_value_source.h.
/// Most users should only include that file, not this one.
/// For background, see http://drake.mit.edu/cxx_inl.html.

#include "./time_varying_polynomial_source.h"

#include <stdexcept>
#include <string>

#include "drake/common/drake_assert.h"
#include "drake/systems/framework/context.h"

#include <iostream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;

namespace drake {
namespace systems {

template <typename T>
TimeVaryingPolynomialSource<T>::TimeVaryingPolynomialSource(
    const PiecewisePolynomial<double>& pp_traj)
    : pp_traj_(pp_traj) {
//  this->DeclareAbstractOutputPort(kInheritedSampling);
//  this->DeclareInputPort(kVectorValued, length, kContinuousSampling);
  this->DeclareOutputPort(kVectorValued, pp_traj.rows(), kContinuousSampling);
}

//template <typename T>
//std::unique_ptr<SystemOutput<T>> TimeVaryingPolynomialSource<T>::AllocateOutput(
//    const ContextBase<T>& context) const {
//  std::unique_ptr<LeafSystemOutput<T>> output(new LeafSystemOutput<T>);
//  output->add_port(source_value_->Clone());
//  return std::unique_ptr<SystemOutput<T>>(output.release());
//}

template <typename T>
void TimeVaryingPolynomialSource<T>::EvalOutput(const ContextBase<T>& context,
                                        SystemOutput<T>* output) const {
  DRAKE_ASSERT(output->get_num_ports() == 1);
  T time = context.get_time();
  System<T>::GetMutableOutputVector(output, 0) = pp_traj_.value(time);
}


// Explicitly instantiates on the most common scalar types.
template class DRAKESYSTEMFRAMEWORK_EXPORT TimeVaryingPolynomialSource<double>;

}  // namespace systems
}  // namespace drake
