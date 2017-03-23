///
/// @file   degenerate_euler_joint.cc
///
/// @brief  DegenerateEulerJoint System implementation, plus
/// instantiation with commonly used scalar types.
///

#include "drake/examples/particles/degenerate_euler_joint.h"

namespace drake {
namespace examples {
namespace particles {

template <typename T>
DegenerateEulerJoint<T>::DegenerateEulerJoint(const MatrixX<T>& translator)
    : translator_(translator) {
  // Output degrees of freedom must be 6.
  DRAKE_DEMAND(this->translator_.rows() == 6);
  // Cannot have less than 1 input degree of freedom!
  DRAKE_DEMAND(this->translator_.cols() > 0);
  // Cannot have more than 5 input degrees of freedom!
  DRAKE_DEMAND(this->translator_.cols() < 6);
  // Twice the degrees of freedom to allocate for positions plus velocities.
  this->DeclareInputPort(systems::kVectorValued, 2 * this->translator_.cols());
  this->DeclareOutputPort(systems::kVectorValued, 2 * this->translator_.rows());
}

template <typename T>
void DegenerateEulerJoint<T>::DoCalcOutput(
  const systems::Context<T>& context,
  systems::SystemOutput<T>* output) const {
  // Get current input position and velocity.
  const systems::BasicVector<T>* input_vector =
    this->EvalVectorInput(context, 0);
  // Obtain the structure we need to write into.
  systems::BasicVector<T>* const output_vector =
    output->GetMutableVectorData(0);
  // Update output by separately multiplying
  // position and velocity with the translating
  // matrix.
  int input_dof = this->translator_.cols();
  output_vector->get_mutable_value() <<
      this->translator_ * input_vector->get_value().head(input_dof),
      this->translator_ * input_vector->get_value().tail(input_dof);
}

template class DegenerateEulerJoint<double>;

}  // namespace particles
}  // namespace examples
}  // namespace drake
