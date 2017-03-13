#ifndef DEGENERATE_EULER_JOINT_INL_H
#define DEGENERATE_EULER_JOINT_INL_H

#include "drake/examples/uniformly_accelerated_particle/src/degenerate_euler_joint.hh"

namespace drake {
  namespace particles {

    template <typename T, int DOF>
    DegenerateEulerJoint<T, DOF>::DegenerateEulerJoint()
      : DegenerateEulerJoint(TranslatingMatrix::Identity()) {}

    template <typename T, int DOF>
    DegenerateEulerJoint<T, DOF>::DegenerateEulerJoint(const TranslatingMatrix& translator)
      : translator_(translator)
    {      
      this->DeclareInputPort(systems::kVectorValued, 2*DOF);
      this->DeclareOutputPort(systems::kVectorValued, 2*6);
    }
    
    template <typename T, int DOF>
    void DegenerateEulerJoint<T, DOF>::DoCalcOutput(const systems::Context<T>& context,
						    systems::SystemOutput<T>* output) const {
      const systems::BasicVector<T>* input_vec =
	this->EvalVectorInput(context, 0);
      systems::BasicVector<T>* const output_vec =
	output->GetMutableVectorData(0);
      output_vec->get_mutable_value() <<
	translator_ * input_vec->get_value().head(DOF),
	translator_ * input_vec->get_value().tail(DOF);
    }
    
  }  // namespace particles
}   // namespace drake

#endif // DEGENERATE_EULER_JOINT_INL_H
