#pragma once

/// @file
/// Template method implementations for degenerate_euler_joint.h.
/// Most users should only include that file, not this one.
///
/// For background, see http://drake.mit.edu/cxx_inl.html.

#include "drake/examples/uniformly_accelerated_particle/degenerate_euler_joint.h"

namespace drake {
  namespace particles {

    template <typename T, int NDOF>
    DegenerateEulerJoint<T, NDOF>::DegenerateEulerJoint(const TranslatingMatrix& translator)
      : translator_(translator)
    {
      this->DeclareInputPort(systems::kVectorValued, 2*NDOF);
      this->DeclareOutputPort(systems::kVectorValued, 12);  // 2*6
    }
    
    template <typename T, int NDOF>
    void DegenerateEulerJoint<T, NDOF>::DoCalcOutput(const systems::Context<T>& context,
						    systems::SystemOutput<T>* output) const {
      // Get current input position and velocity.
      const systems::BasicVector<T>* input_vec =
	this->EvalVectorInput(context, 0);
      // Obtain the structure we need to write into.
      systems::BasicVector<T>* const output_vec =
	output->GetMutableVectorData(0);
      // Update output by separately multiplying
      // position and velocity with the translating
      // matrix
      output_vec->get_mutable_value() <<
	translator_ * input_vec->get_value().head(NDOF), // first NDOF inputs, position
	translator_ * input_vec->get_value().tail(NDOF); // last NDOF inputs, velocity
    }
    
  }  // namespace particles
}   // namespace drake

