#pragma once

#include <memory>

// take a look at other useful things that may be in here
#include<Eigen/Geometry>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/framework/leaf_system.h"


namespace drake {
namespace automotive {

template <typename T>
class TrafficLight : public systems::LeafSystem<T> {
	public:
	  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TrafficLight)

		TrafficLight( T x_position, T y_position, T radius, T period );

		// Do we need an input port? Hmmm... maybe it can just be time triggered
		//const systems::InputPortDescriptor<T>& traffic_input() const;
		const systems::OutputPort<T>& output() const;


	private:
	  void DoCalcOutput( const systems::Context<T>& context,
	                   systems::BasicVector<T>* output ) const;

	  // Indices for input / output ports
	  const int traffic_input_index_{};
	  const int output_index_{};
	  int parameters_index_;

		VectorX<T> ReadParameters( const systems::Context<T>& context ) const;
	  void WriteOutput( const VectorX<T> value,
	                    systems::BasicVector<T>* output ) const;
}; 

} // namespace drake
} // namespace automotive
