#include "traffic_light.h"

#include <cmath>
#include <iostream>

namespace drake {
namespace automotive {

using drake::systems::System;
using systems::BasicVector;
using std::fmod;

// just for debugging
using std::cout;
using std::endl;


template <typename T>
TrafficLight<T>::TrafficLight( T x_position, T y_position,
                               T radius, T period )
                               : output_index_{
                               	 this->DeclareVectorOutputPort(BasicVector<T>(4),
                               	    &TrafficLight::DoCalcOutput).get_index() } {

  //cout << "Constructing a traffic light..." << endl;
	//cout << "x_position: " << x_position << endl;
	//cout << "y_position: " << y_position << endl;
	//cout << "radius: " << radius << endl;
	//cout << "period: " << period << endl;

	// Register parameters
	VectorX<T> parameters(4);
	parameters(0) = x_position;
	parameters(1) = y_position;
	parameters(2) = radius;
	parameters(3) = period;

	//cout << "Creating a basic vector..." << endl;
	BasicVector<T> p(parameters);
	//cout << "Registering parameters..." << endl;
	parameters_index_ = this->DeclareNumericParameter(p);
	//cout << "Done." << endl;
}

//template <typename T>
//const systems::InputPortDescriptor TrafficLight<T>::traffic_input() const {
//	return System<T>::get_input_port(traffic_input_index_);
//}

template <typename T>
const systems::OutputPort<T>& TrafficLight<T>::output() const {
	return System<T>::get_output_port(output_index_);
}

template <typename T>
void TrafficLight<T>::DoCalcOutput( const systems::Context<T>& context,
                                  systems::BasicVector<T>* output ) const {

  VectorX<T> parameters = ReadParameters( context );
  T x_position = parameters(0);
  T y_position = parameters(1);
  T radius = parameters(2);
  T period = parameters(3);

  // get time from context
  T time = context.get_time();

  // if time something something, output: position, radius, and "0" for open
  // if time something something, output: position, radius, and "1" for closed
  T signal = 0;
  if ( fmod( time, period ) >= (period/2) ) {
  	signal = 1;
	}

	VectorX<T> signal_data(4);
	signal_data(0) = x_position;
	signal_data(1) = y_position;
	signal_data(2) = radius;
	signal_data(3) = signal;
	WriteOutput( signal_data, output );
}

template <typename T>
VectorX<T> TrafficLight<T>::ReadParameters( const systems::Context<T>& context ) const {
  const BasicVector<T>& p = 
     this->template GetNumericParameter<BasicVector>( context, parameters_index_ );
  VectorX<T> parameters = p.get_value();
  return parameters;
}

template <typename T>
void TrafficLight<T>::WriteOutput(const VectorX<T> value,
                                  BasicVector<T>* output) const {
  output->set_value(value);
}

template class TrafficLight<double>;

} // namespace automotive
} // namespace drake
