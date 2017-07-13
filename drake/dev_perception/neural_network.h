#pragma once

#include <memory>
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/framework/leaf_system.h"

#include <Eigen/Dense>

namespace drake {

// TODO(nikos-tri) Don't have these in the header file, since it bloats the
// namespace for files that include this one
using drake::systems::Context;
using drake::systems::InputPortDescriptor;
using drake::systems::OutputPort;
using drake::systems::System;
using drake::systems::DiscreteValues;
using drake::systems::BasicVector;

using Eigen::Matrix;
using Eigen::MatrixXd;
using Eigen::VectorXd;


// TODO(nikos-tri) Delete these
using std::cout;
using std::endl;

// If your component is an automotive component; otherwise set the namespace as
// appropriate
namespace perception {


enum class LayerType { FullyConnected, Convolutional };
enum class NonlinearityType {Relu, Sigmoid, Atan};

template <typename T>
class NeuralNetwork : public systems::LeafSystem<T> {

	public:
	DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN( NeuralNetwork )

	/// TODO(nikos-tri): Add documentation
	//explicit NeuralNetwork();
	// TODO(nikos-tri) instead of MatrixXd, use template type T
	explicit NeuralNetwork( std::vector<MatrixXd> W, std::vector<VectorXd> b,
													std::vector<LayerType> layers,
													std::vector<NonlinearityType> nonlinearities );
	// TODO(nikos-tri) Constructor should also take a list of types of layers

	// TODO(nikos-tri) instead of MatrixXd, use template type T
	const InputPortDescriptor<T>& input() const;
	const OutputPort<T>& output() const;

	int getNumLayers() const;
	int getNumInputs() const;
	int getNumOutputs() const;
	std::unique_ptr<MatrixXd> getWeightMatrix( int index, const Context<T>& context ) const;
	std::unique_ptr<VectorXd> getBiasVector( int index, const Context<T>& context ) const;
	std::unique_ptr<BasicVector<T>> encode( const MatrixXd& matrix ) const;
	std::unique_ptr<MatrixXd> decode( const BasicVector<T>& vector ) const;

	private:
	void CalcOutput( const Context<T>& context,
										BasicVector<T>* output ) const;

	VectorXd evaluateLayer( const VectorXd& layerInput,
													MatrixXd Weights,
													VectorXd bias,
													LayerType layer, NonlinearityType nonlinearity ) const;
	VectorXd relu( VectorXd in ) const;

	// To reduce annoying cruft in the main computation and improve readability
	const VectorX<T> readInput( const Context<T>& context ) const;
	void writeOutput( const VectorX<T> value, BasicVector<T>* output ) const;

	
	// Info on the types of layers and nonlinearities
	std::vector<LayerType> layers_;
	std::vector<NonlinearityType> nonlinearities_;

	// So we can recover the weights and biases from the Context
	std::vector<int> matrix_indices_;
	std::vector<int> bias_indices_;

	// Indices for inputs and outputs
	const int input_index_;
	const int output_index_;

	// Structural parameters inferred from weights and biases given to constructor
	int num_inputs_;
	int num_outputs_;
	int num_layers_;

};

} // namespace automotive
} // namespace drake
