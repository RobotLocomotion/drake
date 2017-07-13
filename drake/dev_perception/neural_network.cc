#include "drake/dev_perception/neural_network.h"

namespace drake {

using std::vector;
namespace perception {

//template <typename T>
//NeuralNetwork<T>::NeuralNetwork()
//	:input_index_{ this->DeclareAbstractInputPort().get_index() },
//	output_index_{ this->DeclareVectorOutputPort(
//																		BasicVector<T>(1),
//																		&NeuralNetwork::CalcOutput ).get_index() } {
//}

// TODO(nikos-tri) How can I templatize the MatrixXd by T?
template <typename T>
NeuralNetwork<T>::NeuralNetwork( std::vector<MatrixXd> W,
																	std::vector<VectorXd> b,
																	std::vector<LayerType> layers,
																	std::vector<NonlinearityType> nonlinearities )
	:input_index_{ this->DeclareAbstractInputPort().get_index() },
	output_index_{ this->DeclareVectorOutputPort(
																		BasicVector<T>(1), // for example
																		&NeuralNetwork::CalcOutput ).get_index() } {

	DRAKE_THROW_UNLESS( W.size() == b.size() );
	for ( vector<int>::size_type i = 0; i < W.size(); i++ ) {
		matrix_indices_.push_back( this->DeclareNumericParameter( *(encode(W[i])) ) );

		BasicVector<T> biasVector( b[i] );
		bias_indices_.push_back( this->DeclareNumericParameter( biasVector ) );
	}

	layers_ = layers;
	nonlinearities = nonlinearities;

	// Need to register the structural parameters now, when we have the matrices
	// on hand. Otherwise we need to get a context argument later Ok to do this
	// cast to int since we are not going to have a NN that is so large that it
	// will exhaust the range of int
	MatrixXd firstMatrix = W[0]; num_inputs_ = firstMatrix.cols();

	MatrixXd lastMatrix = W[W.size()-1];
	num_outputs_ = lastMatrix.rows();

	num_layers_ = W.size();
}

template <typename T>
void NeuralNetwork<T>::CalcOutput( const Context<T>& context,
																					BasicVector<T>* output ) const {

	// Read the input
	VectorXd inputValue = readInput( context );

	// Evaluate each layer	
	VectorXd intermediateValue = inputValue;
	std::unique_ptr<MatrixXd> Weights;
	std::unique_ptr<VectorXd> bias;
	LayerType layerType;
	NonlinearityType nonlinearity;
	for ( int i = 0; i < num_layers_; i++ ) {
		Weights = getWeightMatrix( i, context );
		bias = getBiasVector( i, context );
		layerType = layers_[i];
		nonlinearity = nonlinearities_[i];
		
		intermediateValue = evaluateLayer( intermediateValue,
																				*Weights, *bias,
																				layerType, nonlinearity );
	}

	// Write output
	writeOutput( intermediateValue, output );
}

template <typename T>
VectorXd NeuralNetwork<T>::evaluateLayer( const VectorXd& layerInput,
																					MatrixXd Weights,
																					VectorXd bias,
																					LayerType layer, 
																					NonlinearityType nonlinearity ) const {

	// Only suppports fully-connected RELU at this time
	DRAKE_ASSERT( layer == LayerType::FullyConnected );
	DRAKE_ASSERT( nonlinearity == NonlinearityType::Relu );
	VectorXd layerOutput = relu( Weights*layerInput + bias );
	return layerOutput;
}

template <typename T>
VectorXd NeuralNetwork<T>::relu( VectorXd in ) const {
	// TODO(nikos-tri) This function begs to be optimized, somehow -- maybe like
	// negIndices = any( input < 0 ) 
	// input( negIndices ) = 0 
	// ...or something like that.
	VectorXd result = in;
	for ( int i = 0; i < result.size(); i++ ) {
		if ( result(i) < 0 ) {
			result(i) = 0;
		}
	}
	return result;
}





template <typename T>
int NeuralNetwork<T>::getNumLayers() const {
	return num_layers_;
}
template <typename T>
int NeuralNetwork<T>::getNumInputs() const {
	return num_inputs_;
}
template <typename T>
int NeuralNetwork<T>::getNumOutputs() const {
	return num_outputs_;
}

template <typename T>
std::unique_ptr<MatrixXd>
NeuralNetwork<T>::getWeightMatrix( int index, const Context<T>& context ) const {

	DRAKE_THROW_UNLESS((0 <= index) && 
		((vector<int>::size_type)index < matrix_indices_.size()));

	const BasicVector<T>& encodedMatrix = 
		this->template GetNumericParameter<BasicVector>( context,
																									matrix_indices_[index] );

		return decode( encodedMatrix );
}

template <typename T>
std::unique_ptr<VectorXd>
NeuralNetwork<T>::getBiasVector( int index, const Context<T>& context ) const {

	DRAKE_THROW_UNLESS((0 <= index) && 
		((vector<int>::size_type)index < bias_indices_.size()));

	const BasicVector<T>& encodedVector = 
		this->template GetNumericParameter<BasicVector>( context,
																									bias_indices_[index] );

	VectorXd *biasVector = new VectorXd( encodedVector.get_value() );
	std::unique_ptr<VectorXd> uptr( biasVector );
	return uptr;
}

template <typename T>
std::unique_ptr<BasicVector<T>> 
NeuralNetwork<T>::encode( const MatrixXd& matrix ) const {
	// + 2 is to encode matrix dimensions
	VectorXd dataVector( matrix.size() + 2 );

	dataVector << matrix.rows(), matrix.cols();
	int vectorIndex = 2;
	for ( int i = 0; i < matrix.rows(); i++  ) {
		for ( int j = 0; j < matrix.cols(); j++ ) {
			dataVector( vectorIndex ) = matrix( i, j );
			vectorIndex++;
		}
	}

	std::unique_ptr<BasicVector<T>> uptr( new BasicVector<T>( dataVector ) );
	return uptr;
}

template <typename T>
std::unique_ptr<MatrixXd> 
NeuralNetwork<T>::decode( const BasicVector<T>& basicVector ) const {
	int rows = -1; int cols = -1; 
	VectorXd vector = basicVector.get_value();
	rows = vector(0); cols = vector(1);

	MatrixXd * matrix = new MatrixXd(rows, cols);
	*matrix = MatrixXd::Zero(rows, cols);

	int vectorIndex = 2;
	for ( int i = 0; i < rows; i++ ) {
		for ( int j = 0; j < cols; j++ ) {
			(*matrix)(i,j) = vector( vectorIndex );
			vectorIndex++;
		}
	}

	std::unique_ptr<MatrixXd> uptr( matrix );
	return uptr;
}


template <typename T>
const InputPortDescriptor<T>& NeuralNetwork<T>::input() const {
	return System<T>::get_input_port( input_index_ );
}

template <typename T>
const OutputPort<T>& NeuralNetwork<T>::output() const {
	return System<T>::get_output_port( output_index_ );
}

template <typename T>
const VectorX<T> NeuralNetwork<T>::readInput( const Context<T>& context ) const {
	const BasicVector<T> * input = 
		this->template EvalVectorInput<BasicVector>(context, input_index_ );
	DRAKE_ASSERT( (input != nullptr) );
	return input->get_value();
}
template <typename T>
void NeuralNetwork<T>::writeOutput( const VectorX<T> value, BasicVector<T>* output ) const {
	output->set_value( value );
}

template class NeuralNetwork<double>;

} // namespace automotive
} // namespace drake
