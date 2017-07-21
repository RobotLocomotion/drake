#include "drake/dev_perception/feedforward_neural_network.h"

namespace drake {

using std::vector;
using drake::systems::Context;
using drake::systems::InputPortDescriptor;
using drake::systems::OutputPort;
using drake::systems::System;
using drake::systems::BasicVector;

using std::cout;
using std::endl;

namespace perception {

template <typename T>
FeedforwardNeuralNetwork<T>::FeedforwardNeuralNetwork(const std::vector<MatrixX<T>>& W,
                                const std::vector<VectorX<T>>& b,
                                const std::vector<LayerType>& layers,
                                const std::vector<NonlinearityType>& nonlinearities)
    : input_index_{this->DeclareAbstractInputPort().get_index()},
      output_index_{
          this->DeclareVectorOutputPort(BasicVector<T>(W[W.size() - 1].rows()),
                                        &FeedforwardNeuralNetwork::DoCalcOutput)
              .get_index()} {
  DRAKE_THROW_UNLESS(W.size() == b.size());
  DRAKE_THROW_UNLESS(W.size() == layers.size());
  DRAKE_THROW_UNLESS(W.size() == nonlinearities.size());
  for (vector<int>::size_type i = 0; i < W.size(); i++) {
    weight_indices_.push_back(this->DeclareNumericParameter(*(Encode(W[i]))));

    BasicVector<T> biasVector(b[i]);
    bias_indices_.push_back(this->DeclareNumericParameter(biasVector));
  }

  layers_ = layers;
  nonlinearities_ = nonlinearities;

  // Need to register the structural parameters now, when we have the matrices
  // on hand. Otherwise we need to get a context argument later

  // Ok to do this cast to int since we are not going to have a NN that is so
  // large that it will exhaust the range of int
  MatrixX<T> firstMatrix = W[0];
  num_inputs_ = firstMatrix.cols();

  MatrixX<T> lastMatrix = W[W.size() - 1];
  num_outputs_ = lastMatrix.rows();

  num_layers_ = W.size();
}

template <typename T>
void FeedforwardNeuralNetwork<T>::DoCalcOutput(const Context<T>& context,
                                    BasicVector<T>* output) const {
  // Read the input
  VectorX<T> inputValue = ReadInput(context);

  // Evaluate each layer
  VectorX<T> intermediateValue = inputValue;
  std::unique_ptr<MatrixX<T>> Weights;
  std::unique_ptr<VectorX<T>> bias;
  LayerType layerType;
  NonlinearityType nonlinearity;
  for (int i = 0; i < num_layers_; i++) {
    Weights = get_weight_matrix(i, context);
    bias = get_bias_vector(i, context);
    layerType = layers_[i];
    nonlinearity = nonlinearities_[i];

    intermediateValue = EvaluateLayer(intermediateValue, *Weights, *bias,
                                      layerType, nonlinearity);
  }

  // Write output
  WriteOutput(intermediateValue, output);
}

template <typename T>
VectorX<T> FeedforwardNeuralNetwork<T>::EvaluateLayer(
    const VectorX<T>& layerInput, MatrixX<T> Weights, VectorX<T> bias,
    LayerType layer, NonlinearityType nonlinearity) const {
  // Only suppports fully-connected RELU at this time
  DRAKE_ASSERT(layer == LayerType::FullyConnected);
  DRAKE_ASSERT(nonlinearity == NonlinearityType::Relu);
  VectorX<T> layerOutput = relu(Weights * layerInput + bias);
  return layerOutput;
}

template <typename T>
VectorX<T> FeedforwardNeuralNetwork<T>::relu(VectorX<T> in) const {
  // TODO(nikos-tri) This function begs to be optimized, somehow -- maybe like
  // negIndices = any( input < 0 )
  // input( negIndices ) = 0
  // ...or something like that.
  VectorX<T> result = in;
  for (int i = 0; i < result.size(); i++) {
    if (result(i) < 0) {
      result(i) = 0;
    }
  }
  return result;
}

template <typename T>
int FeedforwardNeuralNetwork<T>::get_num_layers() const {
  return num_layers_;
}
template <typename T>
int FeedforwardNeuralNetwork<T>::get_num_inputs() const {
  return num_inputs_;
}
template <typename T>
int FeedforwardNeuralNetwork<T>::get_num_outputs() const {
  return num_outputs_;
}

template <typename T>
std::unique_ptr<MatrixX<T>> FeedforwardNeuralNetwork<T>::get_weight_matrix(
    int index, const Context<T>& context) const {
  DRAKE_THROW_UNLESS((0 <= index) &&
                     ((vector<int>::size_type)index < weight_indices_.size()));

  const BasicVector<T>& encodedMatrix =
      this->template GetNumericParameter<BasicVector>(context,
                                                      weight_indices_[index]);

  return Decode(encodedMatrix);
}

template <typename T>
std::unique_ptr<VectorX<T>> FeedforwardNeuralNetwork<T>::get_bias_vector(
    int index, const Context<T>& context) const {
  DRAKE_THROW_UNLESS((0 <= index) &&
                     ((vector<int>::size_type)index < bias_indices_.size()));

  const BasicVector<T>& encodedVector =
      this->template GetNumericParameter<BasicVector>(context,
                                                      bias_indices_[index]);

  VectorX<T>* biasVector = new VectorX<T>(encodedVector.get_value());
  std::unique_ptr<VectorX<T>> uptr(biasVector);
  return uptr;
}

template <typename T>
std::unique_ptr<BasicVector<T>> FeedforwardNeuralNetwork<T>::Encode(
    const MatrixX<T>& matrix) const {
  // + 2 is to encode matrix dimensions
  VectorX<T> dataVector(matrix.size() + 2);

  dataVector << matrix.rows(), matrix.cols();
  int vectorIndex = 2;
  for (int i = 0; i < matrix.rows(); i++) {
    for (int j = 0; j < matrix.cols(); j++) {
      dataVector(vectorIndex) = matrix(i, j);
      vectorIndex++;
    }
  }

  std::unique_ptr<BasicVector<T>> uptr(new BasicVector<T>(dataVector));
  return uptr;
}

template <typename T>
std::unique_ptr<MatrixX<T>> FeedforwardNeuralNetwork<T>::Decode(
    const BasicVector<T>& basicVector) const {
  int rows = -1;
  int cols = -1;
  VectorX<T> vector = basicVector.get_value();
  rows = vector(0);
  cols = vector(1);

  MatrixX<T>* matrix = new MatrixX<T>(rows, cols);
  *matrix = MatrixX<T>::Zero(rows, cols);

  int vectorIndex = 2;
  for (int i = 0; i < rows; i++) {
    for (int j = 0; j < cols; j++) {
      (*matrix)(i, j) = vector(vectorIndex);
      vectorIndex++;
    }
  }

  std::unique_ptr<MatrixX<T>> uptr(matrix);
  return uptr;
}

template <typename T>
const InputPortDescriptor<T>& FeedforwardNeuralNetwork<T>::input() const {
  return System<T>::get_input_port(input_index_);
}

template <typename T>
const OutputPort<T>& FeedforwardNeuralNetwork<T>::output() const {
  return System<T>::get_output_port(output_index_);
}

template <typename T>
const VectorX<T> FeedforwardNeuralNetwork<T>::ReadInput(const Context<T>& context) const {
  const BasicVector<T>* input =
      this->template EvalVectorInput<BasicVector>(context, input_index_);
  DRAKE_ASSERT((input != nullptr));
  return input->get_value();
}
template <typename T>
void FeedforwardNeuralNetwork<T>::WriteOutput(const VectorX<T> value,
                                   BasicVector<T>* output) const {
  output->set_value(value);
}

template class FeedforwardNeuralNetwork<double>;

}  // namespace automotive
}  // namespace drake
