#include "drake/perception/dev/feedforward_neural_network.h"

namespace drake {

using std::vector;
using drake::systems::Context;
using drake::systems::InputPortDescriptor;
using drake::systems::OutputPort;
using drake::systems::System;
using drake::systems::BasicVector;

using Eigen::AutoDiffScalar;

using std::cout;
using std::endl;

namespace perception {

// TODO(jwnimmer-tri) The W and b should be typed as doubles, not T.
// Only items on the Context should be of type T, never member fields.
template <typename T>
FeedforwardNeuralNetwork<T>::FeedforwardNeuralNetwork(
    const std::vector<MatrixX<T>>& W, const std::vector<VectorX<T>>& b,
    const std::vector<LayerType>& layers,
    const std::vector<NonlinearityType>& nonlinearities)
    : NeuralNetwork<T>(systems::SystemTypeTag<
          perception::FeedforwardNeuralNetwork>{}),
      input_index_{this->DeclareAbstractInputPort().get_index()},
      output_index_{
          this->DeclareVectorOutputPort(BasicVector<T>(W[W.size() - 1].rows()),
                                        &FeedforwardNeuralNetwork::DoCalcOutput)
              .get_index()} {
  DRAKE_THROW_UNLESS(W.size() == b.size());
  DRAKE_THROW_UNLESS(W.size() == layers.size());
  DRAKE_THROW_UNLESS(W.size() == nonlinearities.size());
  for (vector<int>::size_type i = 0; i < W.size(); i++) {
    rows_.push_back(W[i].rows());
    cols_.push_back(W[i].cols());
    weight_indices_.push_back(
        this->DeclareNumericParameter(*(EncodeWeightsToBasicVector(W[i]))));

    BasicVector<T> bias_vector(b[i]);
    bias_indices_.push_back(this->DeclareNumericParameter(bias_vector));
  }

  layers_ = layers;
  nonlinearities_ = nonlinearities;

  // Need to register the structural parameters now, when we have the matrices
  // on hand. Otherwise we need to get a context argument later

  // Ok to do this cast to int since we are not going to have a NN that is so
  // large that it will exhaust the range of int
  MatrixX<T> first_matrix = W[0];
  num_inputs_ = first_matrix.cols();

  MatrixX<T> last_matrix = W[W.size() - 1];
  num_outputs_ = last_matrix.rows();

  num_layers_ = W.size();

  // Need to store copies of these so we can use them in DoToAutoDiffXd()
  weights_matrices_ = W;
  bias_vectors_ = b;
}

namespace {
// Helper to convert U to T.  This can die once the constructor TODO is fixed.
template <typename T, typename U, int Rows, int Cols>
std::vector<Eigen::Matrix<T, Rows, Cols>>
CastVectorOfEigen(const std::vector<Eigen::Matrix<U, Rows, Cols>>& input) {
  std::vector<Eigen::Matrix<T, Rows, Cols>> result;
  result.reserve(input.size());
  for (const auto& item : input) {
    result.push_back(item.template cast<T>());
  }
  return result;
}
}  // namespace

template <typename T>
template <typename U>
FeedforwardNeuralNetwork<T>::FeedforwardNeuralNetwork(
    const FeedforwardNeuralNetwork<U>& other)
    : FeedforwardNeuralNetwork<T>(
          CastVectorOfEigen<T>(other.weights_matrices_),
          CastVectorOfEigen<T>(other.bias_vectors_),
          other.layers_,
          other.nonlinearities_) {}

template <typename T>
void FeedforwardNeuralNetwork<T>::DoCalcOutput(const Context<T>& context,
                                               BasicVector<T>* output) const {
  // Read the input
  VectorX<T> input_value = ReadInput(context);

  // Evaluate each layer
  VectorX<T> intermediate_value = input_value;
  std::unique_ptr<MatrixX<T>> weights;
  std::unique_ptr<VectorX<T>> bias;
  LayerType layer_type;
  NonlinearityType nonlinearity;
  for (int i = 0; i < num_layers_; i++) {
    weights = get_weight_matrix(i, context);
    bias = get_bias_vector(i, context);
    layer_type = layers_[i];
    nonlinearity = nonlinearities_[i];

    intermediate_value = EvaluateLayer(intermediate_value, *weights, *bias,
                                       layer_type, nonlinearity);
  }

  // Write output
  WriteOutput(intermediate_value, output);
}

template <typename T>
VectorX<T> FeedforwardNeuralNetwork<T>::EvaluateLayer(
    const VectorX<T>& layerInput, MatrixX<T> Weights, VectorX<T> bias,
    LayerType layer, NonlinearityType nonlinearity) const {
  // Only suppports fully-connected RELU at this time
  DRAKE_DEMAND(layer == LayerType::FullyConnected);
  DRAKE_DEMAND(nonlinearity == NonlinearityType::Relu);
  VectorX<T> layer_output = relu(Weights * layerInput + bias);

  return layer_output;
}

template <typename T>
VectorX<T> FeedforwardNeuralNetwork<T>::relu(VectorX<T> in) const {
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
                     (static_cast<size_t>(index) < weight_indices_.size()));

  const BasicVector<T>& encodedMatrix =
      this->template GetNumericParameter<BasicVector>(context,
                                                      weight_indices_[index]);

  return DecodeWeightsFromBasicVector(rows_[index], cols_[index],
                                      encodedMatrix);
}

template <typename T>
std::unique_ptr<VectorX<T>> FeedforwardNeuralNetwork<T>::get_bias_vector(
    int index, const Context<T>& context) const {
  DRAKE_THROW_UNLESS((0 <= index) &&
                     (static_cast<size_t>(index) < bias_indices_.size()));

  const BasicVector<T>& encoded_vector =
      this->template GetNumericParameter<BasicVector>(context,
                                                      bias_indices_[index]);

  VectorX<T>* bias_vector = new VectorX<T>(encoded_vector.get_value());
  std::unique_ptr<VectorX<T>> uptr(bias_vector);
  return uptr;
}

template <typename T>
std::unique_ptr<BasicVector<T>> FeedforwardNeuralNetwork<
    T>::EncodeWeightsToBasicVector(const MatrixX<T>& weights) const {
  VectorX<T> data_vector(weights.size());

  int vector_index = 0;
  for (int i = 0; i < weights.rows(); i++) {
    for (int j = 0; j < weights.cols(); j++) {
      data_vector(vector_index) = weights(i, j);
      vector_index++;
    }
  }

  std::unique_ptr<BasicVector<T>> uptr(new BasicVector<T>(data_vector));
  return uptr;
}

template <typename T>
std::unique_ptr<MatrixX<T>>
FeedforwardNeuralNetwork<T>::DecodeWeightsFromBasicVector(
    int rows, int cols, const BasicVector<T>& basic_vector) const {
  MatrixX<T>* weights = new MatrixX<T>(rows, cols);
  *weights = MatrixX<T>::Zero(rows, cols);

  VectorX<T> v = basic_vector.get_value();

  int vector_index = 0;
  for (int i = 0; i < rows; i++) {
    for (int j = 0; j < cols; j++) {
      (*weights)(i, j) = v(vector_index);
      vector_index++;
    }
  }

  std::unique_ptr<MatrixX<T>> uptr(weights);
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
const VectorX<T> FeedforwardNeuralNetwork<T>::ReadInput(
    const Context<T>& context) const {
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
template class FeedforwardNeuralNetwork<AutoDiffXd>;

}  // namespace perception
}  // namespace drake
