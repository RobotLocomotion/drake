#pragma once

#include <memory>
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/framework/leaf_system.h"

#include <Eigen/Dense>

namespace drake {
namespace perception {

enum class LayerType { FullyConnected, Convolutional, Dropout };
enum class NonlinearityType { Relu, Sigmoid, Atan };

template <typename T>
class NeuralNetwork : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(NeuralNetwork)

  /// TODO(nikos-tri): Add documentation
  explicit NeuralNetwork(std::vector<MatrixX<T>> W, std::vector<VectorX<T>> b,
                         std::vector<LayerType> layers,
                         std::vector<NonlinearityType> nonlinearities);

  const systems::InputPortDescriptor<T>& input() const;
  const systems::OutputPort<T>& output() const;

  int getNumLayers() const;
  int getNumInputs() const;
  int getNumOutputs() const;
  std::unique_ptr<MatrixX<T>> getWeightMatrix(
      int index, const systems::Context<T>& context) const;
  std::unique_ptr<VectorX<T>> getBiasVector(
      int index, const systems::Context<T>& context) const;
  std::unique_ptr<systems::BasicVector<T>> encode(
      const MatrixX<T>& matrix) const;
  std::unique_ptr<MatrixX<T>> decode(
      const systems::BasicVector<T>& vector) const;

 private:
  void DoCalcOutput(const systems::Context<T>& context,
                    systems::BasicVector<T>* output) const;
  VectorX<T> evaluateLayer(const VectorX<T>& layerInput, MatrixX<T> Weights,
                           VectorX<T> bias, LayerType layer,
                           NonlinearityType nonlinearity) const;
  VectorX<T> relu(VectorX<T> in) const;

  // To improve readability of the main computation
  const VectorX<T> readInput(const systems::Context<T>& context) const;
  void writeOutput(const VectorX<T> value,
                   systems::BasicVector<T>* output) const;

  // The types of layers and nonlinearities in this NN
  std::vector<LayerType> layers_;
  std::vector<NonlinearityType> nonlinearities_;

  // Indices for weights and biases, which are stored as params in the Context
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

}  // namespace automotive
}  // namespace drake
