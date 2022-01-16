#pragma once

#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

/** Specifies one of the common activation functions in a neural network. */
enum PerceptronActivationType {
  kIdentity,
  kReLU,
  kTanh,
};

/** The MultilayerPerceptron (MLP) is one of the most common forms of neural
 networks used in reinforcement learning (RL) today. This implementation
 provides a System interface to distinguish between the network's inputs and
 outputs (via ports), and the parameters, which are stored in the Context.

 Each layer of the network is implemented as xₙ₊₁ = σ(Wₙxₙ+bₙ), where xₙ is the
 output of the preceding layer, W are the weights, b are the biases, and σ() is
 the activation function.

 Note: For very large-scale neural network implementations, consider using a
 GPU-accelerated machine learning library like PyTorch, TensorFlow, or JAX. But
 most MLPs used in controls / RL are actually quite small. For those networks,
 the cost of transferring values/gradients from Drake to e.g. PyTorch is likely
 not worth the benefits. Another possible workflow might be to train a network
 in PyTorch, but then to copy the weights into an instance of this class for
 simulation.

 @system
 name: MultilayerPerceptron
 input_ports:
 - x
 output_ports:
 - y
 @endsystem

 @tparam_default_scalar
 @ingroup primitive_systems
*/
template <typename T>
class MultilayerPerceptron final : public LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MultilayerPerceptron)

  /** Constructs the MLP.
   @param layers is the number of elements in each layer of the network (the
   activation function does *not* count as an additional layer). The first
   element specifies the number of inputs, and the last layer specifies the
   number of outputs. We require at least 3 layers (to be considered an MLP).
   @param activation_type specifies the activation function, σ(), used in
   *every* layer of the network (including the last layer).

   @pydrake_mkdoc_identifier{single_activation} */
  MultilayerPerceptron(const std::vector<int>& layers,
                       PerceptronActivationType activation_type = kTanh);

  /** Constructs the MLP with an activation_type specified for each non-input
   layer.

   @param layers is the number of elements in each layer of the network (the
   activation function does *not* count as an additional layer). The first
   element specifies the number of inputs, and the last layer specifies the
   number of outputs. We require at least 3 layers (to be considered an MLP).
   @param activation_type specifies the activation function, σ(), used in
   *each* non-input layer of the network (including the last layer).

   `activation_type` should have one less element than `layers`.
   @pydrake_mkdoc_identifier{vector_activation} */
  MultilayerPerceptron(
      const std::vector<int>& layers,
      const std::vector<PerceptronActivationType>& activation_types);

  /** Scalar-converting copy constructor. See @ref system_scalar_conversion. */
  template <typename U>
  explicit MultilayerPerceptron(const MultilayerPerceptron<U>&);

  /** Sets all of the parameters in the `context` to small random numbers. */
  void SetRandomParameters(const Context<T>& context, Parameters<T>* parameters,
                           RandomGenerator* generator) const override;

  /** Returns the total number of parameters in the network, including all
   weights and biases. */
  int num_parameters() const { return num_parameters_; }

  /** Returns the number of elements in each layer of the network. */
  const std::vector<int>& layers() const { return layers_; }

  /** Returns the type of the activation function, σ(), used in the MLP. */
  PerceptronActivationType activation_type(int layer) const {
    DRAKE_DEMAND(layer >=0 && layer < num_weights_);
    return activation_types_[layer];
  }

  /** Returns a reference to all of the parameters (weights and biases) as a
   * single vector. */
  const VectorX<T>& GetParameters(const Context<T>& context) const;

  /** Sets all of the parameters in the network (weights and biases) using a
   * single vector. */
  void SetParameters(Context<T>* context,
                     const Eigen::Ref<const VectorX<T>>& params) const;

  /** Returns the weights used in the mapping from `layer` to `layer+1`.
   @pydrake_mkdoc_identifier{context} */
  Eigen::Map<const MatrixX<T>> GetWeights(const Context<T>& context,
                                          int layer) const;

  /** Returns the biases used in the mapping from `layer` to `layer+1`.
   @pydrake_mkdoc_identifier{context} */
  Eigen::Map<const VectorX<T>> GetBiases(const Context<T>& context,
                                         int layer) const;

  /** Sets the weights in the `context` used in the mapping from `layer` to
   `layer+1`.
    @pydrake_mkdoc_identifier{context} */
  void SetWeights(Context<T>* context, int layer,
                  const Eigen::Ref<const MatrixX<T>>& W) const;

  /** Sets the biases in the `context` used in the mapping from `layer` to
   `layer+1`.
    @pydrake_mkdoc_identifier{context} */
  void SetBiases(Context<T>* context, int layer,
                 const Eigen::Ref<const VectorX<T>>& b) const;

  /** Returns the weights in `params` used in the mapping from `layer` to
   `layer+1`.
   @pydrake_mkdoc_identifier{vector} */
  Eigen::Map<const MatrixX<T>> GetWeights(
      const Eigen::Ref<const VectorX<T>>& params, int layer) const;

  /** Returns the biases in `params` used in the mapping from `layer` to
   `layer+1`.
   @pydrake_mkdoc_identifier{vector} */
  Eigen::Map<const VectorX<T>> GetBiases(
      const Eigen::Ref<const VectorX<T>>& params, int layer) const;

  /** Sets the weights in `params` used in the mapping from `layer` to
   `layer+1`.
   @pydrake_mkdoc_identifier{vector} */
  void SetWeights(EigenPtr<VectorX<T>> params, int layer,
                  const Eigen::Ref<const MatrixX<T>>& W) const;

  /** Sets the biases in `params` used in the mapping from `layer` to `layer+1`.
   @pydrake_mkdoc_identifier{vector} */
  void SetBiases(EigenPtr<VectorX<T>> params, int layer,
                 const Eigen::Ref<const VectorX<T>>& b) const;

  /** Implements the Backpropagation algorithm for the MLP to compute the
   gradients of a scalar loss function with respect to the network parameters.

   Note: The classes uses the System Cache to minimize the number of dynamic
   memory allocations for repeated calls to this function with the same sized
   `X`.  Changing the batch size between calls requires memory allocations.

   @param X is a batch input, with one input per column.
   @param loss is a scalar loss function, with `Y` is the columnwise batch
   output of the network. It should return the scalar loss and set `dloss_dY`,
   the derivatives of the loss with respect to `Y`, which is pre-allocated
   to be the same size as `Y`.
   @param dloss_dparams are the gradients computed. We take the storage as in
   input argument to avoid memory allocations inside the algorithm.
   @returns the calculated loss.
   */
  double Backpropagation(
      const Context<T>& context, const Eigen::Ref<const Eigen::MatrixXd>& X,
      std::function<double(const Eigen::Ref<const Eigen::MatrixXd>& Y,
                           EigenPtr<Eigen::MatrixXd> dloss_dY)>
          loss,
      EigenPtr<Eigen::VectorXd> dloss_dparams) const;

  /** Calls Backpropagation with the mean-squared error loss function:
   loss = 1/N ∑ᵢ |yᵢ−yᵢᵈ|².
   See Backpropagation for details. */
  double BackpropagationMeanSquaredError(
      const Context<T>& context, const Eigen::Ref<const Eigen::MatrixXd>& X,
      const Eigen::Ref<const Eigen::MatrixXd>& Y_desired,
      EigenPtr<Eigen::VectorXd> dloss_dparams) const;

 private:
  // Calculates y = f(x) for the entire network.
  void CalcOutput(const Context<T>& context, BasicVector<T>* y) const;

  // Calculates the cache entries for the hidden units in the network.
  void CalcHiddenLayers(const Context<T>& context,
                        std::vector<VectorX<T>>* hidden) const;

  // Dummy function. The caching interface requires it to exist, but we do not
  // use it in practice.
  void BackpropCacheNoOp(const Context<T>& context,
                         std::vector<MatrixX<T>>* bp) const;

  int num_hidden_layers_;
  int num_weights_;
  int num_parameters_;
  std::vector<int> layers_;
  std::vector<PerceptronActivationType> activation_types_;

  // Stores the position index of each set of weights and biases in the main
  // parameter vector.
  std::vector<int> weight_indices_;
  std::vector<int> bias_indices_;

  // Function references implementing σ(x) and dσ/dx(x).
  std::vector<std::function<MatrixX<T>(const Eigen::Ref<const MatrixX<T>>&)>>
      sigma_{};
  // TODO(russt): Consider returning dsigma and sigma in the same function to
  // reuse computation (e.g. dtanh(x) = 1-tanh(x)^2).
  std::vector<std::function<MatrixX<T>(const Eigen::Ref<const MatrixX<T>>&)>>
      dsigma_{};

  CacheEntry* hidden_layer_cache_{};
  CacheEntry* backprop_cache_{};

  template <typename> friend class MultilayerPerceptron;
};

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::MultilayerPerceptron)
