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

// Forward declarations.
namespace internal {

// Note: This struct is defined outside the class to avoid the ReportZeroHash
// warning in AbstractValue.
template <typename T>
struct CalcLayersData;

}  // namespace internal

/** The MultilayerPerceptron (MLP) is one of the most common forms of neural
 networks used in reinforcement learning (RL) today. This implementation
 provides a System interface to distinguish between the network's inputs and
 outputs (via ports), and the parameters, which are stored in the Context.

 Each layer of the network is implemented as xₙ₊₁ = σ(Wₙxₙ+bₙ), where xₙ is the
 output of the preceding layer, W are the weights, b are the biases, and σ() is
 the activation function.  We additionally use the shorthand x to denote the
 input layer and y to denote the output layer: y=xₘ for an m-layer network.

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
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MultilayerPerceptron);

  /** Constructs the MLP with the same activation type for every layer (except
   the output).

   @param layers is the number of elements in each layer of the network (the
   activation function does *not* count as an additional layer). The first
   element specifies the number of inputs, and the last layer specifies the
   number of outputs.
   @param activation_type specifies an activation function, σ(), used in
   _every_ hidden layer of the network. kIdentity will be used for the output.

   @pydrake_mkdoc_identifier{single_activation} */
  MultilayerPerceptron(const std::vector<int>& layers,
                       PerceptronActivationType activation_type = kTanh);

  /** Constructs the MLP with an activation_type specified for each non-input
   layer.

   @param layers is the number of elements in each layer of the network (the
   activation function does *not* count as an additional layer). The first
   element specifies the number of inputs, and the last layer specifies the
   number of outputs.
   @param activation_types specifies the activation function, σ(), used in
   _each_ non-input layer of the network (including the last layer).

   `activation_types` should have one less element than `layers`.
   @pydrake_mkdoc_identifier{vector_activation} */
  MultilayerPerceptron(
      const std::vector<int>& layers,
      const std::vector<PerceptronActivationType>& activation_types);

  // TODO(russt): A more general form of this might look like e.g.
  // MultilayerPerceptron(
  //    int num_inputs, const std::vector<GradientFunction>& input_features,
  //    const std::vector<int>& remaining_layers,
  //    const std::vector<PerceptronActivationType>& activation_types);
  // but I won't implement that until someone needs it, and the sin/cos for
  // joint angles seems a particularly important case that merits its own API
  // in Drake.
  /** Constructs the MLP with an additional option to transform the input vector
   so that the function is periodic in 2π.

   For instance, for a rotary joint on a robot, this could be used to apply the
   transform [x, y] => [sin x, cos x, y]. This would be accomplished by
   passing `use_sin_cos_for_input = [true, false]`.

   Note that when this transformation is applied, `num_inputs() != layers()[0]`.
   `num_inputs() == 2 != layers()[0] == 3`.

   @param use_sin_cos_for_input is a boolean vector that determines whether the
   sin/cos transform is applied to each element.
   @param remaining_layers is the number of elements in each layer of the
   network (the activation function does *not* count as an additional layer).
   The first element specifies the size of the first hidden layer, and the last
   layer specifies the number of outputs.
   @param activation_types specifies the activation function, σ(), used in
   _each_ non-input layer of the network (including the last layer).

   `activation_types` should have the same number of elements as
   `remaining_layers`.
   @pydrake_mkdoc_identifier{sin_cos_features} */
  MultilayerPerceptron(
      const std::vector<bool>& use_sin_cos_for_input,
      const std::vector<int>& remaining_layers,
      const std::vector<PerceptronActivationType>& activation_types);

  /** Scalar-converting copy constructor. See @ref system_scalar_conversion. */
  template <typename U>
  explicit MultilayerPerceptron(const MultilayerPerceptron<U>&);

  /** Sets all of the parameters (all weights and biases) in the `parameters`
   using the "LeCun initialization": a uniform distribution with mean zero and
   standard deviation √(1/m), where m is the number of connections feeding into
   the corresponding node. See eq. (16) in
   http://yann.lecun.com/exdb/publis/pdf/lecun-98b.pdf .

   This is typically called via System<T>::SetRandomContext. By contrast,
   System<T>::SetDefaultContext will set all weights and biases to zero. */
  void SetRandomParameters(const Context<T>& context, Parameters<T>* parameters,
                           RandomGenerator* generator) const override;

  /** Returns the total number of parameters in the network, including all
   weights and biases. */
  int num_parameters() const { return num_parameters_; }

  /** Returns the number of elements in each layer of the network. */
  const std::vector<int>& layers() const { return layers_; }

  /** Returns the type of the activation function, σ(), used in the MLP. */
  PerceptronActivationType activation_type(int layer) const {
    DRAKE_DEMAND(layer >= 0 &&
                 layer < static_cast<int>(activation_types_.size()));
    return activation_types_[layer];
  }

  /** Returns a reference to all of the parameters (weights and biases) as a
   single vector. Use GetWeights and GetBiases to extract the components. */
  const VectorX<T>& GetParameters(const Context<T>& context) const;

  /** Returns a mutable reference to all of the parameters (weights and biases)
   as a single vector. */
  Eigen::VectorBlock<VectorX<T>> GetMutableParameters(
      Context<T>* context) const;

  /** Sets all of the parameters in the network (weights and biases) using a
   single vector. Use SetWeights and SetBiases to extract the components. */
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

  /** Helper function signature for Backpropagation(). */
  using ScalarLossFunction = std::function<T(
      const Eigen::Ref<const MatrixX<T>>& Y, EigenPtr<MatrixX<T>> dloss_dY)>;

  /** Implements the Backpropagation algorithm for the MLP to compute the
   gradients of a scalar loss function with respect to the network parameters.

   Note: The class uses the System Cache to minimize the number of dynamic
   memory allocations for repeated calls to this function with the same sized
   `X`.  Changing the batch size between calls to this method or BatchOutput
   requires memory allocations.

   @param X is a batch input, with one input per column.
   @param loss is a scalar loss function, where `Y` is the columnwise batch
   output of the network. It should return the scalar loss and set `dloss_dY`,
   the derivatives of the loss with respect to `Y`, which is pre-allocated to
   be the same size as `Y`.
   @param dloss_dparams are the gradients computed. We take the storage as an
   input argument to avoid memory allocations inside the algorithm.
   @returns the calculated loss.

   Note: It is expected that this algorithm will be used with T=double. It uses
   analytical gradients; AutoDiffXd is not required.
   */
  T Backpropagation(const Context<T>& context,
                    const Eigen::Ref<const MatrixX<T>>& X,
                    const ScalarLossFunction& loss,
                    EigenPtr<VectorX<T>> dloss_dparams) const;

  /** Calls Backpropagation with the mean-squared error loss function:
   loss = 1/N ∑ᵢ |yᵢ−yᵢᵈ|², where yᵈ is the desired values for y.
   See Backpropagation for details. */
  T BackpropagationMeanSquaredError(
      const Context<T>& context, const Eigen::Ref<const MatrixX<T>>& X,
      const Eigen::Ref<const MatrixX<T>>& Y_desired,
      EigenPtr<VectorX<T>> dloss_dparams) const;

  /** Evaluates the batch output for the MLP with a batch input vector. Each
   column of `X` represents an input, and each column of `Y` will be assigned
   the corresponding output.

   If the output layer of the network has size 1 (scalar output), and `dYdX !=
   nullptr`, then `dYdX` is populated with the batch gradients of the scalar
   output `Y` relative to the input `X`: the (i,j)th element represents the
   gradient dY(0,j) / dX(i,j).

   Note: In python, use numpy.asfortranarray() to allocate the writeable
   matrices `Y` and (if needed) `dYdX`.

   This methods shares the cache with Backpropagation. If the size of X changes
   here or in Backpropagation, it may force dynamic memory allocations.

   @throws std::exception if dYdX != nullptr and the network has more than one
   output.
   */
  void BatchOutput(const Context<T>& context,
                   const Eigen::Ref<const MatrixX<T>>& X,
                   EigenPtr<MatrixX<T>> Y,
                   EigenPtr<MatrixX<T>> dYdX = nullptr) const;

 private:
  // Calculates y = f(x) for the entire network.
  void CalcOutput(const Context<T>& context, BasicVector<T>* y) const;

  // Calculates the cache entries for the hidden units in the network.
  void CalcLayers(const Context<T>& context,
                  internal::CalcLayersData<T>* data) const;

  // Calculates the (potentially batch) feature vector values.  When `X` is
  // size `num_inputs`-by-`N`, then `Features` is set to size
  // `layers()[0]`-by-`N`.
  void CalcInputFeatures(const Eigen::Ref<const MatrixX<T>>& X,
                         MatrixX<T>* input_features) const;

  int num_weights_;     // The number of weight matrices (number of layers -1 ).
  int num_parameters_;  // Total number of parameters.
  std::vector<int> layers_;  // The number of neurons in each layer.
  std::vector<PerceptronActivationType> activation_types_;
  std::vector<bool> use_sin_cos_for_input_{};
  bool has_input_features_{false};

  // Stores the position index of each set of weights and biases in the main
  // parameter vector.
  std::vector<int> weight_indices_;
  std::vector<int> bias_indices_;

  CacheEntry* calc_layers_cache_{};
  CacheEntry* backprop_cache_{};

  template <typename>
  friend class MultilayerPerceptron;
};

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::MultilayerPerceptron);
