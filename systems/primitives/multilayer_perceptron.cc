#include "drake/systems/primitives/multilayer_perceptron.h"

#include "drake/common/default_scalars.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace systems {

using Eigen::MatrixXd;
using Eigen::VectorXd;

template <typename T>
MultilayerPerceptron<T>::MultilayerPerceptron(
    const std::vector<int>& layers, PerceptronActivationType activation_type)
    : MultilayerPerceptron<T>(layers, std::vector<PerceptronActivationType>(
                                          layers.size() - 1, activation_type)) {
}

template <typename T>
MultilayerPerceptron<T>::MultilayerPerceptron(
    const std::vector<int>& layers,
    const std::vector<PerceptronActivationType>& activation_types)
    : LeafSystem<T>(SystemTypeTag<MultilayerPerceptron>{}),
      num_hidden_layers_(layers.size() - 2),
      num_weights_(layers.size() - 1),
      layers_(layers),
      activation_types_(activation_types) {
  DRAKE_DEMAND(num_hidden_layers_ >= 1);  // Otherwise its not "multilayer"!
  this->DeclareVectorInputPort("x", layers[0]);
  this->DeclareVectorOutputPort("y", layers[num_weights_],
                                &MultilayerPerceptron<T>::CalcOutput);

  num_parameters_ = 0;
  weight_indices_.reserve(num_weights_);
  bias_indices_.reserve(num_weights_);
  for (int i = 0; i < num_weights_; ++i) {
    weight_indices_[i] = num_parameters_;
    num_parameters_ += layers[i + 1] * layers[i];
    bias_indices_[i] = num_parameters_;
    num_parameters_ += layers[i + 1];

    if (activation_types_[i] == kTanh) {
      sigma_.push_back([](const Eigen::Ref<const MatrixX<T>>& X) {
        return X.array().tanh().matrix();
      });
      dsigma_.push_back([](const Eigen::Ref<const MatrixX<T>>& X) {
        return (1.0 - X.array().tanh().square()).matrix();
      });
    } else if (activation_types_[i] == kReLU) {
      sigma_.push_back([](const Eigen::Ref<const MatrixX<T>>& X) {
        return X.array().max(0.0).matrix();
      });
      dsigma_.push_back([](const Eigen::Ref<const MatrixX<T>>& X) {
        return (X.array() <= 0).select(MatrixX<T>::Zero(X.rows(), X.cols()), 1);
      });
    } else {
      DRAKE_DEMAND(activation_types_[i] == kIdentity);
      sigma_.push_back([](const Eigen::Ref<const MatrixX<T>>& X) { return X; });
      dsigma_.push_back([](const Eigen::Ref<const MatrixX<T>>& X) {
        return MatrixX<T>::Ones(X.rows(), X.cols());
      });
    }
  }
  this->DeclareNumericParameter(
      BasicVector<T>(VectorX<T>::Zero(num_parameters_)));

  // Declare cache entry for CalcOutput.
  std::vector<VectorX<T>> hidden_layers;
  for (int i = 0; i < num_hidden_layers_; ++i) {
    hidden_layers.push_back(VectorX<T>::Zero(layers[i + 1]));
  }
  hidden_layer_cache_ =
      &this->DeclareCacheEntry("hidden_layer", hidden_layers,
                               &MultilayerPerceptron<T>::CalcHiddenLayers);

  // Declare cache entries for Backpropagation:
  // Wx_plus_b[i], Y[i], dloss_dY[i], dloss_dW_plus_b[i], and dloss_dW[i].
  std::vector<MatrixX<T>> backprop_data(5 * num_weights_);
  backprop_cache_ = &this->DeclareCacheEntry(
      "backprop", backprop_data, &MultilayerPerceptron<T>::BackpropCacheNoOp);
}

template <typename T>
template <typename U>
MultilayerPerceptron<T>::MultilayerPerceptron(
    const MultilayerPerceptron<U>& other)
    : MultilayerPerceptron<T>(other.layers(), other.activation_types_) {}

template <typename T>
const VectorX<T>& MultilayerPerceptron<T>::GetParameters(
    const Context<T>& context) const {
  return context.get_numeric_parameter(0).value();
}

template <typename T>
void MultilayerPerceptron<T>::SetRandomParameters(
    const Context<T>& context, Parameters<T>* parameters,
    RandomGenerator* generator) const {
  // TODO(russt): Consider more advanced approaches, e.g. Xavier initialization.
  unused(context);
  std::normal_distribution<double> normal(0.0, 0.01);
  BasicVector<T>& params = parameters->get_mutable_numeric_parameter(0);
  for (int i = 0; i < num_parameters_; i++) {
    params[i] = normal(*generator);
  }
}

template <typename T>
void MultilayerPerceptron<T>::SetParameters(
    Context<T>* context, const Eigen::Ref<const VectorX<T>>& params) const {
  DRAKE_DEMAND(params.rows() == num_parameters_);
  context->get_mutable_numeric_parameter(0).SetFromVector(params);
}

template <typename T>
Eigen::Map<const MatrixX<T>> MultilayerPerceptron<T>::GetWeights(
    const Context<T>& context, int layer) const {
  return GetWeights(context.get_numeric_parameter(0).value(), layer);
}

template <typename T>
Eigen::Map<const VectorX<T>> MultilayerPerceptron<T>::GetBiases(
    const Context<T>& context, int layer) const {
  return GetBiases(context.get_numeric_parameter(0).value(), layer);
}

template <typename T>
void MultilayerPerceptron<T>::SetWeights(
    Context<T>* context, int layer,
    const Eigen::Ref<const MatrixX<T>>& W) const {
  DRAKE_DEMAND(layer >= 0 && layer < num_weights_);
  DRAKE_DEMAND(W.rows() == layers_[layer + 1] && W.cols() == layers_[layer]);
  BasicVector<T>& params = context->get_mutable_numeric_parameter(0);
  Eigen::Map<MatrixX<T>>(
      params.get_mutable_value().data() + weight_indices_[layer],
      layers_[layer + 1], layers_[layer]) = W;
}

template <typename T>
void MultilayerPerceptron<T>::SetBiases(
    Context<T>* context, int layer,
    const Eigen::Ref<const VectorX<T>>& b) const {
  DRAKE_DEMAND(layer >= 0 && layer < num_weights_);
  DRAKE_DEMAND(b.rows() == layers_[layer + 1]);
  context->get_mutable_numeric_parameter(0).get_mutable_value().segment(
      bias_indices_[layer], layers_[layer + 1]) = b;
}

template <typename T>
Eigen::Map<const MatrixX<T>> MultilayerPerceptron<T>::GetWeights(
    const Eigen::Ref<const VectorX<T>>& params, int layer) const {
  DRAKE_DEMAND(layer >= 0 && layer < num_weights_);
  DRAKE_DEMAND(params.rows() == num_parameters_);
  return Eigen::Map<const MatrixX<T>>(params.data() + weight_indices_[layer],
                                      layers_[layer + 1], layers_[layer]);
}

template <typename T>
Eigen::Map<const VectorX<T>> MultilayerPerceptron<T>::GetBiases(
    const Eigen::Ref<const VectorX<T>>& params, int layer) const {
  DRAKE_DEMAND(layer >= 0 && layer < num_weights_);
  DRAKE_DEMAND(params.rows() == num_parameters_);
  return Eigen::Map<const VectorX<T>>(params.data() + bias_indices_[layer],
                                      layers_[layer + 1]);
}

template <typename T>
void MultilayerPerceptron<T>::SetWeights(
    EigenPtr<VectorX<T>> params, int layer,
    const Eigen::Ref<const MatrixX<T>>& W) const {
  DRAKE_DEMAND(layer >= 0 && layer < num_weights_);
  DRAKE_DEMAND(params->rows() == num_parameters_);
  DRAKE_DEMAND(W.rows() == layers_[layer + 1] && W.cols() == layers_[layer]);
  Eigen::Map<MatrixX<T>>(params->data() + weight_indices_[layer],
                         layers_[layer + 1], layers_[layer]) = W;
}

template <typename T>
void MultilayerPerceptron<T>::SetBiases(
    EigenPtr<VectorX<T>> params, int layer,
    const Eigen::Ref<const VectorX<T>>& b) const {
  DRAKE_DEMAND(layer >= 0 && layer < num_weights_);
  DRAKE_DEMAND(params->rows() == num_parameters_);
  DRAKE_DEMAND(b.rows() == layers_[layer + 1]);
  params->segment(bias_indices_[layer], layers_[layer + 1]) = b;
}

template <typename T>
double MultilayerPerceptron<T>::Backpropagation(
    const Context<T>& context, const Eigen::Ref<const MatrixXd>& X,
    std::function<double(const Eigen::Ref<const MatrixXd>& Y,
                         EigenPtr<MatrixXd> dloss_dY)>
        loss,
    EigenPtr<Eigen::VectorXd> dloss_dparams) const {
  if constexpr (!std::is_same_v<T, double>) {
    unused(context, X, loss, dloss_dparams);
    throw std::logic_error("Backpropagation only supports T=double.");
  } else {
    this->ValidateContext(context);
    DRAKE_DEMAND(X.rows() == layers_[0]);
    // Note: Should aim for zero dynamic allocations in here (except on the
    // first calls and whenever X changes size).
    auto& cache =
        backprop_cache_->get_mutable_cache_entry_value(context)
            .template GetMutableValueOrThrow<std::vector<MatrixX<T>>>();
    // TODO(russt): Try to reduce the amount of scratch memory.
    MatrixXd* Wx_plus_b = &cache[0];
    MatrixXd* Y = &cache[num_weights_];
    MatrixXd* dloss_dY = &cache[2 * num_weights_];
    MatrixXd* dloss_dWx_plus_b = &cache[3 * num_weights_];
    MatrixXd* dloss_dW = &cache[4 * num_weights_];
    // Forward pass:
    Wx_plus_b[0] =
        (GetWeights(context, 0) * X).colwise() + GetBiases(context, 0);
    Y[0] = sigma_[0](Wx_plus_b[0]);
    for (int i = 1; i < num_weights_; ++i) {
      Wx_plus_b[i] =
          (GetWeights(context, i) * Y[i - 1]).colwise() + GetBiases(context, i);
      Y[i] = sigma_[i](Wx_plus_b[i]);
    }
    dloss_dY[num_weights_ - 1].resize(layers_[num_weights_], X.cols());
    double l = loss(Y[num_weights_ - 1], &dloss_dY[num_weights_ - 1]);
    // Backward pass:
    for (int i = num_weights_ - 1; i >= 0; --i) {
      dloss_dWx_plus_b[i] =
          (dloss_dY[i].array() * dsigma_[i](Wx_plus_b[i]).array()).matrix();
      dloss_dW[i].resize(layers_[i + 1], layers_[i]);
      dloss_dW[i].setZero();
      for (int j = 0; j < X.cols(); ++j) {
        if (i > 0) {
          dloss_dW[i] +=
              dloss_dWx_plus_b[i].col(j) * Y[i - 1].col(j).transpose();
        } else {
          dloss_dW[i] += dloss_dWx_plus_b[i].col(j) * X.col(j).transpose();
        }
      }
      SetWeights(dloss_dparams, i, dloss_dW[i]);
      SetBiases(dloss_dparams, i, dloss_dWx_plus_b[i].rowwise().sum());
      if (i > 0) {
        dloss_dY[i - 1] =
            GetWeights(context, i).transpose() * dloss_dWx_plus_b[i];
      }
    }
    return l;
  }
}

template <typename T>
double MultilayerPerceptron<T>::BackpropagationMeanSquaredError(
    const Context<T>& context, const Eigen::Ref<const MatrixXd>& X,
    const Eigen::Ref<const MatrixXd>& Y_desired,
    EigenPtr<Eigen::VectorXd> dloss_dparams) const {
  auto MSE_loss = [Y_desired](const Eigen::Ref<const MatrixXd>& Y,
                              EigenPtr<MatrixXd> dloss_dY) {
    *dloss_dY = 2 * (Y - Y_desired) / Y.cols();
    return (Y - Y_desired).squaredNorm() / Y.cols();
  };
  return Backpropagation(context, X, MSE_loss, dloss_dparams);
}

template <typename T>
void MultilayerPerceptron<T>::CalcOutput(const Context<T>& context,
                                         BasicVector<T>* y) const {
  // TODO(russt): Test for and eliminate any dynamic allocations.
  this->ValidateContext(context);
  y->get_mutable_value() = sigma_[num_weights_ - 1](
      GetWeights(context, num_weights_ - 1) *
          hidden_layer_cache_->Eval<std::vector<VectorX<T>>>(
              context)[num_hidden_layers_ - 1] +
      GetBiases(context, num_weights_ - 1));
}

template <typename T>
void MultilayerPerceptron<T>::CalcHiddenLayers(
    const Context<T>& context, std::vector<VectorX<T>>* hidden) const {
  (*hidden)[0] =
      sigma_[0](GetWeights(context, 0) * this->get_input_port().Eval(context) +
                GetBiases(context, 0));
  for (int i = 1; i < num_hidden_layers_; ++i) {
    (*hidden)[i] = sigma_[i](GetWeights(context, i) * (*hidden)[i - 1] +
                             GetBiases(context, i));
  }
}

template <typename T>
void MultilayerPerceptron<T>::BackpropCacheNoOp(
    const Context<T>&, std::vector<MatrixX<T>>*) const {
  // Intentionally left blank. The values defining this computation are not in
  // the context, so we assign them directly in the backprop algorithm.
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::MultilayerPerceptron)
