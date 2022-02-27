#define EIGEN_USE_BLAS
#include "drake/systems/primitives/multilayer_perceptron.h"

#include <limits>

#include "drake/common/default_scalars.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace systems {

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace internal {

template <typename T>
struct CalcLayersData {
  explicit CalcLayersData(int n) : Wx(n), Wx_plus_b(n), Xn(n) {}

  MatrixX<T> input_features;
  std::vector<VectorX<T>> Wx;
  std::vector<VectorX<T>> Wx_plus_b;
  std::vector<VectorX<T>> Xn;
};

}  // namespace internal

namespace {

std::vector<PerceptronActivationType> MakeDefaultActivations(
    int num, PerceptronActivationType activation_type) {
  std::vector<PerceptronActivationType> types(num, activation_type);
  types[num - 1] = kIdentity;
  return types;
}

const std::vector<int>& RejectEmpty(const std::vector<int>& arg) {
  if (arg.empty()) {
    throw std::logic_error(
        "The MultilayerPerceptron's layers constructor argument has too few "
        "elements.");
  }
  return arg;
}

template <typename T>
struct BackPropData {
  explicit BackPropData(int n)
      : Wx(n),
        Wx_plus_b(n),
        Xn(n),
        dXn_dWx_plus_b(n),
        dloss_dXn(n),
        dloss_dWx_plus_b(n),
        dloss_dW(n),
        dloss_db(n) {}

  std::vector<MatrixX<T>> Wx;
  std::vector<MatrixX<T>> Wx_plus_b;
  std::vector<MatrixX<T>> Xn;
  std::vector<MatrixX<T>> dXn_dWx_plus_b;
  std::vector<MatrixX<T>> dloss_dXn;
  std::vector<MatrixX<T>> dloss_dWx_plus_b;
  std::vector<MatrixX<T>> dloss_dW;
  std::vector<VectorX<T>> dloss_db;
  MatrixX<T> input_features;
  MatrixX<T> dloss_dinput_features;
};

template <typename T, int cols>
void Activation(
    PerceptronActivationType type,
    const Eigen::Ref<const Eigen::Matrix<T, Eigen::Dynamic, cols>>& X,
    Eigen::Matrix<T, Eigen::Dynamic, cols>* Y,
    Eigen::Matrix<T, Eigen::Dynamic, cols>* dYdX = nullptr) {
  Y->resize(X.rows(), X.cols());
  if (dYdX) {
    dYdX->resize(X.rows(), X.cols());
  }
  if (type == kTanh) {
    *Y = X.array().tanh().matrix();
    if (dYdX) {
      dYdX->noalias() = (1.0 - X.array().tanh().square()).matrix();
    }
  } else if (type == kReLU) {
    *Y = X.array().max(0.0).matrix();
    if (dYdX) {
      dYdX->noalias() = (X.array() <= 0).select(0 * X, 1);
    }
  } else {
    DRAKE_DEMAND(type == kIdentity);
    *Y = X;
    if (dYdX) {
      dYdX->setConstant(1.0);
    }
  }
}

}  // namespace

template <typename T>
MultilayerPerceptron<T>::MultilayerPerceptron(
    const std::vector<int>& layers, PerceptronActivationType activation_type)
    : MultilayerPerceptron<T>(
          layers, MakeDefaultActivations(RejectEmpty(layers).size() - 1,
                                         activation_type)) {}

template <typename T>
MultilayerPerceptron<T>::MultilayerPerceptron(
    const std::vector<int>& layers,
    const std::vector<PerceptronActivationType>& activation_types)
    : MultilayerPerceptron<T>(
          std::vector<bool>(RejectEmpty(layers)[0], false),
          std::vector<int>(RejectEmpty(layers).begin() + 1, layers.end()),
          activation_types) {}

template <typename T>
MultilayerPerceptron<T>::MultilayerPerceptron(
    const std::vector<bool>& use_sin_cos_for_input,
    const std::vector<int>& remaining_layers,
    const std::vector<PerceptronActivationType>& activation_types)
    : LeafSystem<T>(SystemTypeTag<MultilayerPerceptron>{}),
      num_weights_(remaining_layers.size()),
      layers_(remaining_layers.size() + 1),
      activation_types_(activation_types),
      use_sin_cos_for_input_(use_sin_cos_for_input),
      has_input_features_(false) {
  layers_[0] = use_sin_cos_for_input.size();
  for (bool use_sin_cos : use_sin_cos_for_input) {
    if (use_sin_cos) {
      ++layers_[0];
      has_input_features_ = true;
    }
  }
  for (int i = 0; i < static_cast<int>(remaining_layers.size()); ++i) {
    layers_[i + 1] = remaining_layers[i];
  }
  DRAKE_DEMAND(num_weights_ >= 1);
  DRAKE_DEMAND(activation_types_.size() == layers_.size() - 1);
  for (int units_in_layer : layers_) {
    DRAKE_DEMAND(units_in_layer > 0);
  }
  this->DeclareVectorInputPort("x", use_sin_cos_for_input.size());
  this->DeclareVectorOutputPort("y", layers_[num_weights_],
                                &MultilayerPerceptron<T>::CalcOutput);

  num_parameters_ = 0;
  weight_indices_.reserve(num_weights_);
  bias_indices_.reserve(num_weights_);
  for (int i = 0; i < num_weights_; ++i) {
    weight_indices_[i] = num_parameters_;
    num_parameters_ += layers_[i + 1] * layers_[i];
    bias_indices_[i] = num_parameters_;
    num_parameters_ += layers_[i + 1];
  }
  this->DeclareNumericParameter(
      BasicVector<T>(VectorX<T>::Zero(num_parameters_)));

  // Declare cache entry for CalcOutput.
  internal::CalcLayersData<T> calc_layers_data(num_weights_);
  for (int i = 0; i < num_weights_; ++i) {
    calc_layers_data.Wx[i] = VectorX<T>::Zero(layers_[i + 1]);
    calc_layers_data.Wx_plus_b[i] = VectorX<T>::Zero(layers_[i + 1]);
    calc_layers_data.Xn[i] = VectorX<T>::Zero(layers_[i + 1]);
  }
  calc_layers_cache_ = &this->DeclareCacheEntry(
      "calc_layers", calc_layers_data, &MultilayerPerceptron<T>::CalcLayers);

  // Declare cache entry for Backpropagation:
  BackPropData<T> backprop_data(num_weights_);
  backprop_cache_ = &this->DeclareCacheEntry(
      "backprop", ValueProducer(backprop_data, &ValueProducer::NoopCalc));
}

template <typename T>
template <typename U>
MultilayerPerceptron<T>::MultilayerPerceptron(
    const MultilayerPerceptron<U>& other)
    : MultilayerPerceptron<T>(
          other.use_sin_cos_for_input_,
          std::vector<int>(other.layers().begin() + 1, other.layers().end()),
          other.activation_types_) {}

template <typename T>
const VectorX<T>& MultilayerPerceptron<T>::GetParameters(
    const Context<T>& context) const {
  return context.get_numeric_parameter(0).value();
}

template <typename T>
Eigen::VectorBlock<VectorX<T>> MultilayerPerceptron<T>::GetMutableParameters(
    Context<T>* context) const {
  return context->get_mutable_numeric_parameter(0).get_mutable_value();
}

template <typename T>
void MultilayerPerceptron<T>::SetRandomParameters(
    const Context<T>&, Parameters<T>* parameters,
    RandomGenerator* generator) const {
  std::uniform_real_distribution<double> uniform(-1.0, 1.0);
  BasicVector<T>& params = parameters->get_mutable_numeric_parameter(0);
  for (int i = 0; i < num_weights_; ++i) {
    // We choose m here so that uniform(-m,m) has the desired standard
    // deviation √1/n, where n is the number of incoming connections.  The
    // factor of √3 can be derived from the variance of uniform(a,b), which is
    // (b-a)²/12.
    double m = std::sqrt(3.0 / layers_[i]);
    for (int n = weight_indices_[i];
         n < weight_indices_[i] + layers_[i + 1] * layers_[i]; ++n) {
      params[n] = m * uniform(*generator);
    }
    for (int n = bias_indices_[i]; n < bias_indices_[i] + layers_[i + 1]; ++n) {
      params[n] = m * uniform(*generator);
    }
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
  DRAKE_DEMAND(W.rows() == layers_[layer + 1]);
  DRAKE_DEMAND(W.cols() == layers_[layer]);
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
  DRAKE_DEMAND(W.rows() == layers_[layer + 1]);
  DRAKE_DEMAND(W.cols() == layers_[layer]);
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
T MultilayerPerceptron<T>::Backpropagation(
    const Context<T>& context, const Eigen::Ref<const MatrixX<T>>& X,
    const std::function<T(const Eigen::Ref<const MatrixX<T>>& Y,
                          EigenPtr<MatrixX<T>> dloss_dY)>& loss,
    EigenPtr<VectorX<T>> dloss_dparams) const {
  this->ValidateContext(context);
  DRAKE_DEMAND(X.rows() == this->get_input_port().size());
  DRAKE_DEMAND(dloss_dparams->rows() == num_parameters_);
  BackPropData<T>& data =
      backprop_cache_->get_mutable_cache_entry_value(context)
          .template GetMutableValueOrThrow<BackPropData<T>>();
  // Forward pass:
  if (has_input_features_) {
    CalcInputFeatures(X, &data.input_features);
    data.Wx[0].noalias() = GetWeights(context, 0) * data.input_features;
  } else {
    data.Wx[0].noalias() = GetWeights(context, 0) * X;
  }
  data.Wx_plus_b[0].noalias() = data.Wx[0].colwise() + GetBiases(context, 0);
  Activation<T, Eigen::Dynamic>(activation_types_[0], data.Wx_plus_b[0],
                                &data.Xn[0], &data.dXn_dWx_plus_b[0]);
  for (int i = 1; i < num_weights_; ++i) {
    data.Wx[i].noalias() = GetWeights(context, i) * data.Xn[i - 1];
    data.Wx_plus_b[i].noalias() = data.Wx[i].colwise() + GetBiases(context, i);
    Activation<T, Eigen::Dynamic>(activation_types_[i], data.Wx_plus_b[i],
                                  &data.Xn[i], &data.dXn_dWx_plus_b[i]);
  }
  data.dloss_dXn[num_weights_ - 1].resize(layers_[num_weights_], X.cols());
  data.dloss_dXn[num_weights_ - 1].setConstant(
      std::numeric_limits<typename Eigen::NumTraits<T>::Literal>::quiet_NaN());
  const T l =
      loss(data.Xn[num_weights_ - 1], &data.dloss_dXn[num_weights_ - 1]);
  // Backward pass:
  for (int i = num_weights_ - 1; i >= 0; --i) {
    data.dloss_dWx_plus_b[i] =
        (data.dloss_dXn[i].array() * data.dXn_dWx_plus_b[i].array()).matrix();
    data.dloss_dW[i].resize(layers_[i + 1], layers_[i]);
    data.dloss_dW[i].setZero();
    for (int j = 0; j < X.cols(); ++j) {
      if (i > 0) {
        data.dloss_dW[i].noalias() +=
            data.dloss_dWx_plus_b[i].col(j) * data.Xn[i - 1].col(j).transpose();
      } else if (has_input_features_) {
        data.dloss_dW[i].noalias() += data.dloss_dWx_plus_b[i].col(j) *
                                      data.input_features.col(j).transpose();
      } else {
        data.dloss_dW[i].noalias() +=
            data.dloss_dWx_plus_b[i].col(j) * X.col(j).transpose();
      }
    }
    SetWeights(dloss_dparams, i, data.dloss_dW[i]);
    data.dloss_db[i] = data.dloss_dWx_plus_b[i].rowwise().sum();
    SetBiases(dloss_dparams, i, data.dloss_db[i]);
    if (i > 0) {
      data.dloss_dXn[i - 1].noalias() =
          GetWeights(context, i).transpose() * data.dloss_dWx_plus_b[i];
    }
  }
  return l;
}

template <typename T>
T MultilayerPerceptron<T>::BackpropagationMeanSquaredError(
    const Context<T>& context, const Eigen::Ref<const MatrixX<T>>& X,
    const Eigen::Ref<const MatrixX<T>>& Y_desired,
    EigenPtr<VectorX<T>> dloss_dparams) const {
  DRAKE_DEMAND(Y_desired.rows() == layers_[num_weights_]);
  DRAKE_DEMAND(Y_desired.cols() == X.cols());
  DRAKE_DEMAND(Y_desired.rows() == layers_[num_weights_]);
  DRAKE_DEMAND(Y_desired.cols() == X.cols());
  auto MSE_loss = [&Y_desired](const Eigen::Ref<const MatrixX<T>>& Y,
                               EigenPtr<MatrixX<T>> dloss_dY) {
    dloss_dY->noalias() = 2.0 * (Y - Y_desired) / Y.cols();
    return (Y - Y_desired).squaredNorm() / Y.cols();
  };
  return Backpropagation(context, X, MSE_loss, dloss_dparams);
}

template <typename T>
void MultilayerPerceptron<T>::BatchOutput(const Context<T>& context,
                                          const Eigen::Ref<const MatrixX<T>>& X,
                                          EigenPtr<MatrixX<T>> Y,
                                          EigenPtr<MatrixX<T>> dYdX) const {
  this->ValidateContext(context);
  DRAKE_DEMAND(X.rows() == this->get_input_port().size());
  DRAKE_DEMAND(Y->rows() == layers_[num_weights_]);
  DRAKE_DEMAND(Y->cols() == X.cols());
  const bool gradients = dYdX != nullptr;
  if (gradients && layers_[num_weights_] != 1) {
    throw std::logic_error(
        "BatchOutput: dYdX != nullptr, but BatchOutput only supports gradients "
        "when the output layer has size 1.");
  }

  BackPropData<T>& data =
      backprop_cache_->get_mutable_cache_entry_value(context)
          .template GetMutableValueOrThrow<BackPropData<T>>();
  // Forward pass:
  if (has_input_features_) {
    CalcInputFeatures(X, &data.input_features);
    data.Wx[0].noalias() = GetWeights(context, 0) * data.input_features;
  } else {
    data.Wx[0].noalias() = GetWeights(context, 0) * X;
  }
  data.Wx_plus_b[0].noalias() = data.Wx[0].colwise() + GetBiases(context, 0);
  Activation<T, Eigen::Dynamic>(activation_types_[0], data.Wx_plus_b[0],
                                &data.Xn[0],
                                gradients ? &data.dXn_dWx_plus_b[0] : nullptr);
  for (int i = 1; i < num_weights_; ++i) {
    data.Wx[i].noalias() = GetWeights(context, i) * data.Xn[i - 1];
    data.Wx_plus_b[i].noalias() = data.Wx[i].colwise() + GetBiases(context, i);
    Activation<T, Eigen::Dynamic>(
        activation_types_[i], data.Wx_plus_b[i], &data.Xn[i],
        gradients ? &data.dXn_dWx_plus_b[i] : nullptr);
  }
  *Y = data.Xn[num_weights_ - 1];
  if (gradients) {
    // Backward pass:
    // In order to reuse the cache from Backprop, we take loss ≡ Y.
    data.dloss_dXn[num_weights_ - 1] = RowVectorX<T>::Constant(X.cols(), 1.0);
    for (int i = num_weights_ - 1; i >= 0; --i) {
      data.dloss_dWx_plus_b[i] =
          (data.dloss_dXn[i].array() * data.dXn_dWx_plus_b[i].array()).matrix();
      if (i > 0) {
        data.dloss_dXn[i - 1].noalias() =
            GetWeights(context, i).transpose() * data.dloss_dWx_plus_b[i];
      } else if (has_input_features_) {
        data.dloss_dinput_features.noalias() =
            GetWeights(context, 0).transpose() * data.dloss_dWx_plus_b[0];
        int feature_row = 0, input_row = 0;
        for (bool use_sin_cos : use_sin_cos_for_input_) {
          if (use_sin_cos) {
            dYdX->row(input_row) =
                data.dloss_dinput_features.row(feature_row).array() *
                    X.row(input_row).array().cos() -
                data.dloss_dinput_features.row(feature_row + 1).array() *
                    X.row(input_row).array().sin();
            feature_row += 2;
            ++input_row;
          } else {
            dYdX->row(input_row++) =
                data.dloss_dinput_features.row(feature_row++);
          }
        }
      } else {
        dYdX->noalias() =
            GetWeights(context, 0).transpose() * data.dloss_dWx_plus_b[i];
      }
    }
  }
}

template <typename T>
void MultilayerPerceptron<T>::CalcOutput(const Context<T>& context,
                                         BasicVector<T>* y) const {
  this->ValidateContext(context);
  y->get_mutable_value() =
      calc_layers_cache_->Eval<internal::CalcLayersData<T>>(context)
          .Xn[num_weights_ - 1];
}

template <typename T>
void MultilayerPerceptron<T>::CalcLayers(
    const Context<T>& context, internal::CalcLayersData<T>* data) const {
  if (has_input_features_) {
    CalcInputFeatures(this->get_input_port().Eval(context),
                      &data->input_features);
    data->Wx[0].noalias() = GetWeights(context, 0) * data->input_features;
  } else {
    data->Wx[0].noalias() =
        GetWeights(context, 0) * this->get_input_port().Eval(context);
  }
  data->Wx_plus_b[0].noalias() = data->Wx[0].colwise() + GetBiases(context, 0);
  Activation<T, 1>(activation_types_[0], data->Wx_plus_b[0], &(data->Xn[0]));
  for (int i = 1; i < num_weights_; ++i) {
    data->Wx[i].noalias() = GetWeights(context, i) * data->Xn[i - 1];
    data->Wx_plus_b[i].noalias() =
        data->Wx[i].colwise() + GetBiases(context, i);
    Activation<T, 1>(activation_types_[i], data->Wx_plus_b[i], &(data->Xn[i]));
  }
}

template <typename T>
void MultilayerPerceptron<T>::CalcInputFeatures(
    const Eigen::Ref<const MatrixX<T>>& X, MatrixX<T>* input_features) const {
  input_features->resize(layers_[0], X.cols());
  int feature_row = 0, input_row = 0;
  for (bool use_sin_cos : use_sin_cos_for_input_) {
    if (use_sin_cos) {
      input_features->row(feature_row++) = X.row(input_row).array().sin();
      input_features->row(feature_row++) = X.row(input_row++).array().cos();
    } else {
      input_features->row(feature_row++) = X.row(input_row++);
    }
  }
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::MultilayerPerceptron)
