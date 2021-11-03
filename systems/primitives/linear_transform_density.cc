#include "drake/systems/primitives/linear_transform_density.h"

#include "drake/common/default_scalars.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace systems {

template <typename T>
LinearTransformDensity<T>::LinearTransformDensity(
    RandomDistribution distribution, int input_size, int output_size)
    : LeafSystem<T>(SystemTypeTag<LinearTransformDensity>{}),
      distribution_{distribution},
      input_size_{input_size},
      output_size_{output_size} {
  w_in_port_id_ =
      this->DeclareInputPort("w_in", kVectorValued, input_size_, distribution)
          .get_index();
  A_port_id_ =
      this->DeclareInputPort("A", kVectorValued, input_size_ * output_size_)
          .get_index();
  b_port_id_ =
      this->DeclareInputPort("b", kVectorValued, output_size_).get_index();

  w_out_port_id_ =
      this->DeclareVectorOutputPort("w_out", output_size_,
                                    &LinearTransformDensity<T>::CalcOutput)
          .get_index();
  w_out_density_port_id_ =
      this->DeclareVectorOutputPort(
              "w_out_density", 1, &LinearTransformDensity<T>::CalcOutputDensity)
          .get_index();
}

template <typename T>
template <typename U>
LinearTransformDensity<T>::LinearTransformDensity(
    const LinearTransformDensity<U>& other)
    : LinearTransformDensity<T>(other.get_distribution(),
                                other.get_input_port_w_in().size(),
                                other.get_input_port_b().size()) {}

template <typename T>
void LinearTransformDensity<T>::CalcOutput(const Context<T>& context,
                                           BasicVector<T>* w_out) const {
  this->ValidateContext(context);
  const auto w_in = this->get_input_port_w_in().Eval(context);
  const Eigen::Map<const MatrixX<T>> A = this->GetA(context);
  if (!this->get_input_port_b().HasValue(context)) {
    // If port b is not connected, then the b value is default to 0.
    w_out->get_mutable_value() = A * w_in;
  } else {
    const auto b = this->get_input_port_b().Eval(context);
    w_out->get_mutable_value() = A * w_in + b;
  }
}

template <typename T>
void LinearTransformDensity<T>::CalcOutputDensity(
    const Context<T>& context, BasicVector<T>* w_out_density) const {
  w_out_density->get_mutable_value()(0) = CalcDensity(context);
}

template <typename T>
Eigen::Map<const MatrixX<T>> LinearTransformDensity<T>::GetA(
    const Context<T>& context) const {
  const VectorX<T>& A_flat = this->get_input_port_A().Eval(context);
  return Eigen::Map<const MatrixX<T>>(A_flat.data(), output_size_, input_size_);
}

template <typename T>
FixedInputPortValue& LinearTransformDensity<T>::FixConstantA(
    Context<T>* context, const Eigen::Ref<const MatrixX<T>>& A) const {
  this->ValidateContext(context);
  DRAKE_DEMAND(A.rows() == output_size_ && A.cols() == input_size_);
  return this->get_input_port_A().FixValue(
      context,
      Eigen::Map<const VectorX<T>>(A.data(), input_size_ * output_size_));
}

template <typename T>
FixedInputPortValue& LinearTransformDensity<T>::FixConstantB(
    Context<T>* context, const Eigen::Ref<const VectorX<T>>& b) const {
  this->ValidateContext(context);
  return this->get_input_port_b().FixValue(context, b);
}

template <typename T>
T LinearTransformDensity<T>::CalcDensity(const Context<T>& context) const {
  this->ValidateContext(context);
  // Refer to Theorem 2.14 in http://parker.ad.siu.edu/Olive/ich2.pdf for
  // computing the density of multivariate random variables after
  // transformation.
  // The density is pr(A⁻¹(w_out - b)) / |det(A)|
  // Note that in theorem 2.14 of the cited document, t(w_in) = w_out = A * w_in
  // + b, hence t⁻¹(w_out) = A⁻¹(w_out - b), and J = det(A⁻¹) = (det(A))⁻¹
  const Eigen::Map<const MatrixX<T>> A = this->GetA(context);

  // Check if A is invertible.
  if (input_size_ != output_size_) {
    throw std::runtime_error(
        "CalcDensity: to compute the density, we require the matrix A being "
        "invertible.");
  } else {
    Eigen::ColPivHouseholderQR<MatrixX<T>> qr_solver;
    qr_solver.compute(A);
    if (!qr_solver.isInvertible()) {
      throw std::runtime_error(
          "CalcDensity: to compute the density, we require the matrix A being "
          "invertible.");
    } else {
      BasicVector<T> w_out(output_size_);
      this->CalcOutput(context, &w_out);
      // Compute the following quantities:
      //
      //  w_out = A * w_in + b
      //  w_out_nograd = w_out.cast<double>();
      //  w_in_grad = A⁻¹ * (w_out_nograd - b)
      //  pr(w_out) = pr(w_in_grad) * det(A⁻¹)
      //
      // Note that w_out_nograd discards the gradient information. We want to
      // fix the output to this sampled value, and compute the gradient of the
      // probability for taking this fixed output sample.
      VectorX<T> w_out_nograd(output_size_);
      for (int i = 0; i < output_size_; ++i) {
        w_out_nograd(i) = T(ExtractDoubleOrThrow(w_out.get_value()(i)));
      }
      VectorX<T> w_in;
      if (this->get_input_port_b().HasValue(context)) {
        const auto b = this->get_input_port_b().Eval(context);
        w_in = qr_solver.solve(w_out_nograd - b);
      } else {
        // b = 0 when the port is not connected.
        w_in = qr_solver.solve(w_out_nograd);
      }
      // The pdf of the output is
      // pdf(w_in) / |det(A)|
      const T det_abs = qr_solver.absDeterminant();
      const T prob_w_in = CalcProbabilityDensity<T>(distribution_, w_in);
      return prob_w_in / det_abs;
    }
  }
}
}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::systems::LinearTransformDensity)
