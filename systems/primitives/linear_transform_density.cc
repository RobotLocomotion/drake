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
      this->DeclareInputPort("w_in", kVectorValued, input_size_).get_index();
  A_port_id_ =
      this->DeclareInputPort("A", kVectorValued, input_size_ * output_size_)
          .get_index();
  b_port_id_ =
      this->DeclareInputPort("b", kVectorValued, output_size_).get_index();

  this->DeclareVectorOutputPort("w_out", BasicVector<T>(output_size_),
                                &LinearTransformDensity<T>::CalcOutput);
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
  const auto w_in = this->get_input_port_w_in().Eval(context);
  const Eigen::VectorBlock<const VectorX<T>> A_flat =
      this->get_input_port_A().Eval(context);
  const Eigen::Map<const MatrixX<T>> A(A_flat.data(), output_size_,
                                       input_size_);
  if (!this->get_input_port_b().HasValue(context)) {
    // If port b is not connected, then the b value is default to 0.
    w_out->get_mutable_value() = A * w_in;
  } else {
    const auto b = this->get_input_port_b().Eval(context);
    w_out->get_mutable_value() = A * w_in + b;
  }
}

template <typename T>
FixedInputPortValue& LinearTransformDensity<T>::FixConstantA(
    Context<T>* context, const Eigen::Ref<const MatrixX<T>>& A) const {
  return this->get_input_port_A().FixValue(
      context,
      Eigen::Map<const VectorX<T>>(A.data(), input_size_ * output_size_));
}

template <typename T>
FixedInputPortValue& LinearTransformDensity<T>::FixConstantB(
    Context<T>* context, const Eigen::Ref<const VectorX<T>>& b) const {
  return this->get_input_port_b().FixValue(context, b);
}

template <typename T>
T LinearTransformDensity<T>::CalcDensity(const Context<T>& context) const {
  // Refer to Theorem 2.14 in http://parker.ad.siu.edu/Olive/ich2.pdf for
  // computing the density of multivariate random variables after
  // transformation.
  // The density is pr(A⁻¹(w_out - b)) / |det(A)|
  const Eigen::VectorBlock<const VectorX<T>> A_flat =
      this->get_input_port_A().Eval(context);
  const Eigen::Map<const MatrixX<T>> A(A_flat.data(), output_size_,
                                       input_size_);

  // Check if A is invertible.
  if (input_size_ == output_size_) {
    Eigen::ColPivHouseholderQR<MatrixX<T>> qr_solver;
    qr_solver.compute(A);
    if (qr_solver.isInvertible()) {
      BasicVector<T> w_out(output_size_);
      this->CalcOutput(context, &w_out);
      // Now take only the value of w_out. Ignore its gradient because we only
      // observe the value of the output sample w_out. If we were to include the
      // gradient of w_out = A_in * w_in + b, then when we compute w_in=
      // A⁻¹(w_out - b), the gradient of ∂w_in/ ∂A and ∂w_in/ ∂b would be zero,
      // this zero gradient isn't what we want.
      VectorX<T> w_out_val(output_size_);
      for (int i = 0; i < output_size_; ++i) {
        w_out_val(i) = T(ExtractDoubleOrThrow(w_out.get_value()(i)));
      }
      VectorX<T> w_in;
      if (this->get_input_port_b().HasValue(context)) {
        const auto b = this->get_input_port_b().Eval(context);
        w_in = qr_solver.solve(w_out_val - b);
      } else {
        // b = 0 when the port is not connected.
        w_in = qr_solver.solve(w_out_val);
      }
      // The pdf of the output is
      // pdf(w_in) / |det(A)|
      using std::abs;
      const T det_abs = qr_solver.absDeterminant();
      switch (distribution_) {
        case RandomDistribution::kUniform: {
          if ((w_in.array() < 0).any() || (w_in.array() > 1).any()) {
            return T(0.);
          } else {
            return T(1.) / det_abs;
          }
        }
        case RandomDistribution::kExponential: {
          if ((w_in.array() < 0).any()) {
            return T(0.);
          } else {
            // The pdf for an exponential distribution is exp(-x)
            // For multivariate exponential distribution, we just multiply its
            // pdf along each dimension to get the joint pdf on the multivariate
            // random variable.
            return (-w_in.array()).exp().prod() / det_abs;
          }
        }
        case RandomDistribution::kGaussian: {
          // The pdf for a single unit Gaussian is exp(-0.5 x²) / sqrt(2π).
          using std::sqrt;
          return ((-0.5 * w_in.array() * w_in.array()).exp() / sqrt(2 * M_PI))
                     .prod() /
                 det_abs;
        }
      }
    }
  }
  throw std::runtime_error(
      "CalcDensity: to compute the density, we require the matrix A being "
      "invertible.");
}
}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::systems::LinearTransformDensity)
