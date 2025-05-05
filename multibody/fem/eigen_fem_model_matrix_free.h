#pragma once

#include "drake/multibody/fem/fem_model.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

class EigenFemModelMatrixFree;

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake

namespace Eigen {
namespace internal {

/* Gives the wrapper class a dense matrix trait so that Eigen chooses the GEMV
 multiplication in the CG solver. */
template <>
struct traits<drake::multibody::fem::internal::EigenFemModelMatrixFree>
    : traits<MatrixX<double>> {};

}  // namespace internal
}  // namespace Eigen

#include <Eigen/Core>

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

/* Wrapper class around FemModel that's compatible with
 * Eigen::ConjugateGradient. */
class EigenFemModelMatrixFree
    : public Eigen::EigenBase<EigenFemModelMatrixFree> {
 public:
  using Scalar = double;
  using RealScalar = double;
  using StorageIndex = int;

  enum {
    ColsAtCompileTime = Eigen::Dynamic,
    MaxColsAtCompileTime = Eigen::Dynamic,
    IsRowMajor = false
  };

  explicit EigenFemModelMatrixFree(const FemModel<double>* model,
                                   const FemState<double>* state,
                                   const Vector3<double>& weights)
      : model_(model), state_(state), weights_(weights) {}

  /* Required by EigenBase */
  Eigen::Index rows() const { return model_->num_dofs(); }
  Eigen::Index cols() const { return model_->num_dofs(); }

  /* Lazy‐Product: builds the expression A * x */
  template <typename Rhs>
  Eigen::Product<EigenFemModelMatrixFree, Rhs, Eigen::AliasFreeProduct>
  operator*(const Eigen::MatrixBase<Rhs>& x) const {
    return Eigen::Product<EigenFemModelMatrixFree, Rhs,
                          Eigen::AliasFreeProduct>(*this, x.derived());
  }

  /* Wraps around the CalcDifferential() function. */
  void Multiply(const VectorX<double>& x, VectorX<double>* y) const {
    model_->CalcDifferential(*state_, weights_, x, y);
  }

 private:
  const FemModel<double>* model_{};
  const FemState<double>* state_{};
  Vector3<double> weights_;
};

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake

namespace Eigen {
namespace internal {

/* Implements Eigen's GEMV with the custom Multiply. */
template <typename Rhs>
struct generic_product_impl<
    drake::multibody::fem::internal::EigenFemModelMatrixFree, Rhs,
    /* LHS kind = */ DenseShape,
    /* RHS kind = */ DenseShape,
    /* product = */ GemvProduct>
    : generic_product_impl_base<
          drake::multibody::fem::internal::EigenFemModelMatrixFree, Rhs,
          generic_product_impl<
              drake::multibody::fem::internal::EigenFemModelMatrixFree, Rhs>> {
  using Mat = drake::multibody::fem::internal::EigenFemModelMatrixFree;
  using Scalar = typename Mat::Scalar;

  template <typename Dest>
  // NOLINTNEXTLINE(runtime/references)
  static void scaleAndAddTo(Dest& dst, const Mat& A, Rhs const& x,
                            const Scalar&) {
    A.Multiply(x, &dst);
  }
};

}  // namespace internal
}  // namespace Eigen
