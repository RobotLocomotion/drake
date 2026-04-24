#include "drake/common/ad/auto_diff.h"

namespace drake {
namespace ad {
namespace internal {
namespace {

template <typename T1, typename T2>
AutoDiff GenericGevv(const Eigen::Ref<const VectorX<T1>, 0, StrideX>& a,
                     const Eigen::Ref<const VectorX<T2>, 0, StrideX>& b) {
  DRAKE_ASSERT(a.size() == b.size());
  AutoDiff result = a.coeffRef(0) * b.coeffRef(0);
  for (int i = 1; i < a.size(); ++i) {
    const AutoDiff& a_i = a.coeffRef(i);
    const AutoDiff& b_i = b.coeffRef(i);
    // Spell out `result += a_i * b_i` with a combined multiply-accumulate.
    // ∂/∂x result + (a * b) = result' + (a * b)' = result' + ba' + ab'
    result.partials().AddScaled(b_i.value(), a_i.partials());
    result.partials().AddScaled(a_i.value(), b_i.partials());
    result.value() += a_i.value() * b_i.value();
  }
  return result;
}

template <typename T1, typename T2>
void GenericGemm(const Eigen::Ref<const MatrixX<T1>, 0, StrideX>& left,
                 const Eigen::Ref<const MatrixX<T2>, 0, StrideX>& right,
                 EigenPtr<MatrixX<AutoDiff>> result) {
  // These checks are guaranteed by our header file functions that call us.
  DRAKE_ASSERT(result != nullptr);
  DRAKE_ASSERT(result->rows() == left.rows());
  DRAKE_ASSERT(result->cols() == right.cols());
  DRAKE_ASSERT(left.cols() == right.rows());

  // Multiplying Mx0 matrix * 0xN matrix produces a MxN zero matrix without any
  // partials.
  if (left.cols() == 0) {
    result->setZero();
    return;
  }

  // Delegate to Gevv.
  for (int i = 0; i < result->rows(); ++i) {
    for (int j = 0; j < result->cols(); ++j) {
      (*result)(i, j) = GenericGevv<T1, T2>(left.row(i), right.col(j));
    }
  }
}

}  // namespace

template <bool reverse>
void Gemm<reverse>::CalcAA(const MatrixRef<AutoDiff>& A,
                           const MatrixRef<AutoDiff>& B,
                           EigenPtr<MatrixX<AutoDiff>> result) {
  if constexpr (!reverse) {
    GenericGemm<AutoDiff, AutoDiff>(A, B, result);
  } else {
    GenericGemm<AutoDiff, AutoDiff>(B, A, result);
  }
}

template struct Gemm<false>;
template struct Gemm<true>;

}  // namespace internal
}  // namespace ad
}  // namespace drake
