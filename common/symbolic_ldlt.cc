#include <stdexcept>

#include "drake/common/symbolic.h"

using E = drake::symbolic::Expression;
using MatrixXE = drake::MatrixX<E>;

namespace Eigen {

namespace {
// A free-function implementation of Eigen::LDLT<SomeMatrix>::compute, when
// Scalar is symbolic::Expression.  The output arguments correspond to the
// member fields of the LDLT object.  For simplicity, we only offer a single
// flavor without templates (i.e., with MaxRowsAtCompileTime == Dynamic).
void DoCompute(
    const Ref<const MatrixXE>& a,
    MatrixXE* matrix,
    E* l1_norm,
    Transpositions<Dynamic>* transpositions,
    internal::SignMatrix* sign,
    ComputationInfo* info) {
  if (!GetDistinctVariables(a).empty()) {
    throw std::logic_error("Symbolic LDLT is not supported yet");
  }
  const MatrixXd new_a =
      a.unaryExpr([](const E& e) { return drake::ExtractDoubleOrThrow(e); });
  auto ldlt = new_a.ldlt();
  *matrix = ldlt.matrixLDLT();
  *l1_norm = NAN;  // We could recompute this, if we really needed it.
  *transpositions = ldlt.transpositionsP();
  *sign =
      ldlt.isPositive() ? internal::PositiveSemiDef :
      ldlt.isNegative() ? internal::NegativeSemiDef :
      internal::Indefinite;
  *info = ldlt.info();
}
}  // namespace

#define DRAKE_DEFINE_SPECIALIZE_LDLT(SomeMatrix)                \
template <>                                                     \
template <>                                                     \
Eigen::LDLT<SomeMatrix>&                                        \
Eigen::LDLT<SomeMatrix>::compute<Ref<const SomeMatrix>>(        \
    const EigenBase<Ref<const SomeMatrix>>& a) {                \
  MatrixXE matrix;                                              \
  Transpositions<Dynamic> transpositions;                       \
  DoCompute(                                                    \
      a.derived(),                                              \
      &matrix,                                                  \
      &m_l1_norm,                                               \
      &transpositions,                                          \
      &m_sign,                                                  \
      &m_info);                                                 \
  m_matrix = matrix;                                            \
  m_transpositions = transpositions;                            \
  m_isInitialized = true;                                       \
  return *this;                                                 \
}

DRAKE_DEFINE_SPECIALIZE_LDLT(drake::MatrixX<drake::symbolic::Expression>)
DRAKE_DEFINE_SPECIALIZE_LDLT(drake::MatrixUpTo6<drake::symbolic::Expression>)

#undef DRAKE_DEFINE_SPECIALIZE_LDLT

}  // namespace Eigen
