#pragma once

#ifndef DRAKE_COMMON_SYMBOLIC_HEADER
#error Do not directly include this file. Include "drake/common/symbolic.h".
#endif

#include <Eigen/Core>

/// @file
/// Eigen::LDLT is specialized for drake::symbolic::Expression, for certain
/// matrix sizes.  If the expression matrix is all constants, it returns the
/// robust decomposition (as per Eigen's algorithm).  If there are unbound
/// variables, currently throws an exception (but may support this in the
/// future).

#if !defined(DRAKE_DOXYGEN_CXX)
namespace Eigen {

// Provide explicit (full) template specialization for LDLT::compute when the
// Scalar is symbolic::Expression.  We need to list out all of the different
// matrix sizes that Drake uses symbolically, because we cannot partially
// specialize a template class template method.
#define DRAKE_DECLARE_SPECIALIZE_LDLT(SomeMatrix)               \
template <>                                                     \
template <>                                                     \
Eigen::LDLT<SomeMatrix>&                                        \
Eigen::LDLT<SomeMatrix>::compute<Ref<const SomeMatrix>>(        \
    const EigenBase<Ref<const SomeMatrix>>&);                   \
template <>                                                     \
template <>                                                     \
inline                                                          \
Eigen::LDLT<SomeMatrix>&                                        \
Eigen::LDLT<SomeMatrix>::compute<SomeMatrix>(                   \
    const EigenBase<SomeMatrix>& a) {                           \
  const Ref<const SomeMatrix> ref_a(a.derived());               \
  return compute(ref_a);                                        \
}                                                               \

DRAKE_DECLARE_SPECIALIZE_LDLT(drake::MatrixX<drake::symbolic::Expression>)
DRAKE_DECLARE_SPECIALIZE_LDLT(drake::MatrixUpTo6<drake::symbolic::Expression>)

#undef DRAKE_DECLARE_SPECIALIZE_LDLT

}  // namespace Eigen
#endif  // !defined(DRAKE_DOXYGEN_CXX)
