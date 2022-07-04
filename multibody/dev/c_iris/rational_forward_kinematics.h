#pragma once

#include "drake/common/symbolic.h"

namespace drake {
namespace multibody {
namespace c_iris {
/** If e is a multilinear polynomial of cosθ and sinθ, and no
 * cosθᵢ and sinθᵢ appear in the same monomial, then we replace
 * cosθᵢ with (1-tᵢ²)/(1+tᵢ²), and sinθᵢ with 2tᵢ/(1+tᵢ²), and get a rational
 * polynomial of t.
 * @param e The symbolic polynomial in which cosθ and sinθ will be replaced.
 * @param cos_vars cos_vars(i) is cosθᵢ as documented above.
 * @param sin_vars sin_vars(i) is sinθᵢ as documented above.
 * @param t New variables to express cos and sin as rationals of t. tᵢ =
 * tan(θᵢ/2). @pre t.rows() == cos.rows() == sin.rows().
 * @param[out] e_rational The rational polynomial of e after replacement.
 */
void ReplaceCosAndSinWithRationalFunction(
    const symbolic::Polynomial& e, const VectorX<symbolic::Variable>& cos_vars,
    const VectorX<symbolic::Variable>& sin_vars,
    const VectorX<symbolic::Variable>& t,
    symbolic::RationalFunction* e_rational);

}  // namespace c_iris
}  // namespace multibody
}  // namespace drake
