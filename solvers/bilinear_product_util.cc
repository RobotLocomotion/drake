#include "drake/solvers/bilinear_product_util.h"

#include "drake/common/symbolic/replace_bilinear_terms.h"

namespace drake {
namespace solvers {

symbolic::Expression ReplaceBilinearTerms(
    const symbolic::Expression& e,
    const Eigen::Ref<const VectorXDecisionVariable>& x,
    const Eigen::Ref<const VectorXDecisionVariable>& y,
    const Eigen::Ref<const MatrixX<symbolic::Expression>>& W) {
  return symbolic::ReplaceBilinearTerms(e, x, y, W);
}
}  // namespace solvers
}  // namespace drake
