#pragma once

#include <memory>
#include <set>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/contact_solvers/pooled_sap/sap_data.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace pooled_sap {

/* A model of the SAP problem.

[Castro et al., 2021] Castro A., Permenter F. and Han X., 2021. An Unconstrained
Convex Formulation of Compliant Contact. Available at
https://arxiv.org/abs/2110.10107 */
template <typename T>
class SapUnilateralConstraint {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(SapUnilateralConstraint);

  // Default constructor to place into STL containers.
  // Corresponds to a zero-cost constraint.
  SapUnilateralConstraint() = default;

  int num_cliques() const;
  int num_constraint_equations() const;

  // Along other quantities, this will compute Jᵀ⋅G(vc)⋅J. This will allow uso
  // optimize for the very common case of diagonal G and identity J (e.g. limit
  // constraint.)
  void CalcData(const VectorX<T>& vc, SapConstraintData<T>* data) const;

 private:
};

}  // namespace pooled_sap
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
