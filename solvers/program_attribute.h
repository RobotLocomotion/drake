#pragma once

#include <ostream>
#include <string>
#include <unordered_set>

#include "drake/common/hash.h"

namespace drake {
namespace solvers {
enum class ProgramAttribute {
  kGenericCost,  ///< A generic cost, doesn't belong to any specific cost type
                 /// below.
  kGenericConstraint,  ///< A generic constraint, doesn't belong to any specific
                       /// constraint type below.

  kQuadraticCost,        ///< A quadratic function as the cost.
  kQuadraticConstraint,  ///< A constraint on a quadratic function.

  kLinearCost,                ///< A linear function as the cost.
  kLinearConstraint,          ///< A constraint on a linear function.
  kLinearEqualityConstraint,  ///< An equality constraint on a linear function.

  kLinearComplementarityConstraint,  ///< A linear complementarity constraint in
                                     /// the form 0 ≤ z ⊥ Mz+q ≥ 0.

  kLorentzConeConstraint,         ///< A Lorentz cone constraint.
  kRotatedLorentzConeConstraint,  ///< A rotated Lorentz cone constraint.

  kPositiveSemidefiniteConstraint,  /// A positive semidefinite constraint.

  kExponentialConeConstraint,  /// An exponential cone constraint.

  kBinaryVariable,  /// variable taking binary value {0, 1}.

  kCallback,  /// support callback during solving the problem.
};

using ProgramAttributes = std::unordered_set<ProgramAttribute, DefaultHash>;

/**
 * Returns true if @p required is a subset of @p supported.
 */
bool AreRequiredAttributesSupported(const ProgramAttributes& required,
                                    const ProgramAttributes& supported);

std::string to_string(const ProgramAttribute&);
std::ostream& operator<<(std::ostream&, const ProgramAttribute&);
std::string to_string(const ProgramAttributes&);
std::ostream& operator<<(std::ostream&, const ProgramAttributes&);

}  // namespace solvers
}  // namespace drake
