#pragma once

// TODO(2026-06-01): Remove ostream header when `operator<<` is removed.
#include <ostream>
#include <string>
#include <unordered_set>

#include "drake/common/drake_deprecated.h"
#include "drake/common/fmt.h"
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

  kPositiveSemidefiniteConstraint,  ///< A positive semidefinite constraint.

  kExponentialConeConstraint,  ///< An exponential cone constraint.

  kL2NormCost,  ///< An L2 norm |Ax+b|

  kBinaryVariable,  ///< Variable taking binary value {0, 1}.

  kCallback,  ///< Supports callback during solving the problem.
};

using ProgramAttributes = std::unordered_set<ProgramAttribute, DefaultHash>;

/** Returns true iff @p required is a subset of @p supported.

@param[out] unsupported_message (Optional) When provided, if this function
returns false, the message will be set to a phrase describing the unsupported
attributes; or if this function returns true, the message will be set to the
empty string.
*/
bool AreRequiredAttributesSupported(const ProgramAttributes& required,
                                    const ProgramAttributes& supported,
                                    std::string* unsupported_message = nullptr);

std::string to_string(const ProgramAttribute&);

DRAKE_DEPRECATED(
    "2026-06-01",
    "Use fmt functions instead (e.g., fmt::format(), fmt::to_string(), "
    "fmt::print()). Refer to GitHub issue #17742 for more information.")
std::ostream& operator<<(std::ostream&, const ProgramAttribute&);

std::string to_string(const ProgramAttributes&);

DRAKE_DEPRECATED(
    "2026-06-01",
    "Use fmt functions instead (e.g., fmt::format(), fmt::to_string(), "
    "fmt::print()). Refer to GitHub issue #17742 for more information.")
std::ostream& operator<<(std::ostream&, const ProgramAttributes&);

/**
 * A coarse categorization of the optimization problem based on the type of
 * constraints/costs/variables.
 * Notice that Drake chooses the solver based on a finer category; for example
 * we have a specific solver for equality-constrained convex QP.
 */
enum class ProgramType {
  kLP,      ///< Linear Programming, with a linear cost and linear constraints.
  kQP,      ///< Quadratic Programming, with a convex quadratic cost and linear
            ///< constraints.
  kSOCP,    ///< Second-order Cone Programming, with a linear cost and
            ///< second-order cone constraints.
  kSDP,     ///< Semidefinite Programming, with a linear cost and positive
            ///< semidefinite matrix constraints.
  kGP,      ///< Geometric Programming, with a linear cost and exponential cone
            ///< constraints.
  kCGP,     ///< Conic Geometric Programming, this is a superset that unifies
            ///< GP and SDP. Refer to
            ///< http://people.lids.mit.edu/pari/cgp_preprint.pdf for more
            ///< details.
  kMILP,    ///< Mixed-integer Linear Programming. LP with some variables
            ///< taking binary values.
  kMIQP,    ///< Mixed-integer Quadratic Programming. QP with some variables
            ///< taking binary values.
  kMISOCP,  ///< Mixed-integer Second-order Cone Programming. SOCP with some
            ///< variables taking binary values.
  kMISDP,   ///< Mixed-integer Semidefinite Programming. SDP with some
            ///< variables taking binary values.
  kQuadraticCostConicConstraint,  ///< convex quadratic cost with nonlinear
                                  ///< conic constraints.
  kNLP,      ///< nonlinear programming. Programs with generic costs or
             ///< constraints.
  kLCP,      ///< Linear Complementarity Programs. Programs with linear
             ///< complementary constraints and no cost.
  kUnknown,  ///< Does not fall into any of the types above.
};

std::string to_string(const ProgramType&);

DRAKE_DEPRECATED(
    "2026-06-01",
    "Use fmt functions instead (e.g., fmt::format(), fmt::to_string(), "
    "fmt::print()). Refer to GitHub issue #17742 for more information.")
std::ostream& operator<<(std::ostream&, const ProgramType&);

}  // namespace solvers
}  // namespace drake

DRAKE_FORMATTER_AS(, drake::solvers, ProgramAttribute, x,
                   drake::solvers::to_string(x))
DRAKE_FORMATTER_AS(, drake::solvers, ProgramAttributes, x,
                   drake::solvers::to_string(x))
DRAKE_FORMATTER_AS(, drake::solvers, ProgramType, x,
                   drake::solvers::to_string(x))
