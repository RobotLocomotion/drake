#pragma once

#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "drake/geometry/optimization/convex_set.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace geometry {
namespace optimization {

/** Implements a spectrahedron (the feasible set of a semidefinite program).
The ambient dimension of the set is N(N+1)/2; the number of variables required
to describe the N-by-N semidefinite matrix.

@ingroup geometry_optimization
*/
class Spectrahedron final : public ConvexSet {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Spectrahedron)

  /** Default constructor (which constructs the empty set). */
  Spectrahedron();

  /** Constructs the spectrahedron from a MathematicalProgram.
  @pre GetProgramType(prog) must be kSDP.
  @pre @p prog must have exactly one positive-semidefinite constraint which is
  bound to *all* of the decision variables. */
  explicit Spectrahedron(const solvers::MathematicalProgram& prog);

  ~Spectrahedron() final;

 private:
  bool DoIsBounded() const final;

  bool DoPointInSet(const Eigen::Ref<const Eigen::VectorXd>& x,
                    double tol) const final;

  void DoAddPointInSetConstraints(
      solvers::MathematicalProgram* prog,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>& vars)
      const final;

  std::vector<solvers::Binding<solvers::Constraint>>
  DoAddPointInNonnegativeScalingConstraints(
      solvers::MathematicalProgram* prog,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>& x,
      const symbolic::Variable& t) const final;

  std::vector<solvers::Binding<solvers::Constraint>>
  DoAddPointInNonnegativeScalingConstraints(
      solvers::MathematicalProgram* prog,
      const Eigen::Ref<const Eigen::MatrixXd>& A_x,
      const Eigen::Ref<const Eigen::VectorXd>& b,
      const Eigen::Ref<const Eigen::VectorXd>& c, double d,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>& x,
      const Eigen::Ref<const solvers::VectorXDecisionVariable>& t) const final;

  std::pair<std::unique_ptr<Shape>, math::RigidTransformd> DoToShapeWithPose()
      const final;

  copyable_unique_ptr<solvers::MathematicalProgram> sdp_{};
};

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
