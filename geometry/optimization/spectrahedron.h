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

By convention, a zero-dimensional spectrahedron is considered nonempty.

@ingroup geometry_optimization */
class Spectrahedron final : public ConvexSet {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Spectrahedron);

  /** Default constructor (yields the zero-dimensional nonempty set). */
  Spectrahedron();

  /** Constructs the spectrahedron from a MathematicalProgram.
  @throws std::exception if `prog.required_capabilities()` is not a subset of
  supported_attributes(). */
  explicit Spectrahedron(const solvers::MathematicalProgram& prog);

  ~Spectrahedron() final;

  /** Returns the list of solvers::ProgramAttributes supported by this class. */
  static const solvers::ProgramAttributes& supported_attributes();

  // TODO(russt): Add PointInSet(MatrixXd X, double tol) overload, which will
  // only work in the case where the ambient_dimension is ONLY symmetric
  // matrices.

  /** @throws  Not implemented. */
  using ConvexSet::CalcVolume;

  /** Spectrahedron uses the generic method for boundedness checking, which uses
  `parallelism`.
  @param parallelism The maximum number of threads to use.
  @note See @ref ConvexSet::IsBounded "parent class's documentation" for more
  details. */
  using ConvexSet::IsBounded;

 private:
  std::unique_ptr<ConvexSet> DoClone() const final;

  /* We only use DoIsBoundedShortcut here to avoid an edge case that causes an
  error with the CSDP solver (#19927). */
  std::optional<bool> DoIsBoundedShortcut() const final;

  // N.B. No need to override DoMaybeGetPoint here.

  std::optional<bool> DoPointInSetShortcut(
      const Eigen::Ref<const Eigen::VectorXd>& x, double tol) const final;

  std::pair<VectorX<symbolic::Variable>,
            std::vector<solvers::Binding<solvers::Constraint>>>
  DoAddPointInSetConstraints(
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
      const Eigen::Ref<const Eigen::MatrixXd>& A,
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
