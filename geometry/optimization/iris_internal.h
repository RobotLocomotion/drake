#pragma once
/*
 This file contains the internal functions used by iris.cc. The users shouldn't
 call these functions, but we expose them for unit tests.
 */

#include <limits>
#include <memory>
#include <optional>
#include <variant>

#include "drake/geometry/optimization/affine_ball.h"
#include "drake/geometry/optimization/cartesian_product.h"
#include "drake/geometry/optimization/convex_set.h"
#include "drake/geometry/optimization/hyperellipsoid.h"
#include "drake/geometry/optimization/iris_internal.h"
#include "drake/geometry/optimization/minkowski_sum.h"
#include "drake/geometry/optimization/vpolytope.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solver_interface.h"

namespace drake {
namespace geometry {
namespace optimization {
namespace internal {
// Takes q, p_AA, and p_BB and enforces that p_WA(q) == p_WB(q).
class SamePointConstraint : public solvers::Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SamePointConstraint);

  SamePointConstraint(const multibody::MultibodyPlant<double>* plant,
                      const systems::Context<double>& context);

  ~SamePointConstraint() override;

  void set_frameA(const multibody::Frame<double>* frame) { frameA_ = frame; }

  void set_frameB(const multibody::Frame<double>* frame) { frameB_ = frame; }

  void EnableSymbolic();

 private:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override;

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
              VectorX<symbolic::Expression>* y) const override;

  const multibody::MultibodyPlant<double>* const plant_;
  const multibody::Frame<double>* frameA_{nullptr};
  const multibody::Frame<double>* frameB_{nullptr};
  std::unique_ptr<systems::Context<double>> context_;

  std::unique_ptr<multibody::MultibodyPlant<symbolic::Expression>>
      symbolic_plant_{nullptr};
  std::unique_ptr<systems::Context<symbolic::Expression>> symbolic_context_{
      nullptr};
};

// Takes q, p_AA, and p_BB, and enforces that ||p_WA(q) - p_WB(q)||² <= d²,
// where d is a user-defined maximum distance.
class PointsBoundedDistanceConstraint : public solvers::Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PointsBoundedDistanceConstraint);

  PointsBoundedDistanceConstraint(
      const multibody::MultibodyPlant<double>* plant,
      const systems::Context<double>& context, const double max_distance);

  ~PointsBoundedDistanceConstraint() override;

  void set_frameA(const multibody::Frame<double>* frame) {
    same_point_constraint_.set_frameA(frame);
  }

  void set_frameB(const multibody::Frame<double>* frame) {
    same_point_constraint_.set_frameB(frame);
  }

  void set_max_distance(const double max_distance) {
    UpdateUpperBound(Vector1d(max_distance * max_distance));
  }

  void EnableSymbolic() { same_point_constraint_.EnableSymbolic(); }

 private:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override;

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
              VectorX<symbolic::Expression>* y) const override;

  geometry::optimization::internal::SamePointConstraint same_point_constraint_;
};

// Takes q, p_AA, and p_BB and enforces that p_WA(f(q)) == p_WB(f(q)), where f
// is a user-defined parameterization function.
class ParameterizedSamePointConstraint : public solvers::Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ParameterizedSamePointConstraint);

  ParameterizedSamePointConstraint(
      const multibody::MultibodyPlant<double>* plant,
      const systems::Context<double>& context,
      const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>&
          parameterization_double,
      const std::function<AutoDiffVecXd(const AutoDiffVecXd&)>&
          parameterization_autodiff,
      int parameterization_dimension);

  ~ParameterizedSamePointConstraint() override;

  void set_frameA(const multibody::Frame<double>* frame) {
    same_point_constraint_.set_frameA(frame);
  }

  void set_frameB(const multibody::Frame<double>* frame) {
    same_point_constraint_.set_frameB(frame);
  }

 private:
  template <typename T>
  void DoEvalGeneric(const Eigen::Ref<const VectorX<T>>& x, VectorX<T>* y,
                     const std::function<VectorX<T>(const VectorX<T>&)>&
                         parameterization) const;

  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override;

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
              VectorX<symbolic::Expression>* y) const override;

  SamePointConstraint same_point_constraint_;
  const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>&
      parameterization_double_;
  const std::function<AutoDiffVecXd(const AutoDiffVecXd&)>&
      parameterization_autodiff_;
  int parameterization_dimension_;
};

// Takes q, p_AA, and p_BB, and enforces that ||p_WA(f(q)) - p_WB(f(q))||² <=
// d², where d is a user-defined maximum distance and f is a user-defined
// parameterization function.
class ParameterizedPointsBoundedDistanceConstraint
    : public solvers::Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ParameterizedPointsBoundedDistanceConstraint);

  ParameterizedPointsBoundedDistanceConstraint(
      const multibody::MultibodyPlant<double>* plant,
      const systems::Context<double>& context, const double max_distance,
      const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>&
          parameterization_double,
      const std::function<AutoDiffVecXd(const AutoDiffVecXd&)>&
          parameterization_autodiff,
      int parameterization_dimension);

  ~ParameterizedPointsBoundedDistanceConstraint() override;

  void set_frameA(const multibody::Frame<double>* frame) {
    points_bounded_distance_constraint_.set_frameA(frame);
  }

  void set_frameB(const multibody::Frame<double>* frame) {
    points_bounded_distance_constraint_.set_frameB(frame);
  }

  void set_max_distance(const double max_distance) {
    UpdateUpperBound(Vector1d(max_distance * max_distance));
  }

 private:
  template <typename T>
  void DoEvalGeneric(const Eigen::Ref<const VectorX<T>>& x, VectorX<T>* y,
                     const std::function<VectorX<T>(const VectorX<T>&)>&
                         parameterization) const;

  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override;

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
              VectorX<symbolic::Expression>* y) const override;

  PointsBoundedDistanceConstraint points_bounded_distance_constraint_;
  const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>&
      parameterization_double_;
  const std::function<AutoDiffVecXd(const AutoDiffVecXd&)>&
      parameterization_autodiff_;
  int parameterization_dimension_;
};

/* Defines a MathematicalProgram to solve the problem
 min_q (q-d) CᵀC (q-d)
 s.t. setA in frameA and setB in frameB are in collision in q.
      Aq ≤ b.
 where C, d are the matrix and center from the hyperellipsoid E.

 The class design supports repeated solutions of the (nearly) identical
 problem from different initial guesses.
 */
class ClosestCollisionProgram {
 public:
  typedef std::variant<
      std::shared_ptr<SamePointConstraint>,
      std::shared_ptr<PointsBoundedDistanceConstraint>,
      std::shared_ptr<ParameterizedSamePointConstraint>,
      std::shared_ptr<ParameterizedPointsBoundedDistanceConstraint>>
      AcceptableConstraint;

  ClosestCollisionProgram(AcceptableConstraint same_point_constraint,
                          const multibody::Frame<double>& frameA,
                          const multibody::Frame<double>& frameB,
                          const ConvexSet& setA, const ConvexSet& setB,
                          const Hyperellipsoid& E,
                          const Eigen::Ref<const Eigen::MatrixXd>& A,
                          const Eigen::Ref<const Eigen::VectorXd>& b);

  ~ClosestCollisionProgram();

  void UpdatePolytope(const Eigen::Ref<const Eigen::MatrixXd>& A,
                      const Eigen::Ref<const Eigen::VectorXd>& b);

  // Returns true iff a collision is found.
  // Sets `closest` to an optimizing solution q*, if a solution is found.
  bool Solve(const solvers::SolverInterface& solver,
             const Eigen::Ref<const Eigen::VectorXd>& q_guess,
             const std::optional<solvers::SolverOptions>& solver_options,
             Eigen::VectorXd* closest);

 private:
  solvers::MathematicalProgram prog_;
  solvers::VectorXDecisionVariable q_;
  std::optional<solvers::Binding<solvers::LinearConstraint>> P_constraint_{};
};

// Takes a constraint bound to another mathematical program and defines a new
// constraint that is the negation of one index and one (lower/upper) bound.
class CounterexampleConstraint : public solvers::Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CounterexampleConstraint);

  explicit CounterexampleConstraint(const solvers::MathematicalProgram* prog)
      : solvers::Constraint(
            1, prog->num_vars(),
            Vector1d(-std::numeric_limits<double>::infinity()),
            Vector1d::Constant(-kSolverConstraintTolerance - 1e-14)),
        prog_{prog} {
    DRAKE_DEMAND(prog != nullptr);
  }

  ~CounterexampleConstraint() = default;

  // Sets the actual constraint to be falsified, overwriting any previously set
  // constraints. The Binding<Constraint> must remain valid for the lifetime of
  // this object (or until a new Binding<Constraint> is set).
  void set(const solvers::Binding<solvers::Constraint>*
               binding_with_constraint_to_be_falsified,
           int index, bool falsify_lower_bound);

 private:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override;

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override;

  void DoEval(const Eigen::Ref<const Eigen::VectorX<symbolic::Variable>>&,
              Eigen::VectorX<symbolic::Expression>*) const override {
    // MathematicalProgram::EvalBinding doesn't support symbolic, and we
    // shouldn't get here.
    throw std::logic_error(
        "CounterexampleConstraint doesn't support DoEval for symbolic.");
  }

  const solvers::MathematicalProgram* prog_{};
  const solvers::Binding<solvers::Constraint>* binding_{};
  int index_{0};
  bool falsify_lower_bound_{true};

  // To find a counterexample for a constraints,
  //  g(x) ≤ ub,
  // we need to ask the solver to find
  //  g(x) + kSolverConstraintTolerance > ub,
  // which we implement as
  //  g(x) + kSolverConstraintTolerance ≥ ub + eps.
  // The variable is static so that it is initialized by the time it is accessed
  // in the initializer list of the constructor.
  // TODO(russt): We need a more robust way to get this from the solver. This
  // value works for SNOPT and is reasonable for most solvers.
  static constexpr double kSolverConstraintTolerance{1e-6};
};

// Defines a MathematicalProgram to solve the problem
// min_q (q-d)*CᵀC(q-d)
// s.t. counterexample-constraint
//      Aq ≤ b.
// where C, d are the matrix and center from the hyperellipsoid E.
//
// The class design supports repeated solutions of the (nearly) identical
// problem from different initial guesses.
class CounterexampleProgram {
 public:
  CounterexampleProgram(
      std::shared_ptr<CounterexampleConstraint> counter_example_constraint,
      const Hyperellipsoid& E, const Eigen::Ref<const Eigen::MatrixXd>& A,
      const Eigen::Ref<const Eigen::VectorXd>& b);

  void UpdatePolytope(const Eigen::Ref<const Eigen::MatrixXd>& A,
                      const Eigen::Ref<const Eigen::VectorXd>& b);

  // Returns true iff a counterexample is found.
  // Sets `closest` to an optimizing solution q*, if a solution is found.
  bool Solve(const solvers::SolverInterface& solver,
             const Eigen::Ref<const Eigen::VectorXd>& q_guess,
             const std::optional<solvers::SolverOptions>& solver_options,
             Eigen::VectorXd* closest);

 private:
  solvers::MathematicalProgram prog_;
  solvers::VectorXDecisionVariable q_;
  std::optional<solvers::Binding<solvers::LinearConstraint>> P_constraint_{};
};

// Constructs a ConvexSet for each supported Shape and adds it to the set.
class IrisConvexSetMaker final : public ShapeReifier {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IrisConvexSetMaker);

  IrisConvexSetMaker(const QueryObject<double>& query,
                     std::optional<FrameId> reference_frame)
      : query_{query}, reference_frame_{reference_frame} {};

  void set_reference_frame(const FrameId& reference_frame) {
    DRAKE_DEMAND(reference_frame.is_valid());
    *reference_frame_ = reference_frame;
  }

  void set_geometry_id(const GeometryId& geom_id) { geom_id_ = geom_id; }

  using ShapeReifier::ImplementGeometry;

  void ImplementGeometry(const Box&, void* data);

  void ImplementGeometry(const Capsule&, void* data);

  void ImplementGeometry(const Cylinder&, void* data);

  void ImplementGeometry(const Ellipsoid&, void* data);

  void ImplementGeometry(const HalfSpace&, void* data);

  void ImplementGeometry(const Sphere&, void* data);

  void ImplementGeometry(const Convex&, void* data);

  void ImplementGeometry(const Mesh&, void* data);

 private:
  const QueryObject<double>& query_{};
  std::optional<FrameId> reference_frame_{};
  GeometryId geom_id_{};
};

struct GeometryPairWithDistance {
  GeometryId geomA;
  GeometryId geomB;
  double distance;

  GeometryPairWithDistance(GeometryId gA, GeometryId gB, double dist)
      : geomA(gA), geomB(gB), distance(dist) {}

  bool operator<(const GeometryPairWithDistance& other) const {
    return distance < other.distance;
  }
};

}  // namespace internal
}  // namespace optimization
}  // namespace geometry
}  // namespace drake
