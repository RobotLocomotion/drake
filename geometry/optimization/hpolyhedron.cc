#include "drake/geometry/optimization/hpolyhedron.h"

#include <limits>
#include <memory>
#include <set>
#include <stdexcept>

#include <Eigen/Eigenvalues>
#include <fmt/format.h>

#include "drake/math/matrix_util.h"
#include "drake/math/rotation_matrix.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace geometry {
namespace optimization {

using Eigen::MatrixXd;
using Eigen::RowVectorXd;
using Eigen::VectorXd;
using math::RigidTransformd;
using solvers::Binding;
using solvers::Constraint;
using solvers::MathematicalProgram;
using solvers::MatrixXDecisionVariable;
using solvers::VectorXDecisionVariable;
using symbolic::Expression;
using symbolic::Variable;

namespace {
const double kInf = std::numeric_limits<double>::infinity();

// Checks if Ax ≤ b defines an empty set
bool IsEmpty(const Eigen::Ref<const MatrixXd>& A,
             const Eigen::Ref<const VectorXd>& b) {
  solvers::MathematicalProgram prog;
  // Turn off Gurobi DualReduction to ensure that infeasible problems always
  // return solvers::SolutionResult::kInfeasibleConstraints rather than
  // SolutionResult::kInfeasibleOrUnbounded.
  prog.SetSolverOption(solvers::GurobiSolver::id(), "DualReductions", 0);

  solvers::VectorXDecisionVariable x =
      prog.NewContinuousVariables(A.cols(), "x");
  prog.AddLinearConstraint(A, VectorXd::Constant(b.rows(), -kInf), b, x);
  auto result = solvers::Solve(prog);
  return result.get_solution_result() ==
         solvers::SolutionResult::kInfeasibleConstraints;
}

bool IsEmpty(solvers::MathematicalProgram* prog) {
  prog->SetSolverOption(solvers::GurobiSolver::id(), "DualReductions", 0);
  auto result = solvers::Solve(*prog);
  return result.get_solution_result() ==
         solvers::SolutionResult::kInfeasibleConstraints;
}

/* Check whether the constraint cᵀ x ≤ d is already implied by the linear
 constraints in prog. This is done by solving a small linear program
 and modifying the coefficients of  `new_constraint` binding. This method may
 throw a runtime error if the constraints are ill-conditioned.
 */
bool IsRedundant(const Eigen::Ref<const MatrixXd>& c, double d,
                 solvers::MathematicalProgram* prog,
                 Binding<solvers::LinearConstraint>* new_constraint,
                 Binding<solvers::LinearCost>* program_cost_binding,
                 bool already_checked_feasibility) {
  // Ensure that prog is an LP
  DRAKE_DEMAND(prog->GetAllConstraints().size() ==
               prog->GetAllLinearConstraints().size());
  DRAKE_DEMAND(prog->GetAllCosts().size() == 1 &&
               prog->linear_costs()[0] == *program_cost_binding);

  // Turn off Gurobi DualReduction to ensure that infeasible problems always
  // return solvers::SolutionResult::kInfeasibleConstraints rather than
  // SolutionResult::kInfeasibleOrUnbounded.
  prog->SetSolverOption(solvers::GurobiSolver::id(), "DualReductions", 0);

  if (!already_checked_feasibility) {
    // Constraints of prog define an empty set therefore any constraint is
    // redundant.
    if (IsEmpty(prog)) {
      return true;
    }
  }

  // This inequality ensures a bounded objective since the left hand side of
  // the inequality is the same as the cost function on the next line.
  new_constraint->evaluator()->UpdateCoefficients(
      c, VectorXd::Constant(1, -kInf),
      VectorXd::Constant(1, d + 1));

  program_cost_binding->evaluator()->UpdateCoefficients(-c.transpose(), 0);

  auto result = solvers::Solve(*prog);

  // Constraints define an empty set or the current inequality of other is not
  // redundant. Since we tested whether this polyhedron is empty earlier, the
  // function would already have exited so this is proof that this inequality
  // is irredundant.
  bool PolyhedronIsEmpty = result.get_solution_result() ==
                           solvers::SolutionResult::kInfeasibleConstraints;

  if (!PolyhedronIsEmpty && !result.is_success()) {
    throw std::runtime_error(fmt::format(
        "Solver {} failed to compute the set difference; it "
        "terminated with SolutionResult {}). This should only happen"
        "if the problem is ill-conditioned",
        result.get_solver_id().name(), result.get_solution_result()));
  }

  // If -result.get_optimal_cost() > other.b()(i) then the inequality is
  // irredundant. Without this constant
  // IrredundantBallIntersectionContainedInBothOriginal fails.
  return !(PolyhedronIsEmpty || -result.get_optimal_cost() > d + 1E-9);
}

}  // namespace

HPolyhedron::HPolyhedron(const Eigen::Ref<const MatrixXd>& A,
                         const Eigen::Ref<const VectorXd>& b)
    : ConvexSet(&ConvexSetCloner<HPolyhedron>, A.cols()), A_{A}, b_{b} {
  DRAKE_DEMAND(A.rows() == b.size());
  // Note: If necessary, we could support infinite b, either by removing the
  // corresponding rows of A (since the constraint is vacuous), or checking
  // this explicitly in all relevant computations (like IsBounded).
  DRAKE_DEMAND(b.array().isFinite().all());
}

HPolyhedron::HPolyhedron(const QueryObject<double>& query_object,
                         GeometryId geometry_id,
                         std::optional<FrameId> reference_frame)
    : ConvexSet(&ConvexSetCloner<HPolyhedron>, 3) {
  std::pair<MatrixXd, VectorXd> Ab_G;
  query_object.inspector().GetShape(geometry_id).Reify(this, &Ab_G);

  const RigidTransformd X_WE =
      reference_frame ? query_object.GetPoseInWorld(*reference_frame)
                      : RigidTransformd::Identity();
  const RigidTransformd& X_WG = query_object.GetPoseInWorld(geometry_id);
  const RigidTransformd X_GE = X_WG.InvertAndCompose(X_WE);
  // A_G*(p_GE + R_GE*p_EE_var) ≤ b_G
  A_ = Ab_G.first * X_GE.rotation().matrix();
  b_ = Ab_G.second - Ab_G.first * X_GE.translation();
}

HPolyhedron::~HPolyhedron() = default;

Hyperellipsoid HPolyhedron::MaximumVolumeInscribedEllipsoid() const {
  MathematicalProgram prog;
  const int N = this->ambient_dimension();
  MatrixXDecisionVariable C = prog.NewSymmetricContinuousVariables(N, "C");
  VectorXDecisionVariable d = prog.NewContinuousVariables(N, "d");

  // max log det (C).  This method also imposes C ≽ 0.
  prog.AddMaximizeLogDeterminantCost(C.cast<Expression>());
  // |aᵢC|₂ ≤ bᵢ - aᵢd, ∀i
  // TODO(russt): We could potentially avoid Expression parsing here by using
  // AddLorentzConeConstraint(A,b,vars), but it's nontrivial because of the
  // duplicate entries in the symmetric matrix C.  E.g. the Lorentz cone A would
  // not be simply block_diagonal(-A_.row(i), A_.row(i), ..., A_.row(i)).
  VectorX<Expression> z(N + 1);
  for (int i = 0; i < b_.size(); ++i) {
    z[0] = b_(i) - A_.row(i).dot(d);
    z.tail(N) = C * A_.row(i).transpose();
    prog.AddLorentzConeConstraint(z);
  }
  auto result = solvers::Solve(prog);
  if (!result.is_success()) {
    throw std::runtime_error(fmt::format(
        "Solver {} failed to solve the maximum inscribed ellipse problem; it "
        "terminated with SolutionResult {}). Make sure that your polyhedron is "
        "bounded and has an interior.",
        result.get_solver_id().name(), result.get_solution_result()));
  }
  return Hyperellipsoid(result.GetSolution(C).inverse(), result.GetSolution(d));
}

VectorXd HPolyhedron::ChebyshevCenter() const {
  MathematicalProgram prog;
  VectorXDecisionVariable x = prog.NewContinuousVariables(ambient_dimension());
  VectorXDecisionVariable r = prog.NewContinuousVariables<1>("r");

  const double inf = std::numeric_limits<double>::infinity();
  // max r
  prog.AddLinearCost(Vector1d(-1.0), 0, r);

  // r ≥ 0.
  prog.AddBoundingBoxConstraint(0, inf, r);

  // aᵢᵀ x + |aᵢ| r ≤ bᵢ.
  RowVectorXd a(A_.cols() + 1);
  for (int i = 0; i < A_.rows(); i++) {
    a[0] = A_.row(i).norm();
    a.tail(A_.cols()) = A_.row(i);
    prog.AddLinearConstraint(a, -inf, b_[i], {r, x});
  }

  auto result = solvers::Solve(prog);
  if (!result.is_success()) {
    throw std::runtime_error(fmt::format(
        "Solver {} failed to solve the Chebyshev center problem; it terminated "
        "with SolutionResult {}). Make sure that your polyhedron is bounded "
        "and has an interior.",
        result.get_solver_id().name(), result.get_solution_result()));
  }
  return result.GetSolution(x);
}

HPolyhedron HPolyhedron::CartesianProduct(const HPolyhedron& other) const {
  MatrixXd A_product = MatrixXd::Zero(A_.rows() + other.A().rows(),
                                      A_.cols() + other.A().cols());
  A_product.topLeftCorner(A_.rows(), A_.cols()) = A_;
  A_product.bottomRightCorner(other.A().rows(), other.A().cols()) = other.A();
  VectorXd b_product(b_.size() + other.b().size());
  b_product << b_, other.b();
  return {A_product, b_product};
}

HPolyhedron HPolyhedron::CartesianPower(int n) const {
  MatrixXd A_power = MatrixXd::Zero(n * A_.rows(), n * A_.cols());
  for (int i{0}; i < n; ++i) {
    A_power.block(i * A_.rows(), i * A_.cols(), A_.rows(), A_.cols()) = A_;
  }
  VectorXd b_power = b_.replicate(n, 1);
  return {A_power, b_power};
}

HPolyhedron HPolyhedron::Intersection(const HPolyhedron& other,
                                      bool check_for_redundancy) const {
  if (check_for_redundancy) {
    return this->DoIntersectionWithChecks(other);
  }
  return this->DoIntersectionNoChecks(other);
}

HPolyhedron HPolyhedron::MakeBox(const Eigen::Ref<const VectorXd>& lb,
                                 const Eigen::Ref<const VectorXd>& ub) {
  DRAKE_DEMAND(lb.size() == ub.size());
  DRAKE_DEMAND((lb.array() <= ub.array()).all());
  const int N = lb.size();
  MatrixXd A(2 * N, N);
  A << MatrixXd::Identity(N, N), -MatrixXd::Identity(N, N);
  VectorXd b(2 * N);
  b << ub, -lb;
  return HPolyhedron(A, b);
}

HPolyhedron HPolyhedron::MakeUnitBox(int dim) {
  return MakeBox(VectorXd::Constant(dim, -1.0), VectorXd::Constant(dim, 1.0));
}

bool HPolyhedron::DoIsBounded() const {
  if (A_.rows() < A_.cols()) {
    return false;
  }
  Eigen::ColPivHouseholderQR<MatrixXd> qr(A_);
  if (qr.dimensionOfKernel() > 0) {
    return false;
  }
  // Stiemke's theorem of alternatives says that, given A with ker(A) = {0}, we
  // either have existence of x ≠ 0 such that Ax ≥ 0 or we have existence of y
  // > 0 such that y^T A = 0.  Since any y that verifies the second condition
  // can be arbitrarily scaled, and would still pass the second condition,
  // instead of asking y > 0, we can equivalently ask y ≥ 1.  So boundedness
  // corresponds to the following LP being feasible: find y s.t. y ≥ 1, y^T A =
  // 0.
  MathematicalProgram prog;
  auto y = prog.NewContinuousVariables(A_.rows(), "y");
  prog.AddBoundingBoxConstraint(1.0, std::numeric_limits<double>::infinity(),
                                y);
  prog.AddLinearEqualityConstraint(A_.transpose(), VectorXd::Zero(A_.cols()),
                                   y);
  auto result = solvers::Solve(prog);
  return result.is_success();
}

bool HPolyhedron::ContainedIn(const HPolyhedron& other) const {
  DRAKE_DEMAND(other.A().cols() == A_.cols());
  // `this` defines an empty set and therefore is contained in any `other`
  // HPolyhedron
  if (IsEmpty(A_, b_)) {
    return true;
  }

  solvers::MathematicalProgram prog;
  solvers::VectorXDecisionVariable x =
      prog.NewContinuousVariables(A_.cols(), "x");
  prog.AddLinearConstraint(A_, VectorXd::Constant(b_.rows(), -kInf), b_,
                           x);
  auto result = solvers::Solve(prog);

  Binding<solvers::LinearConstraint> redundant_constraint_binding =
      prog.AddLinearConstraint(other.A().row(0),
                               VectorXd::Constant(1, -kInf),
                               other.b().row(0), x);
  Binding<solvers::LinearCost> program_cost_binding =
      prog.AddLinearCost(-other.A().row(0), 0, x);

  for (int i = 0; i < other.A().rows(); i++) {
    // If any of the constraints of `other` are irredundant then `other` is
    // not contained in this.
    if (!IsRedundant(other.A().row(i), other.b()(i), &prog,
                     &redundant_constraint_binding, &program_cost_binding,
                     true)) {
      return false;
    }
  }
  return true;
}

HPolyhedron HPolyhedron::DoIntersectionNoChecks(
    const HPolyhedron& other) const {
  DRAKE_DEMAND(ambient_dimension() == other.ambient_dimension());
  MatrixXd A_intersect =
      MatrixXd::Zero(A_.rows() + other.A().rows(), A_.cols());
  A_intersect.topRows(A_.rows()) = A_;
  A_intersect.bottomRows(other.A().rows()) = other.A();
  VectorXd b_intersect(b_.size() + other.b().size());
  b_intersect << b_, other.b();
  return {A_intersect, b_intersect};
}

HPolyhedron HPolyhedron::DoIntersectionWithChecks(
    const HPolyhedron& other) const {
  DRAKE_DEMAND(other.A().cols() == A_.cols());

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A(
      A_.rows() + other.A().rows(), A_.cols());
  VectorXd b(A_.rows() + other.A().rows());
  A.topRows(A_.rows()) = A_;
  b.head(A_.rows()) = b_;

  solvers::MathematicalProgram prog;
  solvers::VectorXDecisionVariable x =
      prog.NewContinuousVariables(A_.cols(), "x");
  prog.AddLinearConstraint(A_, VectorXd::Constant(b_.rows(), -kInf), b_,
                           x);
  auto result = solvers::Solve(prog);
  // `this` defines an empty set therefore any additional constraint is
  // redundant.
  if (result.get_solution_result() ==
      solvers::SolutionResult::kInfeasibleConstraints) {
    return {A_, b_};
  }

  Binding<solvers::LinearConstraint> redundant_constraint_binding =
      prog.AddLinearConstraint(other.A().row(0),
                               VectorXd::Constant(1, -kInf),
                               other.b().row(0), x);
  Binding<solvers::LinearCost> program_cost_binding =
      prog.AddLinearCost(-other.A().row(0), 0, x);

  int num_kept = A_.rows();
  for (int i = 0; i < other.A().rows(); i++) {
    if (!IsRedundant(other.A().row(i), other.b()(i), &prog,
                     &redundant_constraint_binding, &program_cost_binding,
                     true)) {
      A.row(num_kept) = other.A().row(i);
      b.row(num_kept) = other.b().row(i);
      num_kept++;
      if (IsEmpty(A.topRows(num_kept), b.topRows(num_kept))) {
        return {A.topRows(num_kept), b.topRows(num_kept)};
      }
    }
  }
  return {A.topRows(num_kept), b.topRows(num_kept)};
}

HPolyhedron HPolyhedron::ReduceInequalities() const {
  const int num_inequalities = A_.rows();
  const int num_vars = A_.cols();

  std::set<int> kept_indices;
  for (int i = 0; i < num_inequalities; i++) {
    kept_indices.emplace(i);
  }

  for (int excluded_index = 0; excluded_index < num_inequalities;
       excluded_index++) {
    solvers::MathematicalProgram prog;
    solvers::VectorXDecisionVariable x =
        prog.NewContinuousVariables(num_vars, "x");

    std::set<int> cur_kept_indices = kept_indices;
    cur_kept_indices.erase(excluded_index);

    // Current constraints.
    for (const int i : cur_kept_indices) {
      prog.AddLinearConstraint(A_.row(i), VectorXd::Constant(1, -kInf),
                               b_.row(i), x);
    }

    // Constraint to check redundant.
    Binding<solvers::LinearConstraint> redundant_constraint_binding =
        prog.AddLinearConstraint(
            A_.row(excluded_index), VectorXd::Constant(1, -kInf),
            b_.row(excluded_index) + VectorXd::Ones(1), x);

    // Construct cost binding for prog.
    Binding<solvers::LinearCost> program_cost_binding =
        prog.AddLinearCost(-A_.row(excluded_index), 0, x);

    auto result = solvers::Solve(prog);

    // the current inequality is redundant
    if (IsRedundant(A_.row(excluded_index), b_(excluded_index), &prog,
                    &redundant_constraint_binding, &program_cost_binding,
                    false)) {
      kept_indices.erase(excluded_index);
    }
  }

  MatrixXd A_new(kept_indices.size(), num_vars);
  VectorXd b_new(kept_indices.size());
  int i = 0;
  for (const int ind : kept_indices) {
    A_new.row(i) = A_.row(ind);
    b_new.row(i) = b_.row(ind);
    i++;
  }
  return {A_new, b_new};
}

bool HPolyhedron::DoPointInSet(const Eigen::Ref<const VectorXd>& x,
                               double tol) const {
  DRAKE_DEMAND(A_.cols() == x.size());
  return ((A_ * x).array() <= b_.array() + tol).all();
}

void HPolyhedron::DoAddPointInSetConstraints(
    MathematicalProgram* prog,
    const Eigen::Ref<const VectorXDecisionVariable>& vars) const {
  prog->AddLinearConstraint(
      A_,
      VectorXd::Constant(b_.size(), -std::numeric_limits<double>::infinity()),
      b_, vars);
}

std::vector<Binding<Constraint>>
HPolyhedron::DoAddPointInNonnegativeScalingConstraints(
    MathematicalProgram* prog,
    const Eigen::Ref<const VectorXDecisionVariable>& x,
    const Variable& t) const {
  std::vector<Binding<Constraint>> constraints;
  // A x ≤ t b, written as [A,-b][x;t] ≤ 0
  const int m = A_.rows();
  const int n = ambient_dimension();
  MatrixXd Abar(m, n + 1);
  Abar.leftCols(n) = A_;
  Abar.col(n) = -b_;
  constraints.emplace_back(prog->AddLinearConstraint(
      Abar, VectorXd::Constant(m, -std::numeric_limits<double>::infinity()),
      VectorXd::Zero(m), {x, Vector1<Variable>(t)}));
  return constraints;
}

std::vector<Binding<Constraint>>
HPolyhedron::DoAddPointInNonnegativeScalingConstraints(
    MathematicalProgram* prog, const Eigen::Ref<const MatrixXd>& A_x,
    const Eigen::Ref<const VectorXd>& b_x, const Eigen::Ref<const VectorXd>& c,
    double d, const Eigen::Ref<const VectorXDecisionVariable>& x,
    const Eigen::Ref<const VectorXDecisionVariable>& t) const {
  std::vector<Binding<Constraint>> constraints;
  // A (A_x x + b_x) ≤ (c' t + d) b, written as [A * A_x, -b * c'][x;t] ≤ d * b
  // - A * b_x
  const int m = A_.rows();
  MatrixXd A_bar(m, x.size() + t.size());
  A_bar.leftCols(x.size()) = A_ * A_x;
  A_bar.rightCols(t.size()) = -b_ * c.transpose();
  VectorXd b_bar = d * b_ - A_ * b_x;
  constraints.emplace_back(prog->AddLinearConstraint(
      A_bar, VectorXd::Constant(m, -std::numeric_limits<double>::infinity()),
      b_bar, {x, t}));
  return constraints;
}

std::pair<std::unique_ptr<Shape>, math::RigidTransformd>
HPolyhedron::DoToShapeWithPose() const {
  throw std::runtime_error(
      "ToShapeWithPose is not implemented yet for HPolyhedron.  Implementing "
      "this will likely require additional support from the Convex shape "
      "class (to support in-memory mesh data, or file I/O).");
}

void HPolyhedron::ImplementGeometry(const HalfSpace&, void* data) {
  auto* Ab = static_cast<std::pair<MatrixXd, VectorXd>*>(data);
  // z <= 0.0.
  Ab->first = Eigen::RowVector3d{0.0, 0.0, 1.0};
  Ab->second = Vector1d{0.0};
}

void HPolyhedron::ImplementGeometry(const Box& box, void* data) {
  Eigen::Matrix<double, 6, 3> A;
  A << Eigen::Matrix3d::Identity(), -Eigen::Matrix3d::Identity();
  Vector6d b;
  // clang-format off
  b << box.width()/2.0, box.depth()/2.0, box.height()/2.0,
        box.width()/2.0, box.depth()/2.0, box.height()/2.0;
  // clang-format on
  auto* Ab = static_cast<std::pair<MatrixXd, VectorXd>*>(data);
  Ab->first = A;
  Ab->second = b;
}

HPolyhedron HPolyhedron::PontryaginDifference(const HPolyhedron& other) const {
  /**
   * The Pontryagin set difference of Polytope P = {x | Ax <= b} and
   * Q = {x | Cx <= d} can be computed P - Q = {x | Ax <= b - h}
   * where hᵢ = max aᵢᵀx subject to x ∈ Q
   */

  DRAKE_DEMAND(this->ambient_dimension() == other.ambient_dimension());
  DRAKE_DEMAND(this->IsBounded());
  DRAKE_DEMAND(other.IsBounded());
  const double kInf = std::numeric_limits<double>::infinity();

  VectorXd b_diff(b_.rows());
  MathematicalProgram prog;
  solvers::VectorXDecisionVariable x =
      prog.NewContinuousVariables(ambient_dimension_, "x");
  // -inf <= Ax <= b
  prog.AddLinearConstraint(other.A(),
                           VectorXd::Constant(other.b().rows(), -kInf),
                           other.b(), x);

  auto result = solvers::Solve(prog);
  // other is an empty polyhedron and so Pontryagin difference does nothing
  if (result.get_solution_result() ==
      solvers::SolutionResult::kInfeasibleConstraints) {
    return {A_, b_};
  }

  Binding<solvers::LinearCost> program_cost_binding =
      prog.AddLinearCost(A_.row(0), 0, x);
  for (int i = 0; i < b_.rows(); ++i) {
    program_cost_binding.evaluator()->UpdateCoefficients(-A_.row(i), 0);
    result = solvers::Solve(prog);
    // since constraint set is bounded and non-empty then the program should
    // always have an optimal solution
    if (!result.is_success()) {
      throw std::runtime_error(fmt::format(
          "Solver {} failed to compute the set difference; it "
          "terminated with SolutionResult {}). This should only happen"
          "if the problem is ill-conditioned",
          result.get_solver_id().name(), result.get_solution_result()));
    }
    b_diff(i) = b_(i) + result.get_optimal_cost();
  }
  return {A_, b_diff};
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
