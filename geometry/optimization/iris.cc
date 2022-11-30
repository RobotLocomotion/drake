#include "drake/geometry/optimization/iris.h"

#include <algorithm>
#include <limits>
#include <optional>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "drake/common/symbolic/expression.h"
#include "drake/geometry/optimization/cartesian_product.h"
#include "drake/geometry/optimization/convex_set.h"
#include "drake/geometry/optimization/minkowski_sum.h"
#include "drake/geometry/optimization/vpolytope.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/choose_best_solver.h"
#include "drake/solvers/ipopt_solver.h"
#include "drake/solvers/snopt_solver.h"

namespace drake {
namespace geometry {
namespace optimization {

using Eigen::MatrixXd;
using Eigen::Ref;
using Eigen::Vector3d;
using Eigen::VectorXd;
using math::RigidTransform;
using multibody::Body;
using multibody::Frame;
using multibody::JacobianWrtVariable;
using multibody::MultibodyPlant;
using solvers::MathematicalProgram;
using solvers::Binding;
using solvers::Constraint;
using symbolic::Expression;
using systems::Context;

HPolyhedron Iris(const ConvexSets& obstacles, const Ref<const VectorXd>& sample,
                 const HPolyhedron& domain, const IrisOptions& options) {
  const int dim = sample.size();
  const int N = obstacles.size();
  DRAKE_DEMAND(domain.ambient_dimension() == dim);
  for (int i = 0; i < N; ++i) {
    DRAKE_DEMAND(obstacles[i]->ambient_dimension() == dim);
  }
  DRAKE_DEMAND(domain.IsBounded());
  const double kEpsilonEllipsoid = 1e-2;
  Hyperellipsoid E = Hyperellipsoid::MakeHypersphere(kEpsilonEllipsoid, sample);
  HPolyhedron P = domain;

  // On each iteration, we will build the collision-free polytope represented as
  // {x | A * x <= b}.  Here we pre-allocate matrices of the maximum size.
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A(
      domain.A().rows() + N, dim);
  VectorXd b(domain.A().rows() + N);
  A.topRows(domain.A().rows()) = domain.A();
  b.head(domain.A().rows()) = domain.b();
  // Use pairs {scale, index}, so that I can back out the indices after a sort.
  std::vector<std::pair<double, int>> scaling(N);
  MatrixXd closest_points(dim, N);

  double best_volume = E.Volume();
  int iteration = 0;
  MatrixXd tangent_matrix;

  while (true) {
    DRAKE_ASSERT(best_volume > 0);
    // Find separating hyperplanes
    for (int i = 0; i < N; ++i) {
      const auto touch = E.MinimumUniformScalingToTouch(*obstacles[i]);
      scaling[i].first = touch.first;
      scaling[i].second = i;
      closest_points.col(i) = touch.second;
    }
    std::sort(scaling.begin(), scaling.end());

    int num_constraints = domain.A().rows();
    tangent_matrix = 2.0 * E.A().transpose() * E.A();
    for (int i = 0; i < N; ++i) {
      // Only add a constraint if this obstacle still has overlap with the set
      // that has been constructed so far on this iteration.
      if (HPolyhedron(A.topRows(num_constraints), b.head(num_constraints))
              .IntersectsWith(*obstacles[scaling[i].second])) {
        // Add the tangent to the (scaled) ellipsoid at this point as a
        // constraint.
        const VectorXd point = closest_points.col(scaling[i].second);
        A.row(num_constraints) =
            (tangent_matrix * (point - E.center())).normalized();
        b[num_constraints] = A.row(num_constraints) * point;
        num_constraints++;
      }
    }

    if (options.require_sample_point_is_contained &&
        ((A.topRows(num_constraints) * sample).array() >=
         b.head(num_constraints).array())
            .any()) {
      break;
    }
    P = HPolyhedron(A.topRows(num_constraints), b.head(num_constraints));

    iteration++;
    if (iteration >= options.iteration_limit) {
      break;
    }

    E = P.MaximumVolumeInscribedEllipsoid();
    const double volume = E.Volume();
    const double delta_volume = volume - best_volume;
    if (delta_volume <= options.termination_threshold) {
      break;
    }
    if (delta_volume / best_volume <= options.relative_termination_threshold) {
      break;
    }
    best_volume = volume;
  }

  return P;
}

namespace {
// Constructs a ConvexSet for each supported Shape and adds it to the set.
class IrisConvexSetMaker final : public ShapeReifier {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IrisConvexSetMaker)

  IrisConvexSetMaker(const QueryObject<double>& query,
                     std::optional<FrameId> reference_frame)
      : query_{query}, reference_frame_{reference_frame} {};

  void set_reference_frame(const FrameId& reference_frame) {
    DRAKE_DEMAND(reference_frame.is_valid());
    *reference_frame_ = reference_frame;
  }

  void set_geometry_id(const GeometryId& geom_id) { geom_id_ = geom_id; }

  using ShapeReifier::ImplementGeometry;

  void ImplementGeometry(const Box&, void* data) {
    DRAKE_DEMAND(geom_id_.is_valid());
    auto& set = *static_cast<copyable_unique_ptr<ConvexSet>*>(data);
    // Note: We choose HPolyhedron over VPolytope here, but the IRIS paper
    // discusses a significant performance improvement using a "least-distance
    // programming" instance from CVXGEN that exploited the VPolytope
    // representation.  So we may wish to revisit this.
    set = std::make_unique<HPolyhedron>(query_, geom_id_, reference_frame_);
  }

  void ImplementGeometry(const Capsule&, void* data) {
    DRAKE_DEMAND(geom_id_.is_valid());
    auto& set = *static_cast<copyable_unique_ptr<ConvexSet>*>(data);
    set = std::make_unique<MinkowskiSum>(query_, geom_id_, reference_frame_);
  }

  void ImplementGeometry(const Cylinder&, void* data) {
    DRAKE_DEMAND(geom_id_.is_valid());
    auto& set = *static_cast<copyable_unique_ptr<ConvexSet>*>(data);
    set =
        std::make_unique<CartesianProduct>(query_, geom_id_, reference_frame_);
  }

  void ImplementGeometry(const Ellipsoid&, void* data) {
    DRAKE_DEMAND(geom_id_.is_valid());
    auto& set = *static_cast<copyable_unique_ptr<ConvexSet>*>(data);
    set = std::make_unique<Hyperellipsoid>(query_, geom_id_, reference_frame_);
  }

  void ImplementGeometry(const HalfSpace&, void* data) {
    DRAKE_DEMAND(geom_id_.is_valid());
    auto& set = *static_cast<copyable_unique_ptr<ConvexSet>*>(data);
    set = std::make_unique<HPolyhedron>(query_, geom_id_, reference_frame_);
  }

  void ImplementGeometry(const Sphere&, void* data) {
    DRAKE_DEMAND(geom_id_.is_valid());
    auto& set = *static_cast<copyable_unique_ptr<ConvexSet>*>(data);
    set = std::make_unique<Hyperellipsoid>(query_, geom_id_, reference_frame_);
  }

  void ImplementGeometry(const Convex&, void* data) {
    DRAKE_DEMAND(geom_id_.is_valid());
    auto& set = *static_cast<copyable_unique_ptr<ConvexSet>*>(data);
    set = std::make_unique<VPolytope>(query_, geom_id_, reference_frame_);
  }

 private:
  const QueryObject<double>& query_{};
  std::optional<FrameId> reference_frame_{};
  GeometryId geom_id_{};
};

}  // namespace

ConvexSets MakeIrisObstacles(const QueryObject<double>& query_object,
                             std::optional<FrameId> reference_frame) {
  const SceneGraphInspector<double>& inspector = query_object.inspector();
  const GeometrySet all_ids(inspector.GetAllGeometryIds());
  const std::unordered_set<GeometryId> geom_ids =
      inspector.GetGeometryIds(all_ids, Role::kProximity);
  ConvexSets sets(geom_ids.size());

  IrisConvexSetMaker maker(query_object, reference_frame);
  int count = 0;
  for (GeometryId geom_id : geom_ids) {
    maker.set_geometry_id(geom_id);
    inspector.GetShape(geom_id).Reify(&maker, &sets[count++]);
  }
  return sets;
}

namespace {

// Takes q, p_AA, and p_BB and enforces that p_WA == p_WB.
class SamePointConstraint : public Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SamePointConstraint)

  SamePointConstraint(const MultibodyPlant<double>* plant,
                      const Context<double>& context)
      : Constraint(3, plant->num_positions() + 6, Vector3d::Zero(),
                            Vector3d::Zero()),
        plant_(plant),
        context_(plant->CreateDefaultContext()) {
    DRAKE_DEMAND(plant_ != nullptr);
    context_->SetTimeStateAndParametersFrom(context);
  }

  ~SamePointConstraint() override {}

  void set_frameA(const multibody::Frame<double>* frame) { frameA_ = frame; }

  void set_frameB(const multibody::Frame<double>* frame) { frameB_ = frame; }

  void EnableSymbolic() {
    if (symbolic_plant_ != nullptr) {
      return;
    }
    symbolic_plant_ = systems::System<double>::ToSymbolic(*plant_);
    symbolic_context_ = symbolic_plant_->CreateDefaultContext();
    symbolic_context_->SetTimeStateAndParametersFrom(*context_);
  }

 private:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override {
    DRAKE_DEMAND(frameA_ != nullptr);
    DRAKE_DEMAND(frameB_ != nullptr);
    VectorXd q = x.head(plant_->num_positions());
    Vector3d p_AA = x.template segment<3>(plant_->num_positions()),
             p_BB = x.template tail<3>();
    Vector3d p_WA, p_WB;
    plant_->SetPositions(context_.get(), q);
    plant_->CalcPointsPositions(*context_, *frameA_, p_AA,
                                plant_->world_frame(), &p_WA);
    plant_->CalcPointsPositions(*context_, *frameB_, p_BB,
                                plant_->world_frame(), &p_WB);
    *y = p_WA - p_WB;
  }

  // p_WA = X_WA(q)*p_AA
  // dp_WA = Jq_v_WA*dq + X_WA(q)*dp_AA
  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override {
    DRAKE_DEMAND(frameA_ != nullptr);
    DRAKE_DEMAND(frameB_ != nullptr);
    VectorX<AutoDiffXd> q = x.head(plant_->num_positions());
    Vector3<AutoDiffXd> p_AA = x.template segment<3>(plant_->num_positions()),
                        p_BB = x.template tail<3>();
    plant_->SetPositions(context_.get(), ExtractDoubleOrThrow(q));
    const RigidTransform<double>& X_WA =
        plant_->EvalBodyPoseInWorld(*context_, frameA_->body());
    const RigidTransform<double>& X_WB =
        plant_->EvalBodyPoseInWorld(*context_, frameB_->body());
    Eigen::Matrix3Xd Jq_v_WA(3, plant_->num_positions()),
        Jq_v_WB(3, plant_->num_positions());
    plant_->CalcJacobianTranslationalVelocity(
        *context_, JacobianWrtVariable::kQDot, *frameA_,
        ExtractDoubleOrThrow(p_AA), plant_->world_frame(),
        plant_->world_frame(), &Jq_v_WA);
    plant_->CalcJacobianTranslationalVelocity(
        *context_, JacobianWrtVariable::kQDot, *frameB_,
        ExtractDoubleOrThrow(p_BB), plant_->world_frame(),
        plant_->world_frame(), &Jq_v_WB);

    const Eigen::Vector3d y_val =
        X_WA * math::ExtractValue(p_AA) - X_WB * math::ExtractValue(p_BB);
    Eigen::Matrix3Xd dy(3, plant_->num_positions() + 6);
    dy << Jq_v_WA - Jq_v_WB, X_WA.rotation().matrix(),
        -X_WB.rotation().matrix();
    *y = math::InitializeAutoDiff(y_val, dy * math::ExtractGradient(x));
  }

  void DoEval(const Ref<const VectorX<symbolic::Variable>>& x,
              VectorX<symbolic::Expression>* y) const override {
    DRAKE_DEMAND(symbolic_plant_ != nullptr);
    DRAKE_DEMAND(frameA_ != nullptr);
    DRAKE_DEMAND(frameB_ != nullptr);
    const Frame<Expression>& frameA =
        symbolic_plant_->get_frame(frameA_->index());
    const Frame<Expression>& frameB =
        symbolic_plant_->get_frame(frameB_->index());
    VectorX<Expression> q = x.head(plant_->num_positions());
    Vector3<Expression> p_AA = x.template segment<3>(plant_->num_positions()),
                        p_BB = x.template tail<3>();
    Vector3<Expression> p_WA, p_WB;
    symbolic_plant_->SetPositions(symbolic_context_.get(), q);
    symbolic_plant_->CalcPointsPositions(*symbolic_context_, frameA, p_AA,
                                         symbolic_plant_->world_frame(), &p_WA);
    symbolic_plant_->CalcPointsPositions(*symbolic_context_, frameB, p_BB,
                                         symbolic_plant_->world_frame(), &p_WB);
    *y = p_WA - p_WB;
  }

  const MultibodyPlant<double>* const plant_;
  const multibody::Frame<double>* frameA_{nullptr};
  const multibody::Frame<double>* frameB_{nullptr};
  std::unique_ptr<Context<double>> context_;

  std::unique_ptr<MultibodyPlant<Expression>> symbolic_plant_{nullptr};
  std::unique_ptr<Context<Expression>> symbolic_context_{nullptr};
};

// Defines a MathematicalProgram to solve the problem
// min_q (q-d) CᵀC (q-d)
// s.t. setA in frameA and setB in frameB are in collision in q.
//      Aq ≤ b.
// where C, d are the matrix and center from the hyperellipsoid E.
//
// The class design supports repeated solutions of the (nearly) identical
// problem from different initial guesses.
class ClosestCollisionProgram {
 public:
  ClosestCollisionProgram(
      std::shared_ptr<SamePointConstraint> same_point_constraint,
      const multibody::Frame<double>& frameA,
      const multibody::Frame<double>& frameB, const ConvexSet& setA,
      const ConvexSet& setB, const Hyperellipsoid& E,
      const Eigen::Ref<const Eigen::MatrixXd>& A,
      const Eigen::Ref<const Eigen::VectorXd>& b) {
    q_ = prog_.NewContinuousVariables(A.cols(), "q");

    P_constraint_ = prog_.AddLinearConstraint(
        A,
        VectorXd::Constant(b.size(), -std::numeric_limits<double>::infinity()),
        b, q_);

    // Scale the objective so the eigenvalues are close to 1, using
    // scale*lambda_min = 1/scale*lambda_max.
    const MatrixXd Asq = E.A().transpose() * E.A();
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(Asq);
    const double scale = 1.0 / std::sqrt(es.eigenvalues().maxCoeff() *
                                         es.eigenvalues().minCoeff());
    prog_.AddQuadraticErrorCost(scale * Asq, E.center(), q_);

    auto p_AA = prog_.NewContinuousVariables<3>("p_AA");
    auto p_BB = prog_.NewContinuousVariables<3>("p_BB");
    setA.AddPointInSetConstraints(&prog_, p_AA);
    setB.AddPointInSetConstraints(&prog_, p_BB);

    same_point_constraint->set_frameA(&frameA);
    same_point_constraint->set_frameB(&frameB);
    prog_.AddConstraint(same_point_constraint, {q_, p_AA, p_BB});

    // Help nonlinear optimizers (e.g. SNOPT) avoid trivial local minima at the
    // origin.
    prog_.SetInitialGuess(p_AA, Vector3d::Constant(.01));
    prog_.SetInitialGuess(p_BB, Vector3d::Constant(.01));
  }

  void UpdatePolytope(const Eigen::Ref<const Eigen::MatrixXd>& A,
                      const Eigen::Ref<const Eigen::VectorXd>& b) {
    P_constraint_->evaluator()->UpdateCoefficients(
        A,
        VectorXd::Constant(b.size(), -std::numeric_limits<double>::infinity()),
        b);
  }

  // Returns true iff a collision is found.
  // Sets `closest` to an optimizing solution q*, if a solution is found.
  bool Solve(const solvers::SolverInterface& solver,
             const Eigen::Ref<const Eigen::VectorXd>& q_guess,
             VectorXd* closest) {
    prog_.SetInitialGuess(q_, q_guess);
    solvers::MathematicalProgramResult result;
    solver.Solve(prog_, std::nullopt, std::nullopt, &result);
    if (result.is_success()) {
      *closest = result.GetSolution(q_);
      return true;
    }
    return false;
  }

 private:
  MathematicalProgram prog_;
  solvers::VectorXDecisionVariable q_;
  std::optional<Binding<solvers::LinearConstraint>> P_constraint_{};
};

// Takes a constraint bound to another mathematical program and defines a new
// constraint that is the negation of one index and one (lower/upper) bound.
class CounterExampleConstraint : public Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CounterExampleConstraint)

  explicit CounterExampleConstraint(const MathematicalProgram* prog)
      : Constraint(1, prog->num_vars(),
                   Vector1d(-std::numeric_limits<double>::infinity()),
                   Vector1d::Constant(-kSolverConstraintTolerance - 1e-14)),
        prog_{prog} {
    DRAKE_DEMAND(prog != nullptr);
  }

  ~CounterExampleConstraint() = default;

  // Sets the actual constraint to be falsified, overwriting any previously set
  // constraints. The Binding<Constraint> must remain valid for the lifetime of
  // this object (or until a new Binding<Constraint> is set).
  void set(const Binding<Constraint>*
               binding_with_constraint_to_be_falsified,
           int index, bool falsify_lower_bound) {
    DRAKE_DEMAND(binding_with_constraint_to_be_falsified != nullptr);
    const int N =
        binding_with_constraint_to_be_falsified->evaluator()->num_constraints();
    DRAKE_DEMAND(index >= 0 && index < N);
    binding_  = binding_with_constraint_to_be_falsified;
    index_ = index;
    falsify_lower_bound_ = falsify_lower_bound;
  }

 private:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override {
    DRAKE_DEMAND(binding_ != nullptr);
    const double val = prog_->EvalBinding(*binding_, x)[index_];
    if (falsify_lower_bound_) {
      // val - lb <= -kSolverConstraintTolerance < 0.
      (*y)[0] = val - binding_->evaluator()->lower_bound()[index_];
    } else {
      // ub - val <= -kSolverConstraintTolerance < 0.
      (*y)[0] = binding_->evaluator()->upper_bound()[index_] - val;
    }
  }

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override {
    DRAKE_DEMAND(binding_ != nullptr);
    const AutoDiffXd val = prog_->EvalBinding(*binding_, x)[index_];
    if (falsify_lower_bound_) {
      // val - lb <= -kSolverConstraintTolerance < 0.
      (*y)[0] = val - binding_->evaluator()->lower_bound()[index_];
    } else {
      // ub - val <= -kSolverConstraintTolerance < 0.
      (*y)[0] = binding_->evaluator()->upper_bound()[index_] - val;
    }
  }

  void DoEval(const Ref<const VectorX<symbolic::Variable>>&,
              VectorX<symbolic::Expression>*) const override {
    // MathematicalProgram::EvalBinding doesn't support symbolic, and we
    // shouldn't get here.
    throw std::logic_error(
        "CounterExampleConstraint doesn't support DoEval for symbolic.");
  }

  const MathematicalProgram* prog_{};
  const Binding<Constraint>* binding_{};
  int index_{0};
  bool falsify_lower_bound_{true};

  // To find a counter-example for a constraints,
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
// s.t. counter-example-constraint
//      Aq ≤ b.
// where C, d are the matrix and center from the hyperellipsoid E.
//
// The class design supports repeated solutions of the (nearly) identical
// problem from different initial guesses.
class CounterExampleProgram {
 public:
  CounterExampleProgram(
      std::shared_ptr<CounterExampleConstraint> counter_example_constraint,
      const Hyperellipsoid& E, const Eigen::Ref<const Eigen::MatrixXd>& A,
      const Eigen::Ref<const Eigen::VectorXd>& b) {
    q_ = prog_.NewContinuousVariables(A.cols(), "q");

    P_constraint_ = prog_.AddLinearConstraint(
        A,
        VectorXd::Constant(b.size(), -std::numeric_limits<double>::infinity()),
        b, q_);
    // Scale the objective so the eigenvalues are close to 1, using
    // scale*lambda_min = 1/scale*lambda_max.
    const MatrixXd Asq = E.A().transpose() * E.A();
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(Asq);
    const double scale = 1.0 / std::sqrt(es.eigenvalues().maxCoeff() *
                                        es.eigenvalues().minCoeff());
    prog_.AddQuadraticErrorCost(scale * Asq, E.center(), q_);

    prog_.AddConstraint(counter_example_constraint, q_);
  }

  void UpdatePolytope(const Eigen::Ref<const Eigen::MatrixXd>& A,
                      const Eigen::Ref<const Eigen::VectorXd>& b) {
    P_constraint_->evaluator()->UpdateCoefficients(
        A,
        VectorXd::Constant(b.size(), -std::numeric_limits<double>::infinity()),
        b);
  }

  // Returns true iff a counter-example is found.
  // Sets `closest` to an optimizing solution q*, if a solution is found.
  bool Solve(const solvers::SolverInterface& solver,
             const Eigen::Ref<const Eigen::VectorXd>& q_guess,
             VectorXd* closest) {
    prog_.SetInitialGuess(q_, q_guess);
    solvers::MathematicalProgramResult result;
    solver.Solve(prog_, std::nullopt, std::nullopt, &result);
    if (result.is_success()) {
      *closest = result.GetSolution(q_);
      return true;
    }
    return false;
  }

 private:
  MathematicalProgram prog_;
  solvers::VectorXDecisionVariable q_;
  std::optional<Binding<solvers::LinearConstraint>> P_constraint_{};
};

// Add the tangent to the (scaled) ellipsoid at @p point as a
// constraint.
void AddTangentToPolytope(
    const Hyperellipsoid& E, const Eigen::Ref<const Eigen::VectorXd>& point,
    double configuration_space_margin,
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>* A,
    Eigen::VectorXd* b, int* num_constraints) {
  while (*num_constraints >= A->rows()) {
    // Increase pre-allocated polytope size.
    A->conservativeResize(A->rows() * 2, A->cols());
    b->conservativeResize(b->rows() * 2);
  }

  A->row(*num_constraints) =
      (E.A().transpose() * E.A() * (point - E.center())).normalized();
  (*b)[*num_constraints] =
      A->row(*num_constraints) * point - configuration_space_margin;
  if (A->row(*num_constraints)*E.center() > (*b)[*num_constraints]) {
    throw std::logic_error(
        "The current center of the IRIS region is within "
        "options.configuration_space_margin of being infeasible.  Check your "
        "sample point and/or any additional constraints you've passed in via "
        "the options. The configuration space surrounding the sample point "
        "must have an interior.");
  }
  *num_constraints += 1;
}

void MakeGuessFeasible(const HPolyhedron& P, const IrisOptions& options,
                 const VectorXd& closest, Eigen::VectorXd* guess) {
  const auto& A = P.A();
  const auto& b = P.b();
  const int N = A.rows();
  if (A.row(N - 1) * *guess - b[N - 1] > 0) {
    // The HPolyhedron uniform sampler wants feasible points.  First
    // try projecting the closest point back into the set.
    *guess = closest - options.configuration_space_margin *
                           A.row(N - 1).normalized().transpose();
    // If this causes a different constraint to be violated, then just
    // go back to the "center" of the set.
    if (!P.PointInSet(*guess, 1e-12)) {
      *guess = P.ChebyshevCenter();
    }
  }
}

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

}  // namespace

HPolyhedron IrisInConfigurationSpace(const MultibodyPlant<double>& plant,
                                     const Context<double>& context,
                                     const IrisOptions& options) {
  // Check the inputs.
  plant.ValidateContext(context);
  const int nq = plant.num_positions();
  const Eigen::VectorXd sample = plant.GetPositions(context);
  const int nc = static_cast<int>(options.configuration_obstacles.size());
  // Note: We require finite joint limits to define the bounding box for the
  // IRIS algorithm.
  DRAKE_DEMAND(plant.GetPositionLowerLimits().array().isFinite().all());
  DRAKE_DEMAND(plant.GetPositionUpperLimits().array().isFinite().all());
  DRAKE_DEMAND(options.num_collision_infeasible_samples >= 0);
  for (int i = 0; i < nc; ++i) {
    DRAKE_DEMAND(options.configuration_obstacles[i]->ambient_dimension() == nq);
    if (options.configuration_obstacles[i]->PointInSet(sample)) {
      throw std::runtime_error(
          fmt::format("The seed point is in configuration obstacle {}", i));
    }
  }

  if (options.prog_with_additional_constraints) {
    DRAKE_DEMAND(options.prog_with_additional_constraints->num_vars() == nq);
    DRAKE_DEMAND(options.num_additional_constraint_infeasible_samples >= 0);
  }

  // Make the polytope and ellipsoid.
  HPolyhedron P = HPolyhedron::MakeBox(plant.GetPositionLowerLimits(),
                                       plant.GetPositionUpperLimits());
  DRAKE_DEMAND(P.A().rows() == 2 * nq);
  const double kEpsilonEllipsoid = 1e-2;
  Hyperellipsoid E = Hyperellipsoid::MakeHypersphere(kEpsilonEllipsoid, sample);

  // Make all of the convex sets and supporting quantities.
  auto query_object =
      plant.get_geometry_query_input_port().Eval<QueryObject<double>>(context);
  const SceneGraphInspector<double>& inspector = query_object.inspector();
  IrisConvexSetMaker maker(query_object, inspector.world_frame_id());
  std::unordered_map<GeometryId, copyable_unique_ptr<ConvexSet>> sets{};
  std::unordered_map<GeometryId, const multibody::Frame<double>*> frames{};
  const std::unordered_set<GeometryId> geom_ids = inspector.GetGeometryIds(
      GeometrySet(inspector.GetAllGeometryIds()), Role::kProximity);
  copyable_unique_ptr<ConvexSet> temp_set;
  for (GeometryId geom_id : geom_ids) {
    // Make all sets in the local geometry frame.
    FrameId frame_id = inspector.GetFrameId(geom_id);
    maker.set_reference_frame(frame_id);
    maker.set_geometry_id(geom_id);
    inspector.GetShape(geom_id).Reify(&maker, &temp_set);
    sets.emplace(geom_id, std::move(temp_set));
    frames.emplace(geom_id, &plant.GetBodyFromFrameId(frame_id)->body_frame());
  }

  auto pairs = inspector.GetCollisionCandidates();
  const int n = static_cast<int>(pairs.size());
  auto same_point_constraint =
      std::make_shared<SamePointConstraint>(&plant, context);

  // As a surrogate for the true objective, the pairs are sorted by the distance
  // between each collision pair from the sample point configuration. This could
  // improve computation times and produce regions with fewer faces.
  std::vector<GeometryPairWithDistance> sorted_pairs;
  for (const auto& [geomA, geomB] : pairs) {
    const double distance =
        query_object.ComputeSignedDistancePairClosestPoints(geomA, geomB)
            .distance;
    if (distance < 0.0) {
      throw std::runtime_error(
          fmt::format("The seed point is in collision; geometry {} is in "
                      "collision with geometry {}",
                      inspector.GetName(geomA), inspector.GetName(geomB)));
    }
    sorted_pairs.emplace_back(geomA, geomB, distance);
  }
  std::sort(sorted_pairs.begin(), sorted_pairs.end());

  // On each iteration, we will build the collision-free polytope represented as
  // {x | A * x <= b}.  Here we pre-allocate matrices with a generous maximum
  // size.
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A(
      P.A().rows() + 2 * n + nc, nq);
  VectorXd b(P.A().rows() + 2 * n + nc);
  A.topRows(P.A().rows()) = P.A();
  b.head(P.A().rows()) = P.b();
  int num_initial_constraints = P.A().rows();

  std::shared_ptr<CounterExampleConstraint> counter_example_constraint{};
  std::unique_ptr<CounterExampleProgram> counter_example_prog{};
  std::vector<Binding<Constraint>> additional_constraint_bindings{};
  if (options.prog_with_additional_constraints) {
    counter_example_constraint = std::make_shared<CounterExampleConstraint>(
                options.prog_with_additional_constraints);
    additional_constraint_bindings =
        options.prog_with_additional_constraints->GetAllConstraints();
    // Fail fast if the seed point is infeasible.
    {
      if (!options.prog_with_additional_constraints->CheckSatisfied(
              additional_constraint_bindings, sample)) {
        throw std::runtime_error(
            "options.prog_with_additional_constraints is infeasible at the "
            "sample point. The seed point must be feasible.");
      }
    }
    // Handle bounding box and linear constraints as a special case (extracting
    // them from the additional_constraint_bindings).
    auto AddConstraint = [&](const Eigen::MatrixXd& new_A,
                             const Eigen::VectorXd& new_b,
                             const solvers::VectorXDecisionVariable& vars) {
      while (num_initial_constraints + new_A.rows() >= A.rows()) {
        // Increase pre-allocated polytope size.
        A.conservativeResize(A.rows() * 2, A.cols());
        b.conservativeResize(b.rows() * 2);
      }
      for (int i = 0; i < new_b.rows(); ++i) {
        if (!std::isinf(new_b[i])) {
          A.row(num_initial_constraints).setZero();
          for (int j = 0; j < vars.rows(); ++j) {
            const int index = options.prog_with_additional_constraints
                                  ->FindDecisionVariableIndex(vars[j]);
            A(num_initial_constraints, index) = new_A(i, j);
          }
          b[num_initial_constraints++] = new_b[i];
        }
      }
    };
    auto HandleLinearConstraints =
        [&](const auto& bindings) {
          for (const auto& binding : bindings) {
            AddConstraint(binding.evaluator()->GetDenseA(),
                          binding.evaluator()->upper_bound(),
                          binding.variables());
            AddConstraint(-binding.evaluator()->GetDenseA(),
                          -binding.evaluator()->lower_bound(),
                          binding.variables());
            auto pos = std::find(additional_constraint_bindings.begin(),
                                 additional_constraint_bindings.end(), binding);
            DRAKE_ASSERT(pos != additional_constraint_bindings.end());
            additional_constraint_bindings.erase(pos);
          }
        };
    HandleLinearConstraints(
        options.prog_with_additional_constraints->bounding_box_constraints());
    HandleLinearConstraints(
        options.prog_with_additional_constraints->linear_constraints());
    counter_example_prog = std::make_unique<CounterExampleProgram>(
        counter_example_constraint, E, A.topRows(num_initial_constraints),
        b.head(num_initial_constraints));

    P = HPolyhedron(A.topRows(num_initial_constraints),
                    b.head(num_initial_constraints));
  }

  DRAKE_THROW_UNLESS(P.PointInSet(sample, 1e-12));

  double best_volume = E.Volume();
  int iteration = 0;
  VectorXd closest(nq);
  RandomGenerator generator(options.random_seed);
  std::vector<std::pair<double, int>> scaling(nc);
  MatrixXd closest_points(nq, nc);

  auto solver = solvers::MakeFirstAvailableSolver(
      {solvers::SnoptSolver::id(), solvers::IpoptSolver::id()});

  while (true) {
    int num_constraints = num_initial_constraints;
    bool sample_point_requirement = true;
    VectorXd guess = sample;
    HPolyhedron P_candidate = P;
    DRAKE_ASSERT(best_volume > 0);
    // Find separating hyperplanes

    // Add constraints from configuration space obstacles to reduce the domain
    // for later optimization.
    if (options.configuration_obstacles.size() > 0) {
      const ConvexSets& obstacles = options.configuration_obstacles;
      for (int i = 0; i < nc; ++i) {
        const auto touch = E.MinimumUniformScalingToTouch(*obstacles[i]);
        scaling[i].first = touch.first;
        scaling[i].second = i;
        closest_points.col(i) = touch.second;
      }
      std::sort(scaling.begin(), scaling.end());

      for (int i = 0; i < nc; ++i) {
        // Only add a constraint if this obstacle still has overlap with the set
        // that has been constructed so far on this iteration.
        if (HPolyhedron(A.topRows(num_constraints), b.head(num_constraints))
                .IntersectsWith(*obstacles[scaling[i].second])) {
          const VectorXd point = closest_points.col(scaling[i].second);
          AddTangentToPolytope(E, point, 0.0, &A, &b, &num_constraints);
        }
      }

      if (options.require_sample_point_is_contained) {
        sample_point_requirement =
            ((A.topRows(num_constraints) * sample).array() <=
             b.head(num_constraints).array())
                .any();
      }
    }

    if (!sample_point_requirement) break;

    // Use the fast nonlinear optimizer until it fails
    // num_collision_infeasible_samples consecutive times.
    for (const auto& pair : sorted_pairs) {
      int consecutive_failures = 0;
      ClosestCollisionProgram prog(
          same_point_constraint, *frames.at(pair.geomA), *frames.at(pair.geomB),
          *sets.at(pair.geomA), *sets.at(pair.geomB), E,
          A.topRows(num_constraints), b.head(num_constraints));
      while (sample_point_requirement &&
             consecutive_failures <
                 options.num_collision_infeasible_samples) {
        if (prog.Solve(*solver, guess, &closest)) {
          consecutive_failures = 0;
          AddTangentToPolytope(E, closest, options.configuration_space_margin,
                               &A, &b, &num_constraints);
          P_candidate = HPolyhedron(A.topRows(num_constraints),
                          b.head(num_constraints));
          MakeGuessFeasible(P_candidate, options, closest, &guess);
          if (options.require_sample_point_is_contained) {
            sample_point_requirement =
                A.row(num_constraints - 1) * sample <= b(num_constraints - 1);
            if (!sample_point_requirement) break;
          }
          prog.UpdatePolytope(A.topRows(num_constraints),
                              b.head(num_constraints));
        } else {
          ++consecutive_failures;
        }
        guess = P_candidate.UniformSample(&generator, guess);
      }
    }

    if (!sample_point_requirement) break;

    if (options.prog_with_additional_constraints) {
      counter_example_prog->UpdatePolytope(A.topRows(num_constraints),
                                            b.head(num_constraints));
      for (const auto& binding : additional_constraint_bindings) {
        for (int index = 0; index < binding.evaluator()->num_constraints();
             ++index) {
          int consecutive_failures = 0;
          for (bool falsify_lower_bound : {true, false}) {
            if (falsify_lower_bound &&
                std::isinf(binding.evaluator()->lower_bound()[index])) {
              continue;
            }
            if (!falsify_lower_bound &&
                std::isinf(binding.evaluator()->upper_bound()[index])) {
              continue;
            }
            counter_example_constraint->set(&binding, index,
                                            falsify_lower_bound);
            while (consecutive_failures <
                   options.num_additional_constraint_infeasible_samples) {
              if (counter_example_prog->Solve(*solver, guess, &closest)) {
                consecutive_failures = 0;
                AddTangentToPolytope(E, closest,
                                     options.configuration_space_margin, &A, &b,
                                     &num_constraints);
                P_candidate = HPolyhedron(A.topRows(num_constraints),
                                          b.head(num_constraints));
                MakeGuessFeasible(P_candidate, options, closest, &guess);
                if (options.require_sample_point_is_contained) {
                  sample_point_requirement =
                      A.row(num_constraints - 1) * sample <=
                      b(num_constraints - 1);
                  if (!sample_point_requirement) break;
                }
                counter_example_prog->UpdatePolytope(A.topRows(num_constraints),
                                                     b.head(num_constraints));
              } else {
                ++consecutive_failures;
              }
              guess = P.UniformSample(&generator, guess);
            }
          }
        }
      }
    }

    if (!sample_point_requirement) break;

    P = HPolyhedron(A.topRows(num_constraints), b.head(num_constraints));

    iteration++;
    if (iteration >= options.iteration_limit) {
      break;
    }

    E = P.MaximumVolumeInscribedEllipsoid();
    const double volume = E.Volume();
    const double delta_volume = volume - best_volume;
    if (delta_volume <= options.termination_threshold) {
      break;
    }
    if (delta_volume / best_volume <= options.relative_termination_threshold) {
      break;
    }
    best_volume = volume;
  }
  return P;
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
