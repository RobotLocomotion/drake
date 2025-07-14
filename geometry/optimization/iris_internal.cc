#include "drake/geometry/optimization/iris_internal.h"

#include <limits>

namespace drake {
namespace geometry {
namespace optimization {
namespace internal {

using Eigen::MatrixXd;
using Eigen::VectorXd;
using solvers::Binding;
using solvers::Constraint;

SamePointConstraint::SamePointConstraint(
    const multibody::MultibodyPlant<double>* plant,
    const systems::Context<double>& context)
    : Constraint(3, plant->num_positions() + 6, Eigen::Vector3d::Zero(),
                 Eigen::Vector3d::Zero()),
      plant_(plant),
      context_(plant->CreateDefaultContext()) {
  DRAKE_DEMAND(plant_ != nullptr);
  context_->SetTimeStateAndParametersFrom(context);
}

SamePointConstraint::~SamePointConstraint() = default;

void SamePointConstraint::EnableSymbolic() {
  if (symbolic_plant_ != nullptr) {
    return;
  }
  symbolic_plant_ = systems::System<double>::ToSymbolic(*plant_);
  symbolic_context_ = symbolic_plant_->CreateDefaultContext();
  symbolic_context_->SetTimeStateAndParametersFrom(*context_);
}

void SamePointConstraint::DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                                 Eigen::VectorXd* y) const {
  DRAKE_DEMAND(frameA_ != nullptr);
  DRAKE_DEMAND(frameB_ != nullptr);
  Eigen::VectorXd q = x.head(plant_->num_positions());
  Eigen::Vector3d p_AA = x.template segment<3>(plant_->num_positions()),
                  p_BB = x.template tail<3>();
  Eigen::Vector3d p_WA, p_WB;
  plant_->SetPositions(context_.get(), q);
  plant_->CalcPointsPositions(*context_, *frameA_, p_AA, plant_->world_frame(),
                              &p_WA);
  plant_->CalcPointsPositions(*context_, *frameB_, p_BB, plant_->world_frame(),
                              &p_WB);
  *y = p_WA - p_WB;
}

// p_WA = X_WA(q)*p_AA
// dp_WA = Jq_v_WA*dq + X_WA(q)*dp_AA
void SamePointConstraint::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                                 AutoDiffVecXd* y) const {
  DRAKE_DEMAND(frameA_ != nullptr);
  DRAKE_DEMAND(frameB_ != nullptr);
  VectorX<AutoDiffXd> q = x.head(plant_->num_positions());
  Vector3<AutoDiffXd> p_AA = x.template segment<3>(plant_->num_positions()),
                      p_BB = x.template tail<3>();
  plant_->SetPositions(context_.get(), ExtractDoubleOrThrow(q));
  const math::RigidTransform<double>& X_WA =
      plant_->EvalBodyPoseInWorld(*context_, frameA_->body());
  const math::RigidTransform<double>& X_WB =
      plant_->EvalBodyPoseInWorld(*context_, frameB_->body());
  Eigen::Matrix3Xd Jq_v_WA(3, plant_->num_positions()),
      Jq_v_WB(3, plant_->num_positions());
  plant_->CalcJacobianTranslationalVelocity(
      *context_, multibody::JacobianWrtVariable::kQDot, *frameA_,
      ExtractDoubleOrThrow(p_AA), plant_->world_frame(), plant_->world_frame(),
      &Jq_v_WA);
  plant_->CalcJacobianTranslationalVelocity(
      *context_, multibody::JacobianWrtVariable::kQDot, *frameB_,
      ExtractDoubleOrThrow(p_BB), plant_->world_frame(), plant_->world_frame(),
      &Jq_v_WB);

  const Eigen::Vector3d y_val =
      X_WA * math::ExtractValue(p_AA) - X_WB * math::ExtractValue(p_BB);
  Eigen::Matrix3Xd dy(3, plant_->num_positions() + 6);
  dy << Jq_v_WA - Jq_v_WB, X_WA.rotation().matrix(), -X_WB.rotation().matrix();
  *y = math::InitializeAutoDiff(y_val, dy * math::ExtractGradient(x));
}

void SamePointConstraint::DoEval(
    const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
    VectorX<symbolic::Expression>* y) const {
  DRAKE_DEMAND(symbolic_plant_ != nullptr);
  DRAKE_DEMAND(frameA_ != nullptr);
  DRAKE_DEMAND(frameB_ != nullptr);
  const multibody::Frame<symbolic::Expression>& frameA =
      symbolic_plant_->get_frame(frameA_->index());
  const multibody::Frame<symbolic::Expression>& frameB =
      symbolic_plant_->get_frame(frameB_->index());
  VectorX<symbolic::Expression> q = x.head(plant_->num_positions());
  Vector3<symbolic::Expression> p_AA = x.template segment<3>(
                                    plant_->num_positions()),
                                p_BB = x.template tail<3>();
  Vector3<symbolic::Expression> p_WA, p_WB;
  symbolic_plant_->SetPositions(symbolic_context_.get(), q);
  symbolic_plant_->CalcPointsPositions(*symbolic_context_, frameA, p_AA,
                                       symbolic_plant_->world_frame(), &p_WA);
  symbolic_plant_->CalcPointsPositions(*symbolic_context_, frameB, p_BB,
                                       symbolic_plant_->world_frame(), &p_WB);
  *y = p_WA - p_WB;
}

PointsBoundedDistanceConstraint::PointsBoundedDistanceConstraint(
    const multibody::MultibodyPlant<double>* plant,
    const systems::Context<double>& context, const double max_distance)
    : Constraint(1, plant->num_positions() + 6, Vector1d::Zero(),
                 Vector1d(max_distance * max_distance)),
      same_point_constraint_(plant, context) {
  DRAKE_THROW_UNLESS(max_distance >= 0.0);
}

PointsBoundedDistanceConstraint::~PointsBoundedDistanceConstraint() = default;

void PointsBoundedDistanceConstraint::DoEval(
    const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd* y) const {
  Eigen::VectorXd displacement;
  same_point_constraint_.Eval(x, &displacement);
  *y = Vector1d(displacement.squaredNorm());
}

void PointsBoundedDistanceConstraint::DoEval(
    const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd* y) const {
  AutoDiffVecXd displacement;
  same_point_constraint_.Eval(x, &displacement);
  *y = Vector1<AutoDiffXd>(displacement.squaredNorm());
}

void PointsBoundedDistanceConstraint::DoEval(
    const Eigen::Ref<const VectorX<symbolic::Variable>>& x,
    VectorX<symbolic::Expression>* y) const {
  VectorX<symbolic::Expression> displacement;
  same_point_constraint_.Eval(x, &displacement);
  *y = Vector1<symbolic::Expression>(displacement.squaredNorm());
}

ParameterizedSamePointConstraint::ParameterizedSamePointConstraint(
    const multibody::MultibodyPlant<double>* plant,
    const systems::Context<double>& context,
    const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>&
        parameterization_double,
    const std::function<AutoDiffVecXd(const AutoDiffVecXd&)>&
        parameterization_autodiff,
    int parameterization_dimension)
    : Constraint(3, parameterization_dimension + 6, Eigen::Vector3d::Zero(),
                 Eigen::Vector3d::Zero()),
      same_point_constraint_(plant, context),
      parameterization_double_(parameterization_double),
      parameterization_autodiff_(parameterization_autodiff),
      parameterization_dimension_(parameterization_dimension) {}

ParameterizedSamePointConstraint::~ParameterizedSamePointConstraint() = default;

template <typename T>
void ParameterizedSamePointConstraint::DoEvalGeneric(
    const Eigen::Ref<const VectorX<T>>& x, VectorX<T>* y,
    const std::function<VectorX<T>(const VectorX<T>&)>& parameterization)
    const {
  const VectorX<T> q_latent = x.head(parameterization_dimension_);
  const VectorX<T> q_full = parameterization(q_latent);
  VectorX<T> x_full(same_point_constraint_.num_vars());
  x_full << q_full, x.template tail<6>();
  same_point_constraint_.Eval(x_full, y);
}

void ParameterizedSamePointConstraint::DoEval(
    const Eigen::Ref<const VectorXd>& x, VectorXd* y) const {
  DoEvalGeneric<double>(x, y, parameterization_double_);
}

void ParameterizedSamePointConstraint::DoEval(
    const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd* y) const {
  DoEvalGeneric<AutoDiffXd>(x, y, parameterization_autodiff_);
}

void ParameterizedSamePointConstraint::DoEval(
    const Eigen::Ref<const VectorX<symbolic::Variable>>&,
    VectorX<symbolic::Expression>*) const {
  // TODO(cohnt): Consider supporting symbolic evaluation. This would require
  // modifying IrisParameterizationFunction.
  throw std::runtime_error(
      "ParameterizedSamePointConstraint does not support symbolic evaluation.");
}

ParameterizedPointsBoundedDistanceConstraint::
    ParameterizedPointsBoundedDistanceConstraint(
        const multibody::MultibodyPlant<double>* plant,
        const systems::Context<double>& context, const double max_distance,
        const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>&
            parameterization_double,
        const std::function<AutoDiffVecXd(const AutoDiffVecXd&)>&
            parameterization_autodiff,
        int parameterization_dimension)
    : Constraint(1, parameterization_dimension + 6, Vector1d::Zero(),
                 Vector1d(max_distance)),
      // Note that the bound we set for the member
      // points_bounded_distance_constraint_ doesn't matter, since we only ever
      // evaluate the constraint -- not its bounds.
      points_bounded_distance_constraint_(plant, context, 0),
      parameterization_double_(parameterization_double),
      parameterization_autodiff_(parameterization_autodiff),
      parameterization_dimension_(parameterization_dimension) {}

ParameterizedPointsBoundedDistanceConstraint::
    ~ParameterizedPointsBoundedDistanceConstraint() = default;

template <typename T>
void ParameterizedPointsBoundedDistanceConstraint::DoEvalGeneric(
    const Eigen::Ref<const VectorX<T>>& x, VectorX<T>* y,
    const std::function<VectorX<T>(const VectorX<T>&)>& parameterization)
    const {
  const VectorX<T> q_latent = x.head(parameterization_dimension_);
  const VectorX<T> q_full = parameterization(q_latent);
  VectorX<T> x_full(points_bounded_distance_constraint_.num_vars());
  x_full << q_full, x.template tail<6>();
  points_bounded_distance_constraint_.Eval(x_full, y);
}

void ParameterizedPointsBoundedDistanceConstraint::DoEval(
    const Eigen::Ref<const VectorXd>& x, VectorXd* y) const {
  DoEvalGeneric<double>(x, y, parameterization_double_);
}

void ParameterizedPointsBoundedDistanceConstraint::DoEval(
    const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd* y) const {
  DoEvalGeneric<AutoDiffXd>(x, y, parameterization_autodiff_);
}

void ParameterizedPointsBoundedDistanceConstraint::DoEval(
    const Eigen::Ref<const VectorX<symbolic::Variable>>&,
    VectorX<symbolic::Expression>*) const {
  // TODO(cohnt): Consider supporting symbolic evaluation. This would require
  // modifying IrisParameterizationFunction.
  throw std::runtime_error(
      "ParameterizedPointsBoundedDistanceConstraint does not support symbolic "
      "evaluation.");
}

ClosestCollisionProgram::ClosestCollisionProgram(
    AcceptableConstraint same_point_constraint,
    const multibody::Frame<double>& frameA,
    const multibody::Frame<double>& frameB, const ConvexSet& setA,
    const ConvexSet& setB, const Hyperellipsoid& E,
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::VectorXd>& b) {
  q_ = prog_.NewContinuousVariables(A.cols(), "q");

  P_constraint_ = prog_.AddLinearConstraint(
      A,
      Eigen::VectorXd::Constant(b.size(),
                                -std::numeric_limits<double>::infinity()),
      b, q_);

  // Scale the objective so the eigenvalues are close to 1, using
  // scale*lambda_min = 1/scale*lambda_max.
  const Eigen::MatrixXd Asq = E.A().transpose() * E.A();
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(Asq);
  const double scale = 1.0 / std::sqrt(es.eigenvalues().maxCoeff() *
                                       es.eigenvalues().minCoeff());
  prog_.AddQuadraticErrorCost(scale * Asq, E.center(), q_);

  auto p_AA = prog_.NewContinuousVariables<3>("p_AA");
  auto p_BB = prog_.NewContinuousVariables<3>("p_BB");
  setA.AddPointInSetConstraints(&prog_, p_AA);
  setB.AddPointInSetConstraints(&prog_, p_BB);

  std::visit(
      [&](auto& constraint_ptr) {
        constraint_ptr->set_frameA(&frameA);
        constraint_ptr->set_frameB(&frameB);
        prog_.AddConstraint(constraint_ptr, {q_, p_AA, p_BB});
      },
      same_point_constraint);

  // Help nonlinear optimizers (e.g. SNOPT) avoid trivial local minima at the
  // origin.
  prog_.SetInitialGuess(p_AA, Eigen::Vector3d::Constant(.01));
  prog_.SetInitialGuess(p_BB, Eigen::Vector3d::Constant(.01));
}

ClosestCollisionProgram::~ClosestCollisionProgram() = default;

void ClosestCollisionProgram::UpdatePolytope(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::VectorXd>& b) {
  P_constraint_->evaluator()->UpdateCoefficients(
      A, VectorXd::Constant(b.size(), -std::numeric_limits<double>::infinity()),
      b);
}

// Returns true iff a collision is found.
// Sets `closest` to an optimizing solution q*, if a solution is found.
bool ClosestCollisionProgram::Solve(
    const solvers::SolverInterface& solver,
    const Eigen::Ref<const Eigen::VectorXd>& q_guess,
    const std::optional<solvers::SolverOptions>& solver_options,
    VectorXd* closest) {
  prog_.SetInitialGuess(q_, q_guess);
  solvers::MathematicalProgramResult result;
  solver.Solve(prog_, std::nullopt, solver_options, &result);
  if (result.is_success()) {
    *closest = result.GetSolution(q_);
    return true;
  }
  return false;
}

void CounterexampleConstraint::set(
    const Binding<Constraint>* binding_with_constraint_to_be_falsified,
    int index, bool falsify_lower_bound) {
  DRAKE_DEMAND(binding_with_constraint_to_be_falsified != nullptr);
  const int N =
      binding_with_constraint_to_be_falsified->evaluator()->num_constraints();
  DRAKE_DEMAND(index >= 0 && index < N);
  binding_ = binding_with_constraint_to_be_falsified;
  index_ = index;
  falsify_lower_bound_ = falsify_lower_bound;
}

void CounterexampleConstraint::DoEval(const Eigen::Ref<const VectorXd>& x,
                                      VectorXd* y) const {
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

void CounterexampleConstraint::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                                      AutoDiffVecXd* y) const {
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

CounterexampleProgram::CounterexampleProgram(
    std::shared_ptr<CounterexampleConstraint> counter_example_constraint,
    const Hyperellipsoid& E, const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::VectorXd>& b) {
  q_ = prog_.NewContinuousVariables(A.cols(), "q");

  P_constraint_ = prog_.AddLinearConstraint(
      A, VectorXd::Constant(b.size(), -std::numeric_limits<double>::infinity()),
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

void CounterexampleProgram::UpdatePolytope(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::VectorXd>& b) {
  P_constraint_->evaluator()->UpdateCoefficients(
      A, VectorXd::Constant(b.size(), -std::numeric_limits<double>::infinity()),
      b);
}

bool CounterexampleProgram::Solve(
    const solvers::SolverInterface& solver,
    const Eigen::Ref<const Eigen::VectorXd>& q_guess,
    const std::optional<solvers::SolverOptions>& solver_options,
    VectorXd* closest) {
  prog_.SetInitialGuess(q_, q_guess);
  solvers::MathematicalProgramResult result;
  solver.Solve(prog_, std::nullopt, solver_options, &result);
  if (result.is_success()) {
    *closest = result.GetSolution(q_);
    return true;
  }
  return false;
}

void IrisConvexSetMaker::ImplementGeometry(const Box&, void* data) {
  DRAKE_DEMAND(geom_id_.is_valid());
  auto& set = *static_cast<copyable_unique_ptr<ConvexSet>*>(data);
  // Note: We choose HPolyhedron over VPolytope here, but the IRIS paper
  // discusses a significant performance improvement using a "least-distance
  // programming" instance from CVXGEN that exploited the VPolytope
  // representation.  So we may wish to revisit this.
  set = std::make_unique<HPolyhedron>(query_, geom_id_, reference_frame_);
}

void IrisConvexSetMaker::ImplementGeometry(const Capsule&, void* data) {
  DRAKE_DEMAND(geom_id_.is_valid());
  auto& set = *static_cast<copyable_unique_ptr<ConvexSet>*>(data);
  set = std::make_unique<MinkowskiSum>(query_, geom_id_, reference_frame_);
}

void IrisConvexSetMaker::ImplementGeometry(const Cylinder&, void* data) {
  DRAKE_DEMAND(geom_id_.is_valid());
  auto& set = *static_cast<copyable_unique_ptr<ConvexSet>*>(data);
  set = std::make_unique<CartesianProduct>(query_, geom_id_, reference_frame_);
}

void IrisConvexSetMaker::ImplementGeometry(const Ellipsoid&, void* data) {
  DRAKE_DEMAND(geom_id_.is_valid());
  auto& set = *static_cast<copyable_unique_ptr<ConvexSet>*>(data);
  set = std::make_unique<Hyperellipsoid>(query_, geom_id_, reference_frame_);
}

void IrisConvexSetMaker::ImplementGeometry(const HalfSpace&, void* data) {
  DRAKE_DEMAND(geom_id_.is_valid());
  auto& set = *static_cast<copyable_unique_ptr<ConvexSet>*>(data);
  set = std::make_unique<HPolyhedron>(query_, geom_id_, reference_frame_);
}

void IrisConvexSetMaker::ImplementGeometry(const Sphere&, void* data) {
  DRAKE_DEMAND(geom_id_.is_valid());
  auto& set = *static_cast<copyable_unique_ptr<ConvexSet>*>(data);
  set = std::make_unique<Hyperellipsoid>(query_, geom_id_, reference_frame_);
}

void IrisConvexSetMaker::ImplementGeometry(const Convex&, void* data) {
  DRAKE_DEMAND(geom_id_.is_valid());
  auto& set = *static_cast<copyable_unique_ptr<ConvexSet>*>(data);
  set = std::make_unique<VPolytope>(query_, geom_id_, reference_frame_);
}

void IrisConvexSetMaker::ImplementGeometry(const Mesh&, void* data) {
  DRAKE_DEMAND(geom_id_.is_valid());
  auto& set = *static_cast<copyable_unique_ptr<ConvexSet>*>(data);
  set = std::make_unique<VPolytope>(query_, geom_id_, reference_frame_);
}

}  // namespace internal
}  // namespace optimization
}  // namespace geometry
}  // namespace drake
