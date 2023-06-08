#include "drake/geometry/optimization/iris_internal.h"

#include <algorithm>
#include <limits>
#include <utility>

#include <fmt/format.h>

namespace drake {
namespace geometry {
namespace optimization {
namespace internal {
const double kInf = std::numeric_limits<double>::infinity();

using Eigen::MatrixXd;
using Eigen::VectorXd;

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

ClosestCollisionProgram::ClosestCollisionProgram(
    std::shared_ptr<SamePointConstraint> same_point_constraint,
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

  same_point_constraint->set_frameA(&frameA);
  same_point_constraint->set_frameB(&frameB);
  prog_.AddConstraint(same_point_constraint, {q_, p_AA, p_BB});

  // Help nonlinear optimizers (e.g. SNOPT) avoid trivial local minima at the
  // origin.
  prog_.SetInitialGuess(p_AA, Eigen::Vector3d::Constant(.01));
  prog_.SetInitialGuess(p_BB, Eigen::Vector3d::Constant(.01));
}

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
    const Eigen::Ref<const Eigen::VectorXd>& q_guess, VectorXd* closest) {
  prog_.SetInitialGuess(q_, q_guess);
  solvers::MathematicalProgramResult result;
  solver.Solve(prog_, std::nullopt, std::nullopt, &result);
  if (result.is_success()) {
    *closest = result.GetSolution(q_);
    return true;
  }
  return false;
}

namespace {
const planning::CollisionChecker& get_collision_checker(
    const planning::CollisionChecker* ptr) {
  if (ptr == nullptr) {
    throw std::runtime_error("get_collision_checker(): got a nullptr");
  }
  return *ptr;
}
}  // namespace

InCollisionConstraint::InCollisionConstraint(
    SortedPair<multibody::BodyIndex> body_pair,
    const planning::CollisionChecker* collision_checker,
    double influence_distance)
    : solvers::Constraint(
          1, get_collision_checker(collision_checker).plant().num_positions(),
          Vector1d(-kInf), Vector1d(0)),
      body_pair_{std::move(body_pair)},
      collision_checker_{collision_checker},
      influence_distance_{influence_distance} {
  if (collision_checker_->IsCollisionFilteredBetween(body_pair_.first(),
                                                     body_pair_.second())) {
    throw std::runtime_error(fmt::format(
        "InCollisionConstraint(): collision between the body {} and {} is "
        "filtered",
        collision_checker_->plant().get_body(body_pair_.first()).name(),
        collision_checker_->plant().get_body(body_pair_.second()).name()));
  }
}

void InCollisionConstraint::DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
                                   Eigen::VectorXd* y) const {
  y->resize(1);
  const planning::RobotClearance robot_clearance =
      collision_checker_->CalcRobotClearance(x, influence_distance_);
  // Loop through every entry in robot_clearance, find the smallest distance for
  // that pair of bodies.
  (*y)(0) = influence_distance_;
  for (int i = 0; i < robot_clearance.size(); ++i) {
    if (SortedPair<multibody::BodyIndex>(robot_clearance.robot_indices()[i],
                                         robot_clearance.other_indices()[i]) ==
        body_pair_) {
      (*y)(0) = std::min((*y)(0), robot_clearance.distances()(i));
    }
  }
}

void InCollisionConstraint::DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
                                   AutoDiffVecXd* y) const {
  y->resize(1);
  const Eigen::VectorXd q_val = math::ExtractValue(x);
  const Eigen::MatrixXd q_grad = math::ExtractGradient(x);
  const planning::RobotClearance robot_clearance =
      collision_checker_->CalcRobotClearance(q_val, influence_distance_);
  // Loop through every entry in robot_clearance, find the smallest distance for
  // that pair of bodies.
  (*y)(0).value() = influence_distance_;
  for (int i = 0; i < robot_clearance.size(); ++i) {
    if (SortedPair<multibody::BodyIndex>(robot_clearance.robot_indices()[i],
                                         robot_clearance.other_indices()[i]) ==
        body_pair_) {
      if (robot_clearance.distances()(i) < (*y)(0).value()) {
        (*y)(0).value() = robot_clearance.distances()(i);
        (*y)(0).derivatives() =
            (robot_clearance.jacobians().row(i) * q_grad).transpose();
      }
    }
  }
}

void InCollisionConstraint::DoEval(
    const Eigen::Ref<const VectorX<symbolic::Variable>>&,
    VectorX<symbolic::Expression>*) const {
  throw std::runtime_error(
      "InCollisionConstraint doesn't support Eval with symbolic variable.");
}
}  // namespace internal
}  // namespace optimization
}  // namespace geometry
}  // namespace drake
