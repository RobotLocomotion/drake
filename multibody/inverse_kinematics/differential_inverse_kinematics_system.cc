#include "drake/multibody/inverse_kinematics/differential_inverse_kinematics_system.h"

#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <fmt/ranges.h>

#include "drake/common/eigen_types.h"
#include "drake/common/text_logging.h"
#include "drake/multibody/inverse_kinematics/differential_inverse_kinematics.h"
#include "drake/multibody/parsing/scoped_names.h"
#include "drake/solvers/osqp_solver.h"
#include "drake/systems/framework/bus_value.h"

namespace drake {
namespace multibody {
namespace {

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using math::RigidTransformd;
using parsing::GetScopedFrameByName;
using planning::CollisionChecker;
using planning::CollisionCheckerContext;
using planning::DofMask;
using planning::JointLimits;
using planning::RobotClearance;
using solvers::Binding;
using solvers::EvaluatorBase;
using solvers::MathematicalProgram;
using solvers::MathematicalProgramResult;
using solvers::OsqpSolver;
using solvers::SolverOptions;
using solvers::VectorXDecisionVariable;
using systems::BasicVector;
using systems::BusValue;
using systems::Context;
using systems::ValueProducer;

using CallbackDetails = DifferentialInverseKinematicsSystem::CallbackDetails;
using DiagonalMatrixXd = Eigen::DiagonalMatrix<double, Eigen::Dynamic>;

constexpr int kSpatialVelocityRows =
    SpatialVelocity<double>::kSpatialVectorSize;

constexpr int kCartesianSpaceJacobianRows = 6;

constexpr double kInf = std::numeric_limits<double>::infinity();

}  // namespace

DifferentialInverseKinematicsSystem::Ingredient::~Ingredient() = default;

DiagonalMatrixXd
DifferentialInverseKinematicsSystem::Ingredient::BuildBlockDiagonalAxisSelector(
    const std::vector<const Frame<double>*>& frame_list,
    const string_unordered_map<Vector6d>& cartesian_axis_masks) {
  const int N = ssize(frame_list);
  DiagonalMatrixXd selector(6 * N);
  selector.setIdentity();
  for (int i = 0; i < N; ++i) {
    const std::string frame_name = frame_list[i]->scoped_name().to_string();
    if (auto iter = cartesian_axis_masks.find(frame_name);
        iter != cartesian_axis_masks.end()) {
      const Vector6d& mask = iter->second;
      selector.diagonal().segment<6>(6 * i) = mask;
    }
  }
  return selector;
}

DifferentialInverseKinematicsSystem::Recipe::Recipe() = default;

DifferentialInverseKinematicsSystem::Recipe::~Recipe() = default;

std::vector<Binding<EvaluatorBase>>
DifferentialInverseKinematicsSystem::Recipe::AddToProgram(
    CallbackDetails* details) const {
  std::vector<Binding<EvaluatorBase>> result;
  for (const auto& ingredient : ingredients_) {
    std::vector<Binding<EvaluatorBase>> added =
        ingredient->AddToProgram(details);
    result.insert(result.end(), std::make_move_iterator(added.begin()),
                  std::make_move_iterator(added.end()));
  }
  return result;
}

// TODO(jeremy-nimmer) The definition order in this file has been kept intact
// during refactoring to make reviews (and git history) easier to follow. Once
// all of the edits are finished, we should (in a separate commit) shuffle the
// code order in the cc file to match the header.

// TODO(jeremy-nimmer) The inconsistent local variable names in this file (e.g.,
// `ndof` vs `num_cartesian_...`) have been kept intact during refactoring to
// make reviews (and git history) easier to follow. Once all of the edits are
// finished, we should (in a separate commit) improve the names.

DifferentialInverseKinematicsSystem::LeastSquaresCost::LeastSquaresCost(
    const Config& config) {
  SetConfig(config);
}

DifferentialInverseKinematicsSystem::LeastSquaresCost::~LeastSquaresCost() =
    default;

void DifferentialInverseKinematicsSystem::LeastSquaresCost::SetConfig(
    const Config& config) {
  DRAKE_THROW_UNLESS(std::isfinite(config.cartesian_qp_weight));
  DRAKE_THROW_UNLESS(config.cartesian_qp_weight >= 0);
  for (const auto& [_, mask] : config.cartesian_axis_masks) {
    DRAKE_THROW_UNLESS(((mask.array() == 1.0) || (mask.array() == 0.0)).all());
    DRAKE_THROW_UNLESS((mask.array() == 1.0).any());
  }
  config_ = config;
}

std::vector<Binding<EvaluatorBase>>
DifferentialInverseKinematicsSystem::LeastSquaresCost::AddToProgram(
    CallbackDetails* details) const {
  DRAKE_DEMAND(details != nullptr);
  MathematicalProgram& prog = details->mathematical_program;
  const VectorXDecisionVariable& v_next = details->v_next;
  const std::vector<SpatialVelocity<double>>& Vd_TGlist = details->Vd_TGlist;
  const MatrixXd& Jv_TGs = details->Jv_TGs;
  const int num_cart_constraints = Vd_TGlist.size() * kSpatialVelocityRows;
  const int ndof = details->active_dof.count();
  DRAKE_DEMAND(num_cart_constraints > 0);
  DRAKE_DEMAND(Jv_TGs.rows() == num_cart_constraints);
  DRAKE_DEMAND(Jv_TGs.cols() == ndof);
  static_assert(sizeof(SpatialVelocity<double>) ==
                sizeof(double) * kSpatialVelocityRows);
  const Eigen::Map<const VectorXd> Vd_TGs(Vd_TGlist[0].get_coeffs().data(),
                                          num_cart_constraints);
  const bool is_convex = true;
  // Minimize |V - J*v|² = vᵀJᵀJv - 2VᵀJv + VᵀV.
  // Note: MP::AddQuadraticCost() documents that what is being optimized is:
  //    0.5*xᵀ*Q*x + bᵀ*x + c.
  // 0.5Q = JᵀJ    -->  Q = 2JᵀJ
  // bᵀ = -2VᵀJ    -->  b = -2JᵀV
  // c = VᵀV       -->  A constant term.
  const DiagonalMatrixXd selector = BuildBlockDiagonalAxisSelector(
      details->frame_list, config_.cartesian_axis_masks);
  const MatrixXd masked_Jv = selector * Jv_TGs;
  const VectorXd masked_Vd = selector * Vd_TGs;

  // TODO(SeanCurtis-TRI) When "use_legacy_implementation" is removed, then
  // `scale` is simply the constant 2 and doesn't need a named variable anymore.
  const double scale = config_.use_legacy_implementation ? 1 : 2;
  const MatrixXd Q_cart = scale * masked_Jv.transpose() * masked_Jv;
  const MatrixXd b_cart = -2 * masked_Jv.transpose() * masked_Vd;
  auto binding = prog.AddQuadraticCost(
      config_.cartesian_qp_weight * Q_cart,
      config_.cartesian_qp_weight * b_cart,
      config_.cartesian_qp_weight * masked_Vd.dot(masked_Vd), v_next,
      is_convex);
  binding.evaluator()->set_description("Least squares cost");
  return std::vector<Binding<EvaluatorBase>>{std::move(binding)};
}

DifferentialInverseKinematicsSystem::CollisionConstraint::CollisionConstraint(
    const Config& config) {
  SetConfig(config);
}

DifferentialInverseKinematicsSystem::CollisionConstraint::
    ~CollisionConstraint() = default;

void DifferentialInverseKinematicsSystem::CollisionConstraint::SetConfig(
    const Config& config) {
  DRAKE_THROW_UNLESS(std::isfinite(config.safety_distance));
  DRAKE_THROW_UNLESS(config.influence_distance >= 0);
  config_ = config;
}

void DifferentialInverseKinematicsSystem::CollisionConstraint::
    SetSelectDataForCollisionConstraintFunction(
        const SelectDataForCollisionConstraintFunction&
            select_data_for_collision_constraint) {
  select_data_for_collision_constraint_ = select_data_for_collision_constraint;
}

std::vector<Binding<EvaluatorBase>>
DifferentialInverseKinematicsSystem::CollisionConstraint::AddToProgram(
    CallbackDetails* details) const {
  DRAKE_DEMAND(details != nullptr);
  std::vector<Binding<EvaluatorBase>> result;
  MathematicalProgram& prog = details->mathematical_program;
  const VectorXDecisionVariable& v_next = details->v_next;
  const MultibodyPlant<double>& plant = details->collision_checker.plant();
  const Context<double>& plant_context = details->plant_context;
  const CollisionChecker& collision_checker = details->collision_checker;
  CollisionCheckerContext& collision_checker_context =
      details->collision_checker_context;
  const DofMask& active_dof = details->active_dof;
  const double dt = details->time_step;
  VectorXd dist;
  MatrixXd ddist_dq;
  const auto& robot_position = plant.GetPositions(plant_context);
  // TODO(aditya.bhat): Would be better to share context rather than
  // maintaining separate.
  const RobotClearance robot_clearance =
      collision_checker.CalcContextRobotClearance(&collision_checker_context,
                                                  robot_position,
                                                  config_.influence_distance);
  if (select_data_for_collision_constraint_ != nullptr) {
    // Use the passed-in constraint filtering function.
    select_data_for_collision_constraint_(active_dof, robot_clearance, &dist,
                                          &ddist_dq);
  } else {
    // Reduce the derivative columns to those for active dofs.
    dist = robot_clearance.distances();
    int num_active_dofs = active_dof.count();
    MatrixXd jacobians = robot_clearance.jacobians();
    ddist_dq.resize(jacobians.rows(), num_active_dofs);
    active_dof.GetColumnsFromMatrix(jacobians, &ddist_dq);
  }

  const int num_dist = dist.size();
  if (num_dist > 0) {
    // For the constraint to be well-formed, the number of derivative columns
    // must match the number of decision variables.
    DRAKE_DEMAND(ddist_dq.cols() == v_next.rows());

    // Formulate minimum distance directly using time-step:
    //   ϕᵢ + ∂ϕᵢ/∂q v Δt >= ϕₛ
    VectorXd dist_min = (config_.safety_distance - dist.array()).matrix() / dt;
    VectorXd dist_max = VectorXd::Constant(num_dist, kInf);
    auto binding =
        prog.AddLinearConstraint(ddist_dq, dist_min, dist_max, v_next);
    binding.evaluator()->set_description("Collision constraint");
    result.push_back(std::move(binding));
  }
  return result;
}

namespace {

/* Scales the velocity limits for the position limits.
@param q The current joint positions, active dofs only.
@param joint_limits The joint limits.
@return joint_limits_scaled The scaled joint limits.
The velocity limits are scaled to pad against the position limits.

TODO(jeremy-nimmer) Relocate this function closer to its sole user. */
JointLimits ScaleVelocityLimitsForPositionLimits(
    const VectorXd& q, const JointLimits& joint_limits, const double min_margin,
    const double influence_margin) {
  DRAKE_ASSERT(min_margin >= 0);
  DRAKE_ASSERT(influence_margin > min_margin);
  VectorXd v_limits_lower = joint_limits.velocity_lower();
  VectorXd v_limits_upper = joint_limits.velocity_upper();
  // Scale velocity limits to pad against position limits.
  for (int i = 0; i < joint_limits.num_positions(); ++i) {
    const double lower_margin = q[i] - joint_limits.position_lower()[i];
    const double upper_margin = joint_limits.position_upper()[i] - q[i];
    if (upper_margin < lower_margin) {
      // Closer to upper.
      const double scale =
          (upper_margin - min_margin) / (influence_margin - min_margin);
      v_limits_upper[i] *= std::clamp(scale, 0.0, 1.0);
    } else {
      // Closer to lower.
      const double scale =
          (lower_margin - min_margin) / (influence_margin - min_margin);
      v_limits_lower[i] *= std::clamp(scale, 0.0, 1.0);
    }
  }
  return JointLimits(joint_limits.position_lower(),
                     joint_limits.position_upper(), v_limits_lower,
                     v_limits_upper, joint_limits.acceleration_lower(),
                     joint_limits.acceleration_upper());
}

}  // namespace

DifferentialInverseKinematicsSystem::CartesianPositionLimitConstraint::
    CartesianPositionLimitConstraint(const Config& config) {
  SetConfig(config);
}

DifferentialInverseKinematicsSystem::CartesianPositionLimitConstraint::
    ~CartesianPositionLimitConstraint() = default;

void DifferentialInverseKinematicsSystem::CartesianPositionLimitConstraint::
    SetConfig(const Config& config) {
  DRAKE_THROW_UNLESS(
      (config.p_TG_next_lower.array() <= config.p_TG_next_upper.array()).all());
  config_ = config;
}

std::vector<Binding<EvaluatorBase>> DifferentialInverseKinematicsSystem::
    CartesianPositionLimitConstraint::AddToProgram(
        CallbackDetails* details) const {
  DRAKE_DEMAND(details != nullptr);
  MathematicalProgram& prog = details->mathematical_program;
  const VectorXDecisionVariable& v_next = details->v_next;
  const std::vector<RigidTransformd>& X_TGlist = details->X_TGlist;
  const MatrixXd& Jv_TGs = details->Jv_TGs;
  const double dt = details->time_step;
  std::vector<Binding<EvaluatorBase>> result;
  for (int i = 0; i < ssize(X_TGlist); ++i) {
    const Vector3d& p_TGi = X_TGlist[i].translation();
    auto binding = prog.AddLinearConstraint(
        Jv_TGs.middleRows<3>((6 * i) + 3) * dt, config_.p_TG_next_lower - p_TGi,
        config_.p_TG_next_upper - p_TGi, v_next);
    binding.evaluator()->set_description("Cartesian position limit");
    result.push_back(std::move(binding));
  }
  return result;
}

DifferentialInverseKinematicsSystem::CartesianVelocityLimitConstraint::
    CartesianVelocityLimitConstraint(const Config& config) {
  SetConfig(config);
}

DifferentialInverseKinematicsSystem::CartesianVelocityLimitConstraint::
    ~CartesianVelocityLimitConstraint() = default;

void DifferentialInverseKinematicsSystem::CartesianVelocityLimitConstraint::
    SetConfig(const Config& config) {
  DRAKE_THROW_UNLESS((config.V_next_TG_limit.array() >= 0).all());
  config_ = config;
}

std::vector<Binding<EvaluatorBase>> DifferentialInverseKinematicsSystem::
    CartesianVelocityLimitConstraint::AddToProgram(
        CallbackDetails* details) const {
  DRAKE_DEMAND(details != nullptr);
  MathematicalProgram& prog = details->mathematical_program;
  const std::vector<SpatialVelocity<double>> Vd_TGlist = details->Vd_TGlist;
  const MatrixXd& Jv_TGs = details->Jv_TGs;
  const VectorXDecisionVariable& v_next = details->v_next;
  VectorXd max_Vd_TGs(ssize(Vd_TGlist) * kSpatialVelocityRows);
  for (int i = 0; i < ssize(Vd_TGlist); ++i) {
    max_Vd_TGs.segment(kSpatialVelocityRows * i, kSpatialVelocityRows) =
        config_.V_next_TG_limit;
  }
  auto binding =
      prog.AddLinearConstraint(Jv_TGs, -max_Vd_TGs, max_Vd_TGs, v_next);
  binding.evaluator()->set_description("Cartesian velocity limit");
  return std::vector<Binding<EvaluatorBase>>{std::move(binding)};
}

DifferentialInverseKinematicsSystem::JointVelocityLimitConstraint::
    JointVelocityLimitConstraint(const Config& config,
                                 const JointLimits& joint_limits) {
  SetConfig(config);
  SetJointLimits(joint_limits);
}

DifferentialInverseKinematicsSystem::JointVelocityLimitConstraint::
    ~JointVelocityLimitConstraint() = default;

void DifferentialInverseKinematicsSystem::JointVelocityLimitConstraint::
    SetConfig(const Config& config) {
  DRAKE_THROW_UNLESS(std::isfinite(config.min_margin));
  DRAKE_THROW_UNLESS(config.min_margin >= 0);
  DRAKE_THROW_UNLESS(std::isfinite(config.influence_margin));
  DRAKE_THROW_UNLESS(config.influence_margin > config.min_margin);
  config_ = config;
}

void DifferentialInverseKinematicsSystem::JointVelocityLimitConstraint::
    SetJointLimits(const JointLimits& joint_limits) {
  joint_limits_ = joint_limits;
}

std::vector<Binding<EvaluatorBase>>
DifferentialInverseKinematicsSystem::JointVelocityLimitConstraint::AddToProgram(
    CallbackDetails* details) const {
  DRAKE_DEMAND(details != nullptr);
  MathematicalProgram& prog = details->mathematical_program;
  const VectorXDecisionVariable& v_next = details->v_next;
  const MultibodyPlant<double>& plant = details->collision_checker.plant();
  const Context<double>& plant_context = details->plant_context;
  const DofMask& active_dof = details->active_dof;
  const auto& robot_position = plant.GetPositions(plant_context);

  JointLimits joint_limits_scaled = ScaleVelocityLimitsForPositionLimits(
      active_dof.GetFromArray(robot_position), joint_limits_,
      config_.min_margin, config_.influence_margin);
  auto binding = prog.AddBoundingBoxConstraint(
      joint_limits_scaled.velocity_lower(),
      joint_limits_scaled.velocity_upper(), v_next);
  binding.evaluator()->set_description("Joint velocity limit");
  return std::vector<Binding<EvaluatorBase>>{std::move(binding)};
}

DifferentialInverseKinematicsSystem::JointCenteringCost::JointCenteringCost(
    const Config& config) {
  SetConfig(config);
}

DifferentialInverseKinematicsSystem::JointCenteringCost::~JointCenteringCost() =
    default;

void DifferentialInverseKinematicsSystem::JointCenteringCost::SetConfig(
    const Config& config) {
  DRAKE_THROW_UNLESS(std::isfinite(config.posture_gain));
  DRAKE_THROW_UNLESS(config.posture_gain >= 0);
  for (const auto& [_, mask] : config.cartesian_axis_masks) {
    DRAKE_THROW_UNLESS(((mask.array() == 1.0) || (mask.array() == 0.0)).all());
    DRAKE_THROW_UNLESS((mask.array() == 1.0).any());
  }
  config_ = config;
}

std::vector<Binding<EvaluatorBase>>
DifferentialInverseKinematicsSystem::JointCenteringCost::AddToProgram(
    CallbackDetails* details) const {
  DRAKE_DEMAND(details != nullptr);
  MathematicalProgram& prog = details->mathematical_program;
  const VectorXDecisionVariable& v_next = details->v_next;
  const MultibodyPlant<double>& plant = details->collision_checker.plant();
  const Context<double>& plant_context = details->plant_context;
  const DofMask& active_dof = details->active_dof;
  const VectorXd& nominal_posture = details->nominal_posture;
  const MatrixXd& Jv_TGs = details->Jv_TGs;
  std::vector<Binding<EvaluatorBase>> result;
  // Implements the joint centering cost in the form
  // |P(v_next - k(q_active_nominal - q_active))|²
  const DiagonalMatrixXd cartesian_axis_selector =
      BuildBlockDiagonalAxisSelector(details->frame_list,
                                     config_.cartesian_axis_masks);
  const Eigen::FullPivLU<MatrixXd> lu_Jv(cartesian_axis_selector * Jv_TGs);
  const int ndof = Jv_TGs.cols();
  if (lu_Jv.rank() < ndof) {
    const auto& robot_position = plant.GetPositions(plant_context);
    // TODO(jeremy.nimmer) This formulation has been working well in practice,
    // but if you look carefully you'll notice that the _scale_ of the basis
    // vectors in `P` might not be well-balanced. (Eigen doesn't promise
    // anything about their scale.) If the kernel isn't consistently scaled,
    // then from one configuration to another, the relative weight of the
    // centering term can change. Can it change so as to meaningfully counter
    // the effects of the relative weight between primary and secondary costs?
    // We don't know. The inconsistent scale can also contribute to varying
    // anisotropic effects in the secondary cost. If the columns are scaled
    // differently, the column with the larger magnitude will be more heavily
    // penalized than the column with the smaller magnitude. We should consider
    // explicitly normalizing the columns of the kernel so that we know that it
    // has consistent scale (both irrespective of q and across columns).
    const MatrixXd P = lu_Jv.kernel().transpose();
    const VectorXd q_active_nominal = active_dof.GetFromArray(nominal_posture);
    const VectorXd q_active = active_dof.GetFromArray(robot_position);
    auto binding = prog.Add2NormSquaredCost(
        P, P * config_.posture_gain * (q_active_nominal - q_active), v_next);
    binding.evaluator()->set_description("Joint centering cost");
    result.push_back(std::move(binding));
  }
  return result;
}

namespace {

void LogConstraintViolations(const MathematicalProgram& prog,
                             const MathematicalProgramResult& result) {
  std::vector<std::string> infeasible_constraint_names =
      result.GetInfeasibleConstraintNames(prog, std::nullopt);
  if (infeasible_constraint_names.empty()) {
    infeasible_constraint_names.emplace_back("none");
  }
  log()->warn(
      "QP failed to solve, returning zero velocity; the violated constraints "
      "were {}",
      fmt::join(infeasible_constraint_names, ", "));

  // Debugging information for all constraints.
  if (log()->should_log(spdlog::level::debug)) {
    for (const auto& binding : prog.GetAllConstraints()) {
      const auto& constraint = binding.evaluator();

      VectorXd value = result.EvalBinding(binding);
      VectorXd lower_bound = constraint->lower_bound();
      VectorXd upper_bound = constraint->upper_bound();

      for (int i = 0; i < value.size(); ++i) {
        log()->debug("{}, index {}, value = {}, bounds = [{}, {}]",
                     constraint->get_description(), i, value[i], lower_bound[i],
                     upper_bound[i]);
      }
    }
  }
}

VectorXd TrySolveQPAndFallbackToZero(const MathematicalProgram& prog,
                                     const VectorXDecisionVariable& v_next) {
  // For speed, we avoid using ChooseBestSolver every time (we know it's a QP)
  // and we hard-code to OSQP which is faster (though less accurate) than the
  // other QP solvers in Drake.
  OsqpSolver solver;
  SolverOptions solver_options;
  solver_options.SetOption(OsqpSolver::id(), "adaptive_rho_interval", 0);
  MathematicalProgramResult result = solver.Solve(prog, {}, solver_options);
  if (result.is_success()) {
    return result.GetSolution(v_next);
  }
  LogConstraintViolations(prog, result);
  return VectorXd::Zero(v_next.size());
}

/* Helper class for a cache entry used as "scratch" storage, where the cache
entry has no inputs / no invalidation, and is rather only used as temporary
storage to be overwritten as needed by other calculation code.

The challenging part here is having a shared_ptr<CollisionCheckerContext> as a
member field. That's the type returned by CollisionChecker so we need to stick
with it. However, we don't want shallow copy semantics: cloning a Context must
always deep copy all of its storage. Therefore, we mark this class non-copyable
and implement a Clone method that allocates a new context. */
class CollisionCheckerContextHolder {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CollisionCheckerContextHolder);

  explicit CollisionCheckerContextHolder(
      std::shared_ptr<const planning::CollisionChecker> collision_checker)
      : collision_checker_(std::move(collision_checker)),
        context_(collision_checker_->MakeStandaloneModelContext()) {}

  std::unique_ptr<CollisionCheckerContextHolder> Clone() const {
    return std::make_unique<CollisionCheckerContextHolder>(collision_checker_);
  }

  CollisionCheckerContext& get_mutable() const {
    DRAKE_DEMAND(context_ != nullptr);
    return *context_;
  }

 private:
  const std::shared_ptr<const planning::CollisionChecker> collision_checker_;
  std::shared_ptr<CollisionCheckerContext> context_;
};

}  // namespace

/* Each value on the input port for either desired cartesian poses or desired
cartesian velocities is reprocessed into this internal value which is easier to
work with. The desires stored here are always in terms of spatial velocities; if
the input desire was for a pose, it is converted to a velocity by dividing the
pose error by the time step. All vectors have the same size (the number of
desires). */
struct DifferentialInverseKinematicsSystem::CartesianDesires {
  /* The frame being controlled; a borrowed pointer into our control plant. */
  std::vector<const Frame<double>*> frame_list;
  /* The current pose of the controlled frame. */
  std::vector<RigidTransformd> X_TGlist;
  /* The commanded velocity of the controlled frame. */
  std::vector<SpatialVelocity<double>> Vd_TGlist;
};

DifferentialInverseKinematicsSystem::DifferentialInverseKinematicsSystem(
    std::shared_ptr<const Recipe> recipe, const std::string_view task_frame,
    std::shared_ptr<const CollisionChecker> collision_checker,
    const DofMask& active_dof, const double time_step, const double K_VX,
    const SpatialVelocity<double>& Vd_TG_limit)
    : recipe_(std::move(recipe)),
      collision_checker_(std::move(collision_checker)),
      active_dof_(active_dof),
      time_step_(time_step),
      K_VX_(K_VX),
      Vd_TG_limit_(Vd_TG_limit),
      task_frame_(&GetScopedFrameByName(plant(), task_frame)) {
  DRAKE_THROW_UNLESS(recipe_ != nullptr);
  DRAKE_THROW_UNLESS(collision_checker_ != nullptr);
  DRAKE_THROW_UNLESS(time_step > 0);
  DRAKE_THROW_UNLESS(K_VX > 0);
  DRAKE_THROW_UNLESS((Vd_TG_limit.get_coeffs().array() >= 0).all());

  input_port_index_position_ =
      this->DeclareVectorInputPort("position", plant().num_positions())
          .get_index();

  input_port_index_nominal_posture_ =
      this->DeclareVectorInputPort("nominal_posture", plant().num_positions())
          .get_index();

  input_port_index_desired_cartesian_poses_ =
      this->DeclareAbstractInputPort("desired_cartesian_poses",
                                     Value<BusValue>{})
          .get_index();

  input_port_index_desired_cartesian_velocities_ =
      this->DeclareAbstractInputPort("desired_cartesian_velocities",
                                     Value<BusValue>{})
          .get_index();

  // Declare cache entry for the multibody plant context.
  auto plant_context = plant().CreateDefaultContext();
  plant_context_cache_index_ =
      this->DeclareCacheEntry(
              "plant_context_cache", *plant_context,
              &DifferentialInverseKinematicsSystem::PrepareMultibodyContext,
              {this->input_port_ticket(get_input_port_position().get_index())})
          .cache_index();

  // Declare cache entry for the reprocessed input from either the desired
  // cartesian poses or desired cartesian velocities.
  cartesian_desires_cache_index_ =
      this->DeclareCacheEntry(
              "cartesian_desires_cache", CartesianDesires{},
              &DifferentialInverseKinematicsSystem::PrepareCartesianDesires,
              {this->input_port_ticket(
                   get_input_port_desired_cartesian_poses().get_index()),
               this->input_port_ticket(
                   get_input_port_desired_cartesian_velocities().get_index()),
               this->cache_entry_ticket(plant_context_cache_index_)})
          .cache_index();

  // Declare scratch storage for the collision checker context. We must use an
  // AllocateCallback for the entry, because we need each Context we create to
  // have a distinct shared_ptr in the holder. As with all "scratch" storage,
  // there is no Calc function -- the code that uses this storage will (re)set
  // its value as necessary during use.
  collision_checker_context_scratch_index_ =
      this->DeclareCacheEntry(
              "collision_checker_context_context_scratch",
              ValueProducer(
                  [this]() {
                    return AbstractValue::Make(CollisionCheckerContextHolder(
                        this->collision_checker_));
                  },
                  &ValueProducer::NoopCalc),
              {this->nothing_ticket()})
          .cache_index();

  output_port_index_commanded_velocity_ =
      this->DeclareVectorOutputPort(
              "commanded_velocity", active_dof_.count(),
              &DifferentialInverseKinematicsSystem::CalcCommandedVelocity,
              {this->all_input_ports_ticket(),
               this->cache_entry_ticket(plant_context_cache_index_),
               this->cache_entry_ticket(cartesian_desires_cache_index_)})
          .get_index();
}

DifferentialInverseKinematicsSystem::~DifferentialInverseKinematicsSystem() =
    default;

void DifferentialInverseKinematicsSystem::PrepareMultibodyContext(
    const Context<double>& context, Context<double>* plant_context) const {
  const VectorXd& position = get_input_port_position().Eval(context);

  // Assert that the input port values are finite. One possible reason it could
  // be NaN is if the initial position is not set in the diagram. It can be set
  // using `set_initial_position()`.
  DRAKE_THROW_UNLESS(position.allFinite());

  plant().SetPositions(plant_context, position);
}

void DifferentialInverseKinematicsSystem::PrepareCartesianDesires(
    const Context<double>& context, CartesianDesires* cartesian_desires) const {
  cartesian_desires->frame_list.clear();
  cartesian_desires->X_TGlist.clear();
  cartesian_desires->Vd_TGlist.clear();

  // Check that exactly one of the two cartesian desire ports is connected.
  const bool has_cartesian_positions_input =
      get_input_port_desired_cartesian_poses().HasValue(context);
  const bool has_cartesian_velocities_input =
      get_input_port_desired_cartesian_velocities().HasValue(context);
  DRAKE_THROW_UNLESS(has_cartesian_positions_input !=
                     has_cartesian_velocities_input);

  // Evaluate the connected input.
  const BusValue& desired = (has_cartesian_velocities_input
                                 ? get_input_port_desired_cartesian_velocities()
                                 : get_input_port_desired_cartesian_poses())
                                .Eval<BusValue>(context);

  // Grab the plant context so that we can compute kinematics.
  const auto& plant_context = this->get_cache_entry(plant_context_cache_index_)
                                  .template Eval<Context<double>>(context);

  // Loop over all controlled frames requested on the input port.
  for (const auto&& [frame_name, abstract_value] : desired) {
    const Frame<double>* frame_i = &GetScopedFrameByName(plant(), frame_name);
    const RigidTransformd X_TGi =
        plant().CalcRelativeTransform(plant_context, *task_frame_, *frame_i);
    SpatialVelocity<double> Vd_TGi;
    if (has_cartesian_velocities_input) {
      Vd_TGi = abstract_value.template get_value<SpatialVelocity<double>>();
      // TODO(jeremy.nimmer): Per #lbm-platform slack thread, velocity limit
      // clamping should also happen for desired velocities; move the limit
      // code from the `else` block below to run outside (after) the if-else.
    } else {
      const auto& desired_pose =
          abstract_value.template get_value<RigidTransformd>();
      const Vector6d dX_TGi = ComputePoseDiffInCommonFrame(X_TGi, desired_pose);
      Vd_TGi.get_coeffs() = K_VX_ * dX_TGi / time_step_;
      const VectorXd& limit = Vd_TG_limit_.get_coeffs();
      for (int i = 0; i < 6; ++i) {
        Vd_TGi.get_coeffs()(i) =
            std::clamp(Vd_TGi.get_coeffs()(i), -limit(i), limit(i));
      }
    }
    cartesian_desires->frame_list.push_back(frame_i);
    cartesian_desires->X_TGlist.push_back(X_TGi);
    cartesian_desires->Vd_TGlist.push_back(Vd_TGi);
  }
}

void DifferentialInverseKinematicsSystem::CalcCommandedVelocity(
    const Context<double>& context, BasicVector<double>* output) const {
  // Get the multibody plant context and cartesian desires.
  const auto& plant_context = this->get_cache_entry(plant_context_cache_index_)
                                  .template Eval<Context<double>>(context);
  const auto& cartesian_desires =
      this->get_cache_entry(cartesian_desires_cache_index_)
          .template Eval<CartesianDesires>(context);
  const auto& collision_checker_context_holder =
      this->get_cache_entry(collision_checker_context_scratch_index_)
          .template Eval<CollisionCheckerContextHolder>(context);
  CollisionCheckerContext& collision_checker_context =
      collision_checker_context_holder.get_mutable();
  const int num_desires = ssize(cartesian_desires.frame_list);

  MatrixXd Jv_TGs(kCartesianSpaceJacobianRows * num_desires,
                  active_dof_.count());
  int point_index = 0;

  const int nv = plant().num_velocities();
  for (int i = 0; i < num_desires; ++i) {
    MatrixXd jacobian_matrix(kCartesianSpaceJacobianRows, nv);
    plant().CalcJacobianSpatialVelocity(plant_context, JacobianWrtVariable::kV,
                                        *cartesian_desires.frame_list[i],
                                        Vector3d::Zero(), *task_frame_,
                                        *task_frame_, &jacobian_matrix);

    MatrixXd active_jacobian_matrix(kCartesianSpaceJacobianRows,
                                    active_dof_.count());
    active_dof_.GetColumnsFromMatrix(jacobian_matrix, &active_jacobian_matrix);
    Jv_TGs.middleRows<kCartesianSpaceJacobianRows>(
        kCartesianSpaceJacobianRows * point_index) = active_jacobian_matrix;

    point_index += 1;
  }

  const VectorXd& nominal_posture =
      get_input_port_nominal_posture().Eval(context);

  MathematicalProgram prog;
  VectorXDecisionVariable v_next =
      prog.NewContinuousVariables(active_dof_.count(), "v_next");
  CallbackDetails details{
      .mathematical_program = prog,
      .v_next = v_next,
      .plant_context = plant_context,
      .collision_checker = *collision_checker_,
      .collision_checker_context = collision_checker_context,
      .active_dof = active_dof_,
      .time_step = time_step_,
      .nominal_posture = nominal_posture,
      .frame_list = cartesian_desires.frame_list,
      .X_TGlist = cartesian_desires.X_TGlist,
      .Vd_TGlist = cartesian_desires.Vd_TGlist,
      .Jv_TGs = Jv_TGs,
  };
  recipe_->AddToProgram(&details);

  const VectorXd commanded_velocity = TrySolveQPAndFallbackToZero(prog, v_next);
  output->set_value(commanded_velocity);
}

}  // namespace multibody
}  // namespace drake
