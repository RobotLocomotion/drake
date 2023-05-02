#include "drake/planning/trajectory_optimization/hybrid_multibody_collocation.h"

#include <limits>

#include "drake/multibody/inverse_kinematics/distance_constraint.h"
#include "drake/multibody/optimization/contact_wrench_evaluator.h"
#include "drake/multibody/optimization/sliding_friction_complementarity_constraint.h"
#include "drake/multibody/optimization/static_friction_cone_constraint.h"
#include "drake/multibody/plant/externally_applied_spatial_force.h"

namespace drake {
namespace planning {
namespace trajectory_optimization {

using Eigen::Map;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::VectorBlock;
using Eigen::VectorXd;
using math::AreAutoDiffVecXdEqual;
using multibody::Frame;
using multibody::JacobianWrtVariable;
using multibody::MultibodyPlant;
using solvers::MathematicalProgram;
using solvers::MatrixXDecisionVariable;
using solvers::VectorXDecisionVariable;
using systems::Context;
using systems::LeafSystem;
using trajectories::PiecewisePolynomial;

const double kInf = std::numeric_limits<double>::infinity();

using ContactPair = SortedPair<geometry::GeometryId>;
using ContactPairs = std::set<ContactPair>;

namespace internal {

class ConstrainedDirectCollocationConstraint : public solvers::Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ConstrainedDirectCollocationConstraint)

  ConstrainedDirectCollocationConstraint(
      const RobotDiagram<AutoDiffXd>* robot_diagram,
      const ContactPairs& in_contact, Context<AutoDiffXd>* context_sample,
      Context<AutoDiffXd>* context_next_sample,
      Context<AutoDiffXd>* context_collocation,
      bool add_next_sample_kinematics = false)
      : Constraint(
            robot_diagram->plant().num_multibody_states() +
                (9 + (add_next_sample_kinematics ? 9 : 0)) * in_contact.size(),
            1 + (2 * robot_diagram->plant().num_multibody_states()) +
                (2 * robot_diagram->plant().get_actuation_input_port().size()) +
                (4 * 3 * in_contact.size())),
        robot_diagram_(robot_diagram),
        in_contact_(in_contact),
        context_sample_(context_sample),
        context_next_sample_(context_next_sample),
        context_collocation_(context_collocation),
        add_next_sample_kinematics_(add_next_sample_kinematics),
        num_inputs_(robot_diagram_->plant().get_actuation_input_port().size()),
        num_contacts_(in_contact_.size()),
        spatial_force_(2 * num_contacts_) {
    UpdateLowerBound(VectorXd::Zero(num_constraints()));
    UpdateUpperBound(VectorXd::Zero(num_constraints()));
    const MultibodyPlant<AutoDiffXd>& plant = robot_diagram_->plant();
    auto& inspector = robot_diagram_->scene_graph().model_inspector();
    int index = 0;
    for (const ContactPair& contact_pair : in_contact_) {
      spatial_force_[index].body_index =
          plant.GetBodyFromFrameId(inspector.GetFrameId(contact_pair.first()))
              ->index();
      spatial_force_[index].F_Bq_W.rotational().setZero();
      ++index;
      spatial_force_[index].body_index =
          plant.GetBodyFromFrameId(inspector.GetFrameId(contact_pair.second()))
              ->index();
      spatial_force_[index].F_Bq_W.rotational().setZero();
      ++index;
    }
  }

  const ContactPairs& in_contact() const { return in_contact_; }

  AutoDiffVecXd CalcDynamics(const Eigen::Ref<const AutoDiffVecXd>& state,
                             const Eigen::Ref<const AutoDiffVecXd>& input,
                             const Eigen::Ref<const AutoDiffVecXd>& force,
                             Context<AutoDiffXd>* context) const {
    const MultibodyPlant<AutoDiffXd>& plant = robot_diagram_->plant();
    Context<AutoDiffXd>* plant_context =
        &robot_diagram_->GetMutableSubsystemContext(plant, context);
    const systems::InputPort<AutoDiffXd>& actuation_port =
        plant.get_actuation_input_port();
    if (!actuation_port.HasValue(*plant_context) ||
        !AreAutoDiffVecXdEqual(input, actuation_port.Eval(*plant_context))) {
      actuation_port.FixValue(plant_context, input);
    }
    if (!AreAutoDiffVecXdEqual(
            state, plant.GetPositionsAndVelocities(*plant_context))) {
      plant.SetPositionsAndVelocities(plant_context, state);
    }

    const auto& query_object =
        plant.get_geometry_query_input_port()
            .template Eval<geometry::QueryObject<AutoDiffXd>>(*plant_context);
    auto& inspector = query_object.inspector();

    int spatial_force_index = 0;
    int force_index = 0;
    for (const ContactPair& contact_pair : in_contact_) {
      geometry::SignedDistancePair<AutoDiffXd> distance_pair =
          query_object.ComputeSignedDistancePairClosestPoints(
              contact_pair.first(), contact_pair.second());
      spatial_force_[spatial_force_index].p_BoBq_B =
          inspector.GetPoseInFrame(distance_pair.id_A)
              .template cast<AutoDiffXd>() *
          distance_pair.p_ACa;
      spatial_force_[spatial_force_index].F_Bq_W.translational() =
          -force.template segment<3>(3 * force_index);
      ++spatial_force_index;
      spatial_force_[spatial_force_index].p_BoBq_B =
          inspector.GetPoseInFrame(distance_pair.id_B)
              .template cast<AutoDiffXd>() *
          distance_pair.p_BCb;
      spatial_force_[spatial_force_index].F_Bq_W.translational() =
          force.template segment<3>(3 * force_index);
      ++spatial_force_index;
      ++force_index;
    }
    plant.get_applied_spatial_force_input_port().FixValue(plant_context,
                                                          spatial_force_);

    return plant.EvalTimeDerivatives(*plant_context).CopyToVector();
  }

 private:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override {
    AutoDiffVecXd y_t;
    Eval(x.cast<AutoDiffXd>(), &y_t);
    *y = math::ExtractValue(y_t);
  }

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override {
    const MultibodyPlant<AutoDiffXd>& plant = robot_diagram_->plant();
    DRAKE_DEMAND(x.size() == num_vars());
    y->resize(num_constraints());

    // Extract our input variables:
    // h - current time (breakpoint)
    // x0, x1 state vector at time steps k, k+1
    // u0, u1 input vector at time steps k, k+1
    // f0, f1 is force vector at k, k+1
    // fbar is the force correction (at the collocation)
    // vbar is the velocity correction (at the collocation)
    int index = 0;
    const AutoDiffXd h = x(index++);
    const auto x0 = x.segment(index, plant.num_multibody_states());
    index += plant.num_multibody_states();
    const auto x1 = x.segment(index, plant.num_multibody_states());
    index += plant.num_multibody_states();
    const auto u0 = x.segment(index, num_inputs_);
    index += num_inputs_;
    const auto u1 = x.segment(index, num_inputs_);
    index += num_inputs_;
    const auto f0 = x.segment(index, 3 * num_contacts_);
    index += 3 * num_contacts_;
    const auto f1 = x.segment(index, 3 * num_contacts_);
    index += 3 * num_contacts_;
    const auto fbar = x.segment(index, 3 * num_contacts_);
    index += 3 * num_contacts_;
    const auto vbar = x.segment(index, 3 * num_contacts_);
    index += 3 * num_contacts_;
    DRAKE_DEMAND(index == num_vars());

    index = 0;
    auto collocation = y->segment(index, plant.num_multibody_states());
    index += plant.num_multibody_states();
    auto phi0 = y->segment(index, 3 * num_contacts_);
    index += 3 * num_contacts_;
    auto psi0 = y->segment(index, 3 * num_contacts_);
    index += 3 * num_contacts_;
    auto alpha0 = y->segment(index, 3 * num_contacts_);
    index += 3 * num_contacts_;

    AutoDiffVecXd xdot0 = CalcDynamics(x0, u0, f0, context_sample_);
    CalcKinematicConstraints(*context_sample_, x0, xdot0, &phi0, &psi0,
                             &alpha0);
    AutoDiffVecXd xdot1 = CalcDynamics(x1, u1, f1, context_next_sample_);
    if (add_next_sample_kinematics_) {
      auto phi1 = y->segment(index, 3 * num_contacts_);
      index += 3 * num_contacts_;
      auto psi1 = y->segment(index, 3 * num_contacts_);
      index += 3 * num_contacts_;
      auto alpha1 = y->segment(index, 3 * num_contacts_);
      index += 3 * num_contacts_;
      CalcKinematicConstraints(*context_next_sample_, x1, xdot1, &phi1, &psi1,
                               &alpha1);
    }
    DRAKE_DEMAND(index == num_constraints());

    // Cubic interpolation to get xcol and xdotcol.
    const AutoDiffVecXd xcol = 0.5 * (x0 + x1) + h / 8 * (xdot0 - xdot1);
    const AutoDiffVecXd xdotcol = -1.5 * (x0 - x1) / h - .25 * (xdot0 + xdot1);

    AutoDiffVecXd g =
        CalcDynamics(xcol, 0.5 * (u0 + u1), fbar, context_collocation_);
    AddInVelocityCorrection(*context_collocation_, vbar, &g);
    collocation = xdotcol - g;
  }

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>&,
              VectorX<symbolic::Expression>*) const override {
    throw std::logic_error(
        "ConstrainedDirectCollocationConstraint does not support symbolic "
        "evaluation.");
  }

  // Note: This depends on member variables updated in CalcDynamics.
  void CalcKinematicConstraints(const Context<AutoDiffXd>& context,
                                const Eigen::Ref<const AutoDiffVecXd>& x,
                                const Eigen::Ref<const AutoDiffVecXd>& xdot,
                                EigenPtr<AutoDiffVecXd> phi,
                                EigenPtr<AutoDiffVecXd> psi,
                                EigenPtr<AutoDiffVecXd> alpha) const {
    const MultibodyPlant<AutoDiffXd>& plant = robot_diagram_->plant();
    const Context<AutoDiffXd>& plant_context =
        robot_diagram_->GetSubsystemContext(plant, context);
    phi->resize(3 * num_contacts_);
    psi->resize(3 * num_contacts_);
    alpha->resize(3 * num_contacts_);

    int spatial_force_index = 0;
    Vector3<AutoDiffXd> p_WAq, p_WBq;
    Matrix3X<AutoDiffXd> J_WAq(3, plant.num_velocities()),
        J_WBq(3, plant.num_velocities()), Jdotv_WAq, Jdotv_WBq;
    for (int i = 0; i < num_contacts_; ++i) {
      const Frame<AutoDiffXd>& A =
          plant.get_body(spatial_force_[spatial_force_index].body_index)
              .body_frame();
      plant.CalcPointsPositions(plant_context, A,
                                spatial_force_[spatial_force_index].p_BoBq_B,
                                plant.world_frame(), &p_WAq);
      plant.CalcJacobianTranslationalVelocity(
          plant_context, JacobianWrtVariable::kV, A,
          spatial_force_[spatial_force_index].p_BoBq_B, plant.world_frame(),
          plant.world_frame(), &J_WAq);
      Jdotv_WAq = plant.CalcBiasTranslationalAcceleration(
          plant_context, JacobianWrtVariable::kV, A,
          spatial_force_[spatial_force_index].p_BoBq_B, plant.world_frame(),
          plant.world_frame());
      ++spatial_force_index;
      const Frame<AutoDiffXd>& B =
          plant.get_body(spatial_force_[spatial_force_index].body_index)
              .body_frame();
      plant.CalcPointsPositions(plant_context, B,
                                spatial_force_[spatial_force_index].p_BoBq_B,
                                plant.world_frame(), &p_WBq);
      plant.CalcJacobianTranslationalVelocity(
          plant_context, JacobianWrtVariable::kV, B,
          spatial_force_[spatial_force_index].p_BoBq_B, plant.world_frame(),
          plant.world_frame(), &J_WBq);
      Jdotv_WBq = plant.CalcBiasTranslationalAcceleration(
          plant_context, JacobianWrtVariable::kV, B,
          spatial_force_[spatial_force_index].p_BoBq_B, plant.world_frame(),
          plant.world_frame());
      ++spatial_force_index;
      phi->segment<3>(3 * i) = p_WAq - p_WBq;
      psi->segment<3>(3 * i) = (J_WAq - J_WBq) * x.tail(plant.num_velocities());
      alpha->segment<3>(3 * i) =
          (J_WAq - J_WBq) * xdot.tail(plant.num_velocities()) + Jdotv_WAq -
          Jdotv_WBq;
    }
  }

  // Note: This depends on member variables updated in CalcDynamics.
  void AddInVelocityCorrection(const Context<AutoDiffXd>& context,
                               const Eigen::Ref<const AutoDiffVecXd>& vbar,
                               AutoDiffVecXd* g) const {
    const MultibodyPlant<AutoDiffXd>& plant = robot_diagram_->plant();
    const Context<AutoDiffXd>& plant_context =
        robot_diagram_->GetSubsystemContext(plant, context);

    int spatial_force_index = 0;
    Matrix3X<AutoDiffXd> J_WAq(3, plant.num_positions()),
        J_WBq(3, plant.num_positions());
    for (int i = 0; i < num_contacts_; ++i) {
      const Frame<AutoDiffXd>& A =
          plant.get_body(spatial_force_[spatial_force_index].body_index)
              .body_frame();
      plant.CalcJacobianTranslationalVelocity(
          plant_context, JacobianWrtVariable::kQDot, A,
          spatial_force_[spatial_force_index].p_BoBq_B, plant.world_frame(),
          plant.world_frame(), &J_WAq);
      ++spatial_force_index;
      const Frame<AutoDiffXd>& B =
          plant.get_body(spatial_force_[spatial_force_index].body_index)
              .body_frame();
      plant.CalcJacobianTranslationalVelocity(
          plant_context, JacobianWrtVariable::kQDot, B,
          spatial_force_[spatial_force_index].p_BoBq_B, plant.world_frame(),
          plant.world_frame(), &J_WBq);
      ++spatial_force_index;
      g->head(plant.num_positions()) +=
          (J_WAq - J_WBq).transpose() * vbar.segment<3>(3 * i);
    }
  }

  const RobotDiagram<AutoDiffXd>* robot_diagram_{};
  const ContactPairs& in_contact_;
  Context<AutoDiffXd>* context_sample_{};
  Context<AutoDiffXd>* context_next_sample_{};
  Context<AutoDiffXd>* context_collocation_{};
  bool add_next_sample_kinematics_{};

  int num_inputs_;
  int num_contacts_;
  mutable std::vector<multibody::ExternallyAppliedSpatialForce<AutoDiffXd>>
      spatial_force_;
};

class ImpactConstraint : public solvers::Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ImpactConstraint)

  ImpactConstraint(const RobotDiagram<AutoDiffXd>* robot_diagram,
                   const ContactPair& new_contact,
                   double coefficient_of_restitution,
                   Context<AutoDiffXd>* post_context)
      : Constraint(robot_diagram->plant().num_velocities() + 3,
                   robot_diagram->plant().num_positions() +
                       2 * robot_diagram->plant().num_velocities() + 3),
        robot_diagram_(robot_diagram),
        contact_(new_contact),
        coefficient_of_restitution_(coefficient_of_restitution),
        context_(post_context) {
    UpdateLowerBound(VectorXd::Zero(num_constraints()));
    UpdateUpperBound(VectorXd::Zero(num_constraints()));
  }

 private:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override {
    AutoDiffVecXd y_t;
    Eval(x.cast<AutoDiffXd>(), &y_t);
    *y = math::ExtractValue(y_t);
  }

  void DoEval(const Eigen::Ref<const AutoDiffVecXd>& x,
              AutoDiffVecXd* y) const override {
    const MultibodyPlant<AutoDiffXd>& plant = robot_diagram_->plant();
    DRAKE_DEMAND(x.size() == num_vars());
    y->resize(num_constraints());

    // Extract our input variables:
    // q+, v+, v-, Lambda
    int index = 0;
    const auto xplus = x.segment(index, plant.num_multibody_states());
    index += plant.num_positions();  // only increment by num_positions.
    const auto vplus = x.segment(index, plant.num_velocities());
    index += plant.num_velocities();
    const auto vminus = x.segment(index, plant.num_velocities());
    index += plant.num_velocities();
    const auto Lambda = x.segment<3>(index);
    DRAKE_ASSERT(index + 3 == num_vars());

    // Implements the derivation from
    // https://underactuated.csail.mit.edu/multibody.html#impulsive_collision
    Context<AutoDiffXd>* plant_context =
        &robot_diagram_->GetMutableSubsystemContext(plant, context_);

    if (!AreAutoDiffVecXdEqual(
            xplus, plant.GetPositionsAndVelocities(*plant_context))) {
      plant.SetPositionsAndVelocities(plant_context, xplus);
    }

    const auto& query_object =
        plant.get_geometry_query_input_port()
            .template Eval<geometry::QueryObject<AutoDiffXd>>(*plant_context);
    auto& inspector = query_object.inspector();

    geometry::SignedDistancePair<AutoDiffXd> distance_pair =
        query_object.ComputeSignedDistancePairClosestPoints(contact_.first(),
                                                            contact_.second());

    Vector3<AutoDiffXd> p_ACa = inspector.GetPoseInFrame(distance_pair.id_A)
                                    .template cast<AutoDiffXd>() *
                                distance_pair.p_ACa;
    Vector3<AutoDiffXd> p_BCb = inspector.GetPoseInFrame(distance_pair.id_B)
                                    .template cast<AutoDiffXd>() *
                                distance_pair.p_BCb;

    const Frame<AutoDiffXd>& A =
        plant.GetBodyFromFrameId(inspector.GetFrameId(contact_.first()))
            ->body_frame();
    const Frame<AutoDiffXd>& B =
        plant.GetBodyFromFrameId(inspector.GetFrameId(contact_.second()))
            ->body_frame();
    Matrix3X<AutoDiffXd> J_WCa(3, plant.num_velocities()),
        J_WCb(3, plant.num_velocities());
    plant.CalcJacobianTranslationalVelocity(
        *plant_context, JacobianWrtVariable::kV, A, p_ACa, plant.world_frame(),
        plant.world_frame(), &J_WCa);
    plant.CalcJacobianTranslationalVelocity(
        *plant_context, JacobianWrtVariable::kV, B, p_BCb, plant.world_frame(),
        plant.world_frame(), &J_WCb);
    Matrix3X<AutoDiffXd> J = J_WCb - J_WCa;

    MatrixX<AutoDiffXd> M(plant.num_velocities(), plant.num_velocities());
    plant.CalcMassMatrixViaInverseDynamics(*plant_context, &M);

    y->head(plant.num_velocities()) =
        M * (vplus - vminus) - J.transpose() * Lambda;
    y->tail<3>() =
        J * vplus -
        Vector3<AutoDiffXd>(-coefficient_of_restitution_ * J.row(0).dot(vminus),
                            0, 0);
  }

  void DoEval(const Eigen::Ref<const VectorX<symbolic::Variable>>&,
              VectorX<symbolic::Expression>*) const override {
    throw std::logic_error(
        "ConstrainedDirectCollocationConstraint does not support symbolic "
        "evaluation.");
  }

  const RobotDiagram<AutoDiffXd>* robot_diagram_{};
  const ContactPair contact_;
  double coefficient_of_restitution_{};
  Context<AutoDiffXd>* context_{};
};

}  // namespace internal

HybridMultibodyCollocation::ConstrainedDirectCollocation::
    ConstrainedDirectCollocation(
        const HybridMultibodyCollocation& hybrid, std::string name,
        const ContactPairs& sticking_contact,
        const RobotDiagram<AutoDiffXd>* robot_diagram,
        const systems::Context<AutoDiffXd>& robot_diagram_context,
        int num_time_samples, double minimum_time_step,
        double maximum_time_step, solvers::MathematicalProgram* prog)
    : MultipleShooting(hybrid.num_inputs(), hybrid.num_states(),
                       num_time_samples, minimum_time_step, maximum_time_step,
                       prog),
      name_(name),
      robot_diagram_(robot_diagram),
      context_w_collisions_(robot_diagram_context.Clone()),
      context_(robot_diagram_context.Clone()),
      sample_contexts_(num_time_samples) {
  // Disable all collisions in context_.
  auto& inspector = robot_diagram_->scene_graph().model_inspector();
  Context<AutoDiffXd>& scene_graph_context =
      robot_diagram_->GetMutableSubsystemContext(robot_diagram_->scene_graph(),
                                                 context_.get());
  auto filter_manager = robot_diagram_->scene_graph().collision_filter_manager(
      &scene_graph_context);
  geometry::GeometrySet geometry_set(inspector.GetAllGeometryIds());
  filter_manager.Apply(
      geometry::CollisionFilterDeclaration().ExcludeWithin(geometry_set));

  // Allocated contexts for each sample time. We share contexts across multiple
  // constraints in order to exploit caching (the dynamics at time k are
  // evaluated both in constraint k and k+1). Note that the constraints cannot
  // be evaluated in parallel.
  for (int i = 0; i < N(); ++i) {
    sample_contexts_[i] = context_->Clone();
  }

  in_contact_ = sticking_contact;
  int num_contacts = in_contact_.size();

  // TODO(russt): Add support for sliding friction.
  placeholder_force_vars_ =
      this->NewSequentialVariable(3 * num_contacts, "force");

  MatrixXDecisionVariable force_correction = prog->NewContinuousVariables(
      3 * num_contacts, N() - 1, "force_correction");
  MatrixXDecisionVariable velocity_correction = prog->NewContinuousVariables(
      3 * num_contacts, N() - 1, "velocity_correction");

  // Add the dynamic constraints.
  // For N-1 time steps, add a constraint which depends on the breakpoint
  // along with the state and input vectors at that breakpoint and the
  // next.
  for (int i = 0; i < N() - 1; ++i) {
    auto constraint =
        std::make_shared<internal::ConstrainedDirectCollocationConstraint>(
            robot_diagram_, in_contact_, sample_contexts_[i].get(),
            sample_contexts_[i + 1].get(), context_.get(), i == N() - 2);
    this->prog()
        .AddConstraint(constraint,
                       {h_vars().segment<1>(i),
                        x_vars().segment(i * num_states(), num_states() * 2),
                        u_vars().segment(i * num_inputs(), num_inputs() * 2),
                        AllContactForces(i), AllContactForces(i + 1),
                        force_correction.col(i), velocity_correction.col(i)})
        .evaluator()
        ->set_description(
            fmt::format("{} collocation constraint for segment {}", name_, i));
  }

  const MultibodyPlant<AutoDiffXd>& plant = robot_diagram->plant();
  const int num_positions = plant.num_positions();
  int force_index = 0;
  for (const ContactPair& p : in_contact_) {
    for (int i = 0; i < N() - 1; ++i) {
      // TODO(russt): use the sample_contexts_ for the friction constraints.
      Context<AutoDiffXd>* plant_context =
          &robot_diagram->GetMutableSubsystemContext(
              plant, context_w_collisions_.get());
      // Friction constraints.
      // We need to maintain ownership of the wrench evaluators.
      auto& contact_wrench_evaluator = contact_wrench_evaluators_.emplace_back(
          std::make_unique<
              multibody::ContactWrenchFromForceInWorldFrameEvaluator>(
              &plant, plant_context, p));
      prog->AddConstraint(
              std::make_shared<multibody::StaticFrictionConeConstraint>(
                  contact_wrench_evaluator.get()),
              {x_vars().segment(i * num_states(), num_positions),
               AllContactForces(i).segment<3>(3 * force_index)})
          .evaluator()
          ->set_description(fmt::format(
              "{} static friction cone between geometries {}-{} at index {}",
              name_, p.first(), p.second(), i));
    }
    ++force_index;
  }
  for (const ContactPair& p : hybrid.GetContactPairCandidates()) {
    if (in_contact_.count(p) == 0) {
      // Add distance >= 0 (non-penetration) for geometries that are not
      // in_contact.
      for (int i = 0; i < N() - 1; ++i) {
        Context<AutoDiffXd>* plant_context =
            &robot_diagram->GetMutableSubsystemContext(
                plant, sample_contexts_[i].get());
        this->prog()
            .AddConstraint(std::make_shared<multibody::DistanceConstraint>(
                               &plant, p, plant_context, 0, kInf),
                           x_vars().segment(i * num_states(), num_positions))
            .evaluator()
            ->set_description(fmt::format("{} non-penetration constraint for "
                                          "geometries {}-{} at index {}",
                                          name_, p.first(), p.second(), i));
      }
    }
  }
}

int HybridMultibodyCollocation::ConstrainedDirectCollocation::ContactIndex(
    const ContactPair& contact) const {
  int index = 0;
  bool found = false;
  for (const auto& p : in_contact_) {
    if (p == contact) {
      found = true;
      break;
    }
    index++;
  }
  if (!found) {
    throw std::runtime_error(fmt::format(
        "Contact between geometries ids {} and {} is not active in this mode.",
        contact.first(), contact.second()));
  }
  return index;
}

solvers::VectorXDecisionVariable
HybridMultibodyCollocation::ConstrainedDirectCollocation::ContactForce(
    const ContactPair& contact) const {
  int contact_index = ContactIndex(contact);
  return placeholder_force_vars_.segment<3>(3 * contact_index);
}

solvers::VectorXDecisionVariable
HybridMultibodyCollocation::ConstrainedDirectCollocation::ContactForce(
    const ContactPair& contact, int index) const {
  int contact_index = ContactIndex(contact);
  return GetSequentialVariableAtIndex("force", index)
      .segment<3>(3 * contact_index);
}

solvers::VectorXDecisionVariable
HybridMultibodyCollocation::ConstrainedDirectCollocation::AllContactForces(
    int index) const {
  return GetSequentialVariableAtIndex("force", index);
}

void HybridMultibodyCollocation::ConstrainedDirectCollocation::DoAddRunningCost(
    const symbolic::Expression& g) {
  // Trapezoidal integration:
  //    sum_{i=0...N-2} h_i/2.0 * (g_i + g_{i+1}), or
  // g_0*h_0/2.0 + [sum_{i=1...N-2} g_i*(h_{i-1} + h_i)/2.0] +
  // g_{N-1}*h_{N-2}/2.0.

  prog().AddCost(SubstitutePlaceholderVariables(g * h_vars()(0) / 2, 0));
  for (int i = 1; i <= N() - 2; i++) {
    prog().AddCost(SubstitutePlaceholderVariables(
        g * (h_vars()(i - 1) + h_vars()(i)) / 2, i));
  }
  prog().AddCost(
      SubstitutePlaceholderVariables(g * h_vars()(N() - 2) / 2, N() - 1));
}

PiecewisePolynomial<double> HybridMultibodyCollocation::
    ConstrainedDirectCollocation::ReconstructInputTrajectory(
        const solvers::MathematicalProgramResult& result) const {
  if (robot_diagram_->plant().num_actuated_dofs() == 0) {
    return PiecewisePolynomial<double>();
  }

  VectorXd times = GetSampleTimes(result);
  std::vector<double> times_vec(N());
  std::vector<MatrixXd> inputs(N());

  for (int i = 0; i < N(); i++) {
    times_vec[i] = times(i);
    inputs[i] = result.GetSolution(input(i));
  }
  return PiecewisePolynomial<double>::FirstOrderHold(times_vec, inputs);
}

PiecewisePolynomial<double> HybridMultibodyCollocation::
    ConstrainedDirectCollocation::ReconstructStateTrajectory(
        const solvers::MathematicalProgramResult& result) const {
  VectorXd times = GetSampleTimes(result);
  std::vector<double> times_vec(N());
  std::vector<MatrixXd> states(N());
  std::vector<MatrixXd> derivatives(N());

  // The only implementation of the dynamics currently resides inside the
  // constraint implementation.
  internal::ConstrainedDirectCollocationConstraint constraint(
      robot_diagram_, in_contact_, context_.get(), context_.get(),
      context_.get());

  for (int i = 0; i < N(); i++) {
    times_vec[i] = times(i);
    VectorXd u = result.GetSolution(input(i));
    states[i] = result.GetSolution(state(i));
    VectorXd force = result.GetSolution(AllContactForces(i));
    derivatives[i] = math::ExtractValue(constraint.CalcDynamics(
        states[i].cast<AutoDiffXd>(), u.cast<AutoDiffXd>(),
        force.cast<AutoDiffXd>(), context_.get()));
  }
  return PiecewisePolynomial<double>::CubicHermite(times_vec, states,
                                                   derivatives);
}

PiecewisePolynomial<double> HybridMultibodyCollocation::
    ConstrainedDirectCollocation::ReconstructContactForceTrajectory(
        const solvers::MathematicalProgramResult& result,
        const ContactPair& contact) const {
  VectorXd times = GetSampleTimes(result);
  if (in_contact_.count(contact) == 0) {
    return PiecewisePolynomial<double>::ZeroOrderHold(
        Vector2d(times(0), times.tail<1>()[0]), MatrixXd::Zero(3, 2));
  }

  std::vector<double> times_vec(N());
  std::vector<MatrixXd> force(N());

  for (int i = 0; i < N(); i++) {
    times_vec[i] = times(i);
    force[i] = result.GetSolution(ContactForce(contact, i));
  }
  return PiecewisePolynomial<double>::FirstOrderHold(times_vec, force);
}

HybridMultibodyCollocation::HybridMultibodyCollocation(
    const RobotDiagram<double>* robot_diagram,
    const Context<double>& robot_diagram_context, double minimum_time_step,
    double maximum_time_step)
    : robot_diagram_(robot_diagram),
      robot_diagram_ad_(systems::System<double>::ToAutoDiffXd(*robot_diagram)),
      robot_diagram_context_ad_(robot_diagram_ad_->CreateDefaultContext()),
      minimum_time_step_{minimum_time_step},
      maximum_time_step_(maximum_time_step) {
  if (!robot_diagram_context.has_only_continuous_state()) {
    throw std::logic_error(
        "HybridMultibodyCollocation: The MultibodyPlant in "
        "`robot_diagram` must be in continuous-time mode (time_step=0).");
  }
  robot_diagram_context_ad_->SetTimeStateAndParametersFrom(
      robot_diagram_context);
}

HybridMultibodyCollocation::ContactPairs
HybridMultibodyCollocation::GetContactPairCandidates() const {
  auto collision_candidates = model_inspector().GetCollisionCandidates();
  ContactPairs contact_pairs;
  contact_pairs.insert(collision_candidates.begin(),
                       collision_candidates.end());
  return contact_pairs;
}

HybridMultibodyCollocation::ConstrainedDirectCollocation*
HybridMultibodyCollocation::AddMode(std::string name, int num_time_samples,
                                    const ContactPairs& sticking_contact) {
  ContactPairs in_contact(sticking_contact);

  ContactPairs candidates = GetContactPairCandidates();
  for (const ContactPair& p : in_contact) {
    if (candidates.count(p) == 0) {
      throw std::runtime_error(
          fmt::format("Contact between geometries {} and {} is not in "
                      "the GetContactPairCandidates.",
                      p.first(), p.second()));
    }
  }

  const ConstrainedDirectCollocation* prev{nullptr};
  if (!dircon_.empty()) {
    prev = dircon_.back().get();
    for (const ContactPair& p : in_contact) {
      if (dircon_.back()->in_contact().count(p) == 0) {
        throw std::runtime_error(fmt::format(
            "Contact between geometries {} and {} was not active in the most "
            "recently added mode. The AddMode method can only be used if the "
            "ContactPairs are a strict subset of the ContactPairs from the "
            "previous mode. Use e.g. AddModeWithInelasticImpact to create new "
            "contacts.",
            p.first(), p.second()));
      }
    }
  }

  auto& d = dircon_.emplace_back(std::unique_ptr<ConstrainedDirectCollocation>(
      new ConstrainedDirectCollocation(
          *this, name, sticking_contact, robot_diagram_ad_.get(),
          *robot_diagram_context_ad_, num_time_samples, minimum_time_step_,
          maximum_time_step_, &prog_)));

  if (prev) {
    // Add continuity constraints with the previous mode.
    prog_.AddLinearEqualityConstraint(prev->final_state() ==
                                      d->initial_state());
    if (robot_diagram_->plant().get_actuation_input_port().size() > 0) {
      prog_.AddLinearEqualityConstraint(prev->input(prev->num_samples() - 1) ==
                                        d->input(0));
    }
    // TODO(russt): Consider adding force continuity constraints.
  }

  return d.get();
}

HybridMultibodyCollocation::ConstrainedDirectCollocation*
HybridMultibodyCollocation::AddModeWithInelasticImpact(
    std::string name, int num_time_samples, const ContactPair& new_contact) {
  // TODO(russt): Add support for sliding friction.

  if (dircon_.empty()) {
    throw std::runtime_error(
        "There are no modes defined yet. You must call AddMode at least once "
        "before you can create a mode with an impact transition using "
        "AddModelWithInelasticImpact.");
  }
  const ConstrainedDirectCollocation& prev = *dircon_.back();

  ContactPairs candidates = GetContactPairCandidates();
  if (candidates.count(new_contact) == 0) {
    throw std::runtime_error(
        fmt::format("Contact between geometries {} and {} is not in "
                    "the GetContactPairCandidates.",
                    new_contact.first(), new_contact.second()));
  }

  ContactPairs in_contact = prev.in_contact();
  in_contact.insert(new_contact);

  auto& d = dircon_.emplace_back(std::unique_ptr<ConstrainedDirectCollocation>(
      new ConstrainedDirectCollocation(
          *this, name, in_contact, robot_diagram_ad_.get(),
          *robot_diagram_context_ad_, num_time_samples, minimum_time_step_,
          maximum_time_step_, &prog_)));
  // Add continuity/impact constraints with the previous mode.
  const MultibodyPlant<AutoDiffXd>& plant = robot_diagram_ad_->plant();

  prog_.AddLinearEqualityConstraint(
      prev.final_state().head(plant.num_positions()) ==
      d->initial_state().head(plant.num_positions()));
  // Define a variable for the impulse, expressed in the world frame.
  auto Lambda = prog_.NewContinuousVariables<3>(
      fmt::format("Lambda_impact_from_mode_{}_to_{}", dircon_.size() - 2,
                  dircon_.size() - 1));
  const double coefficient_of_restitution{0.0};
  prog_.AddConstraint(
      std::make_shared<internal::ImpactConstraint>(
          robot_diagram_ad_.get(), new_contact, coefficient_of_restitution,
          d->mutable_sample_context(0)),
      {d->initial_state(), prev.final_state().tail(plant.num_velocities()),
       Lambda});
  // TODO(russt): Use the sample_context.
  Context<AutoDiffXd>* plant_context =
      &robot_diagram_ad_->GetMutableSubsystemContext(
          plant, robot_diagram_context_ad_.get());
  auto& contact_wrench_evaluator = contact_wrench_evaluators_.emplace_back(
      std::make_unique<multibody::ContactWrenchFromForceInWorldFrameEvaluator>(
          &plant, plant_context, new_contact));
  prog_
      .AddConstraint(std::make_shared<multibody::StaticFrictionConeConstraint>(
                         contact_wrench_evaluator.get()),
                     {d->initial_state().head(plant.num_positions()), Lambda})
      .evaluator()
      ->set_description(
          fmt::format("static friction cone for impact from_mode {} to {}",
                      dircon_.size() - 2, dircon_.size() - 1));
  if (plant.get_actuation_input_port().size() > 0) {
    prog_.AddLinearEqualityConstraint(prev.input(prev.num_samples() - 1) ==
                                      d->input(0));
  }
  return d.get();
}

PiecewisePolynomial<double>
HybridMultibodyCollocation::ReconstructInputTrajectory(
    const solvers::MathematicalProgramResult& result) const {
  if (dircon_.empty() || num_inputs() == 0) {
    return PiecewisePolynomial<double>();
  }

  auto iter = dircon_.begin();
  PiecewisePolynomial<double> pp = (*iter)->ReconstructInputTrajectory(result);
  ++iter;

  for (; iter != dircon_.end(); ++iter) {
    PiecewisePolynomial<double> this_pp =
        (*iter)->ReconstructInputTrajectory(result);
    // TODO(russt): It could be better to add constraints for the times in the
    // optimization.
    this_pp.shiftRight(pp.end_time() - this_pp.start_time());
    pp.ConcatenateInTime(this_pp);
  }

  return pp;
}

PiecewisePolynomial<double>
HybridMultibodyCollocation::ReconstructStateTrajectory(
    const solvers::MathematicalProgramResult& result) const {
  if (dircon_.empty()) {
    return PiecewisePolynomial<double>();
  }

  auto iter = dircon_.begin();
  PiecewisePolynomial<double> pp = (*iter)->ReconstructStateTrajectory(result);
  ++iter;

  for (; iter != dircon_.end(); ++iter) {
    PiecewisePolynomial<double> this_pp =
        (*iter)->ReconstructStateTrajectory(result);
    this_pp.shiftRight(pp.end_time() - this_pp.start_time());
    pp.ConcatenateInTime(this_pp);
  }

  return pp;
}

PiecewisePolynomial<double>
HybridMultibodyCollocation::ReconstructContactForceTrajectory(
    const solvers::MathematicalProgramResult& result,
    const ContactPair& contact) const {
  if (dircon_.empty()) {
    return PiecewisePolynomial<double>();
  }

  auto iter = dircon_.begin();
  PiecewisePolynomial<double> pp =
      (*iter)->ReconstructContactForceTrajectory(result, contact);
  ++iter;

  for (; iter != dircon_.end(); ++iter) {
    PiecewisePolynomial<double> this_pp =
        (*iter)->ReconstructContactForceTrajectory(result, contact);
    this_pp.shiftRight(pp.end_time() - this_pp.start_time());
    pp.ConcatenateInTime(this_pp);
  }

  return pp;
}

}  // namespace trajectory_optimization
}  // namespace planning
}  // namespace drake
