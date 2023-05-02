#pragma once

#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/sorted_pair.h"
#include "drake/multibody/optimization/contact_wrench_evaluator.h"
#include "drake/planning/robot_diagram.h"
#include "drake/planning/trajectory_optimization/direct_collocation.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace planning {
namespace trajectory_optimization {

/* TODO(russt): Add support for
 - Joint limits
 - Sliding friction
 - Elastic impacts
 - UnitQuaternionConstraints (naive application will cause the collocation
 constraints to become infeasible, as described in the DIRCON paper).
*/

/** Implements hybrid trajectory optimization using a MultibodyPlant, where the
 dynamics are implemented in a floating-base coordinates, and contact forces
 are added as explicit decision variables. "Hybrid" here means that
 trajectories are decomposed into continuous modes punctuated by instantaneous
 discrete events when the system makes or breaks contact. Each mode is defined
 by the active set of the SceneGraph collision candidates, the contact model is
 rigid contact (no penetration at optimality), and collisions can be inelastic
 or elastic.

 In this formulation, the mode sequence is specified by the user. In practice,
 this means that the caller must specify e.g. "in the first mode the foot is in
 the air, in the second mode the heel is in contact with the ground, in the
 third mode the heel and toe are in contact with the ground", but the exact
 contact positions and contact timings/mode durations are resolved by the
 optimization.

 Optimization within each mode is formulated using Constrained Direct
 Collocation (DIRCON), as described in

 Michael Posa, Scott Kuindersma, Russ Tedrake. "Optimization and Stabilization
 of Trajectories for Constrained Dynamical Systems." ICRA, 2016.

 For example, to generate an optimization that starts in an "aerial phase"
 (with no contacts), and then transitions to a "ground phase" with the first
 contact active, one could write e.g.
 @code
  HybridMultibodyCollocation hybrid(diagram, context, kMinTimeStep,
                                    kMaxTimeStep);
  auto* aerial_phase = hybrid.AddMode("aerial", kNumTimeSteps, {});
  ContactPair contact(robot_heel_geometry_id, ground_geometry_id);
  auto* heel_phase = hybrid.AddModeWithInelasticImpact(
    "heel", kNumTimeSteps, contact);
  ...
 @endcode

 See https://underactuated.csail.mit.edu/contact.html for additional  details.

 @ingroup planning_trajectory
*/
class HybridMultibodyCollocation {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(HybridMultibodyCollocation)

  using ContactPair = SortedPair<geometry::GeometryId>;
  using ContactPairs = std::set<ContactPair>;

  /** Implements a specialized version of Constrained Direct Collocation
   (DIRCON) with contact forces as additional sequential decision variables. */
  class ConstrainedDirectCollocation : public MultipleShooting {
   public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ConstrainedDirectCollocation)

    ~ConstrainedDirectCollocation() override {}

    /** Returns the set of all active contacts. */
    const ContactPairs& in_contact() const { return in_contact_; }

    /** Returns a mutable pointer to the Context used to evaluate the
    constraints at the time-index @p index. */
    systems::Context<AutoDiffXd>* mutable_sample_context(int index) const {
      return sample_contexts_[index].get();
    }

    /** Returns placeholder decision variables (not actually declared as
    decision variables in the MathematicalProgram) associated with the contact
    force enforcing @p contact, but with the time-index undetermined.  These
    variables will be substituted for real decision variables at particular
    times in methods like AddRunningCost.  Passing these variables directly
    into objectives/constraints for the parent classes will result in an error.

    @throws std::exception if @p contact is not in_contact(). */
    solvers::VectorXDecisionVariable ContactForce(
        const ContactPair& contact) const;

    /** Returns decision variables associated with the contact
    force enforcing @p contact at time-index @p index.

    @throws std::exception if @p contact is not in_contact(). */
    solvers::VectorXDecisionVariable ContactForce(const ContactPair& contact,
                                                  int index) const;

    /** Returns placeholder decision variables (not actually declared as
    decision variables in the MathematicalProgram) associated with the contact
    forces for all contacts.  These variables will be substituted for real
    decision variables at particular times in methods like AddRunningCost.
    Passing these variables directly into objectives/constraints for the parent
    classes will result in an error. */
    solvers::VectorXDecisionVariable AllContactForces() const {
      return placeholder_force_vars_;
    }

    /** Returns decision variables associated with the contact
    forces for all contacts at time-index @p index. */
    solvers::VectorXDecisionVariable AllContactForces(int index) const;

    /** Returns the input trajectory, u(t), for this mode. */
    trajectories::PiecewisePolynomial<double> ReconstructInputTrajectory(
        const solvers::MathematicalProgramResult& result) const override;

    /** Returns the state trajectory, x(t), for this mode. */
    trajectories::PiecewisePolynomial<double> ReconstructStateTrajectory(
        const solvers::MathematicalProgramResult& result) const override;

    /** Returns the force trajectory, lambda(t), for @p contact in this mode. */
    trajectories::PiecewisePolynomial<double> ReconstructContactForceTrajectory(
        const solvers::MathematicalProgramResult& result,
        const ContactPair& contact) const;

   private:
    friend class HybridMultibodyCollocation;

    ConstrainedDirectCollocation(
        const HybridMultibodyCollocation& hybrid, std::string name,
        const ContactPairs& sticking_contact,
        const RobotDiagram<AutoDiffXd>* robot_diagram,
        const systems::Context<AutoDiffXd>& robot_diagram_context,
        int num_time_samples, double minimum_time_step,
        double maximum_time_step, solvers::MathematicalProgram* prog);

    int ContactIndex(const ContactPair& contact) const;

    // Implements a running cost at all time steps using trapezoidal
    // integration.
    void DoAddRunningCost(const symbolic::Expression& e) override;

    std::string name_{};
    const RobotDiagram<AutoDiffXd>* robot_diagram_;
    std::unique_ptr<systems::Context<AutoDiffXd>> context_w_collisions_;
    std::unique_ptr<systems::Context<AutoDiffXd>> context_;
    std::vector<std::unique_ptr<systems::Context<AutoDiffXd>>> sample_contexts_;
    ContactPairs in_contact_;
    solvers::VectorXDecisionVariable placeholder_force_vars_;

    std::vector<
        std::unique_ptr<multibody::ContactWrenchFromForceInWorldFrameEvaluator>>
        contact_wrench_evaluators_;
  };

  /** Constructs the empty HybridMultibodyCollocation problem.
  @param robot_diagram contains the plant and scene_graph defining a
  MultibodyPlant with contact geometry.
  @param robot_diagram_context is a Context for robot_diagram which potentially
  contains parameters that define the dynamics.
  @param minimum_time_step is the minimum_time_step used in all modes.
  @param maximum_time_step is the maximum_time_step used in all modes.
   */
  HybridMultibodyCollocation(
      const RobotDiagram<double>* robot_diagram,
      const systems::Context<double>& robot_diagram_context,
      double minimum_time_step, double maximum_time_step);

  ~HybridMultibodyCollocation() = default;

  /** Returns a reference to the MathematicalProgram associated with the
   trajectory optimization problem. */
  solvers::MathematicalProgram& prog() { return prog_; }

  /** Convenience method to return the number of actuator inputs for the
   MultibodyPlant. */
  int num_inputs() const {
    return robot_diagram_->plant().get_actuation_input_port().size();
  }

  /** Convenience method to return the number of multibody states in the
   MultibodyPlant. */
  int num_states() const {
    return robot_diagram_->plant().num_multibody_states();
  }

  /** Returns the list of all possible contact pairs registered with the
   SceneGraph. Modes are defined by having a subset of these pairs active.
   Pairs that are out of contact still add constraints (to ensure that they are
   not in contact) to each mode. Using collision filter groups in the geometry
   engine can shorten this list and potentially dramatically simplify the
   optimization. */
  ContactPairs GetContactPairCandidates() const;

  /** Convenience method to return the SceneGraph model inspector, which can be
   used to understand and work with the geometry::GeometryId in a ContactPair.
   */
  const geometry::SceneGraphInspector<double>& model_inspector() const {
    return robot_diagram_->scene_graph().model_inspector();
  }

  /** Adds a new ConstrainedDirectCollocation program with @p num_time_samples
   corresponding to a hybrid mode. For the first mode added to this program, @p
   sticking_contact can be any subset of GetContactPairCandidates(). For
   subsequent modes, @p in_contact must be only a subset of the previous mode's
   in_contact() (e.g. bodies must only come *out* of collision). In order to
   add a mode where bodies come *into* collision, see
   AddModeWithInelasticImpact(). */
  ConstrainedDirectCollocation* AddMode(std::string name, int num_time_samples,
                                        const ContactPairs& sticking_contact);

  // TODO(russt): Support multiple simultaneous making/breaking of contact.
  /** Adds a new ConstrainedDirectCollocation program with @p num_time_samples
   corresponding to an additional hybrid mode. Additional constraints are added
   to enforce that the final_state() of the most recently added mode are equal
   to the initial_state() of the new mode after applying the inelastic contact
   dynamics. */
  ConstrainedDirectCollocation* AddModeWithInelasticImpact(
      std::string name, int num_time_samples, const ContactPair& new_contact);

  /** After solving the optimization, this method extracts the input trajectory
   from the result. */
  trajectories::PiecewisePolynomial<double> ReconstructInputTrajectory(
      const solvers::MathematicalProgramResult& result) const;

  /** After solving the optimization, this method extracts the state trajectory
   from the result. */
  trajectories::PiecewisePolynomial<double> ReconstructStateTrajectory(
      const solvers::MathematicalProgramResult& result) const;

  /** After solving the optimization, this method extracts the force trajectory
   associated with @p contact from the result. */
  trajectories::PiecewisePolynomial<double> ReconstructContactForceTrajectory(
      const solvers::MathematicalProgramResult& result,
      const ContactPair& contact) const;

 private:
  const RobotDiagram<double>* robot_diagram_{};
  const std::unique_ptr<RobotDiagram<AutoDiffXd>> robot_diagram_ad_{};
  const std::unique_ptr<systems::Context<AutoDiffXd>>
      robot_diagram_context_ad_{};
  double minimum_time_step_{};
  double maximum_time_step_{};
  solvers::MathematicalProgram prog_{};
  std::vector<std::unique_ptr<ConstrainedDirectCollocation>> dircon_{};
  std::vector<
      std::unique_ptr<multibody::ContactWrenchFromForceInWorldFrameEvaluator>>
      contact_wrench_evaluators_;
};

}  // namespace trajectory_optimization
}  // namespace planning
}  // namespace drake
