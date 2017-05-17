#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/state_machine_system.h"

#include <utility>
#include <vector>

#include "bot_core/robot_state_t.hpp"

#include "drake/common/drake_path.h"
#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/action_primitives/action_primitives_common.h"
#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/pick_and_place_common.h"
#include "drake/examples/kuka_iiwa_arm/dev/monolithic_pick_and_place/synchronous_world_state.h"
#include "drake/examples/kuka_iiwa_arm/dev/pick_and_place/pick_and_place_common.h"

using bot_core::robot_state_t;

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {
/* index of iiwastate */
const int kStateIndex = 0;
}

using manipulation::planner::ConstraintRelaxingIk;
using pick_and_place::PickAndPlaceState;
namespace monolithic_pick_and_place {

struct PickAndPlaceStateMachineSystem::InternalState {
  InternalState() {
    pick_and_place_state = PickAndPlaceState::OPEN_GRIPPER;
    wsg_current_action = GripperActionInput::UNDEFINED;
    iiwa_current_action.is_valid = false;
  }
  ~InternalState() {}

  // This state is used to decide the current state of the
  // finite state machine logic.
  PickAndPlaceState pick_and_place_state;

  // The input to be applied to the IiwaMove ActionPrimitive.
  IiwaActionInput iiwa_current_action;

  // The previous input that was applied to the IiwaMove ActionPrimitive.
  IiwaActionInput previous_action;

  // A logic flag for the validity of the current IiwaActionInput stored
  // within the internal state.
  bool iiwa_action_initiated{false};

  // The input to be applied to the GripperAction ActionPrimitive.
  GripperActionInput wsg_current_action;

  // The previous input that was applied to the GripperAction ActionPrimitive.
  GripperActionInput wsg_previous_action;

  // A logic flag for the validity of the current GripperActionInput stored
  // within the internal state.
  bool wsg_action_initiated{false};

  // Poses used for storing end-points of Iiwa trajectories at various states
  // of the demo.
  Isometry3<double> X_WEndEffector0, X_WEndEffector1;

  // Desired object end pose relative to the base of the iiwa arm.
  Isometry3<double> X_IiwaObj_desired;

  // Desired object end pose in the world frame.
  Isometry3<double> X_WObj_desired;
};

PickAndPlaceStateMachineSystem::PickAndPlaceStateMachineSystem(
    const Isometry3<double>& iiwa_base, const double update_interval)
    : iiwa_base_(iiwa_base),
      planner_(std::make_unique<ConstraintRelaxingIk>(
          drake::GetDrakePath() + kIiwaUrdf, kIiwaEndEffectorName, iiwa_base_)),
      world_state_(
          std::make_unique<SynchronousWorldState>(planner_->get_robot())) {
  input_port_iiwa_state_ = this->DeclareAbstractInputPort().get_index();
  input_port_box_state_ = this->DeclareAbstractInputPort().get_index();
  input_port_wsg_status_ = this->DeclareAbstractInputPort().get_index();
  input_port_wsg_action_status_ = this->DeclareAbstractInputPort().get_index();
  input_port_iiwa_action_status_ = this->DeclareAbstractInputPort().get_index();
  output_port_iiwa_action_ = this->DeclareAbstractOutputPort().get_index();
  output_port_wsg_action_ = this->DeclareAbstractOutputPort().get_index();
  this->DeclarePeriodicUnrestrictedUpdate(update_interval, 0);
}

std::unique_ptr<systems::AbstractValues>
PickAndPlaceStateMachineSystem::AllocateAbstractState() const {
  std::vector<std::unique_ptr<systems::AbstractValue>> abstract_vals;
  abstract_vals.push_back(std::unique_ptr<systems::AbstractValue>(
      new systems::Value<InternalState>(InternalState())));
  return std::make_unique<systems::AbstractValues>(std::move(abstract_vals));
}

std::unique_ptr<systems::AbstractValue>
PickAndPlaceStateMachineSystem::AllocateOutputAbstract(
    const systems::OutputPortDescriptor<double>& descriptor) const {
  std::unique_ptr<systems::AbstractValue> return_val;
  /* allocate iiwa action and wsg output port */
  if (descriptor.get_index() == output_port_iiwa_action_) {
    return_val =
        systems::AbstractValue::Make<IiwaActionInput>(IiwaActionInput());
  } else if (descriptor.get_index() == output_port_wsg_action_) {
    return_val = systems::AbstractValue::Make<GripperActionInput>(
        GripperActionInput::CLOSE);
  }
  return return_val;
}

void PickAndPlaceStateMachineSystem::SetDefaultState(
    const systems::Context<double>&,
    systems::State<double>* state) const {
  InternalState& internal_state =
      state->get_mutable_abstract_state<InternalState>(kStateIndex);

  internal_state.pick_and_place_state = PickAndPlaceState::OPEN_GRIPPER;
  internal_state.iiwa_current_action.is_valid = false;
  internal_state.iiwa_current_action.q.clear();
  internal_state.iiwa_current_action.time.clear();
  internal_state.wsg_current_action = GripperActionInput::UNDEFINED;

  internal_state.X_WEndEffector0 = Isometry3<double>::Identity();
  internal_state.X_WEndEffector1 = Isometry3<double>::Identity();
  internal_state.X_IiwaObj_desired = Isometry3<double>::Identity();
  internal_state.X_WObj_desired = Isometry3<double>::Identity();
}

void PickAndPlaceStateMachineSystem::DoCalcOutput(
    const systems::Context<double>& context,
    systems::SystemOutput<double>* output) const {
  /* Call actions based on state machine logic */

  IiwaActionInput& iiwa_primitive_input =
      output->GetMutableData(output_port_iiwa_action_)
          ->GetMutableValue<IiwaActionInput>();

  GripperActionInput& wsg_primitive_input =
      output->GetMutableData(output_port_wsg_action_)
          ->GetMutableValue<GripperActionInput>();

  const InternalState& internal_state =
      context.get_abstract_state<InternalState>(0);

  iiwa_primitive_input = internal_state.iiwa_current_action;
  wsg_primitive_input = internal_state.wsg_current_action;
}

void PickAndPlaceStateMachineSystem::DoCalcUnrestrictedUpdate(
    const systems::Context<double>& context,
    systems::State<double>* state) const {
  // Extract Internal state.
  InternalState& internal_state =
      state->get_mutable_abstract_state<InternalState>(kStateIndex);

  IiwaActionInput& iiwa_action_input = internal_state.iiwa_current_action;
  GripperActionInput& wsg_action_input = internal_state.wsg_current_action;

  /* Update world state from inputs. */
  const robot_state_t& iiwa_state =
      this->EvalAbstractInput(context, input_port_iiwa_state_)
          ->GetValue<robot_state_t>();
  const robot_state_t& box_state =
      this->EvalAbstractInput(context, input_port_box_state_)
          ->GetValue<robot_state_t>();
  const lcmt_schunk_wsg_status& wsg_status =
      this->EvalAbstractInput(context, input_port_wsg_status_)
          ->GetValue<lcmt_schunk_wsg_status>();
  const ActionPrimitiveState& iiwa_primitive_state =
      this->EvalAbstractInput(context, input_port_iiwa_action_status_)
          ->GetValue<ActionPrimitiveState>();
  const ActionPrimitiveState& wsg_primitive_state =
      this->EvalAbstractInput(context, input_port_wsg_action_status_)
          ->GetValue<ActionPrimitiveState>();

  world_state_->UnpackIiwaStatusMessage(&iiwa_state);
  world_state_->UnpackWsgStatusMessage(&wsg_status);
  world_state_->UnpackObjectStatusMessage(&box_state);

  IKResults ik_res;
  std::vector<double> times;

  // Desired end effector pose in the world frame for pick and place.
  Isometry3<double>& X_WEndEffector0 = internal_state.X_WEndEffector0;
  Isometry3<double>& X_WEndEffector1 = internal_state.X_WEndEffector1;

  // Desired object end pose relative to the base of the iiwa arm.
  Isometry3<double>& X_IiwaObj_desired = internal_state.X_IiwaObj_desired;

  // Desired object end pose in the world frame.
  Isometry3<double>& X_WObj_desired = internal_state.X_IiwaObj_desired;

  // Position the gripper 30cm above the object before grasp.
  const double kPreGraspHeightOffset = 0.3;

  // Position and rotation tolerances.
  // These should be adjusted to a tight bound until IK stops reliably giving
  // results.
  const Vector3<double> kTightPosTol(0.005, 0.005, 0.005);
  const double kTightRotTol = 0.05;

  const Vector3<double> kLoosePosTol(0.05, 0.05, 0.05);
  const double kLooseRotTol = 0.5;

  // Default actions for primitives.
  wsg_action_input = GripperActionInput::UNDEFINED;
  iiwa_action_input.is_valid = false;

  /* state machine logic */
  switch (internal_state.pick_and_place_state) {
    case PickAndPlaceState::OPEN_GRIPPER:
      // changes output to gripper action primitive.
      if (!internal_state.wsg_action_initiated) {
        drake::log()->info("StateMachine : OPEN_GRIPPER at {}",
                           context.get_time());
        // Perform wsg action.
        wsg_action_input = GripperActionInput::OPEN;
        internal_state.wsg_action_initiated = true;
      } else if (wsg_primitive_state == ActionPrimitiveState::WAITING) {
        // This means that the action was already intiated and primitive
        // has returned to WAITING state, i.e. it is complete.
        // resetting wsg_action.
        internal_state.wsg_action_initiated = false;
        internal_state.pick_and_place_state =
            PickAndPlaceState::APPROACH_PICK_PREGRASP;
      }
      break;
    case PickAndPlaceState::APPROACH_PICK_PREGRASP:

      if (!internal_state.iiwa_action_initiated) {
        // Computes the desired end effector pose in the world frame to be
        // kPreGraspHeightOffset above the object.
        drake::log()->info("StateMachine : APPROACH_PICK_PREGRASP at {}",
                           context.get_time());
        X_WEndEffector0 = world_state_->get_iiwa_end_effector_pose();
        X_WEndEffector1 =
            pick_and_place::ComputeGraspPose(world_state_->get_object_pose());
        X_WEndEffector1.translation()[2] += kPreGraspHeightOffset;

        // 2 seconds, no via points.
        bool res = pick_and_place::PlanStraightLineMotion(
            world_state_->get_iiwa_q(), 0, 2, X_WEndEffector0, X_WEndEffector1,
            kLoosePosTol, kLooseRotTol, planner_.get(), &ik_res, &times);
        DRAKE_DEMAND(res);

        iiwa_action_input.is_valid = true;
        iiwa_action_input.time = times;
        iiwa_action_input.q = ik_res.q_sol;
        internal_state.iiwa_action_initiated = true;

      } else if (iiwa_primitive_state == ActionPrimitiveState::WAITING) {
        // This means that the action was already intiated and primitive
        // has returned to WAITING state, i.e. it is complete.
        // resetting iiwa_action.

        internal_state.iiwa_action_initiated = false;
        internal_state.pick_and_place_state = PickAndPlaceState::APPROACH_PICK;
      }
      break;
    case PickAndPlaceState::APPROACH_PICK:
      if (!internal_state.iiwa_action_initiated) {
        drake::log()->info("StateMachine : APPROACH_PICK at {}",
                           context.get_time());
        X_WEndEffector0 = X_WEndEffector1;
        X_WEndEffector1 =
            pick_and_place::ComputeGraspPose(world_state_->get_object_pose());

        // 1 second, 3 via points. More via points to ensure the end effector
        // moves in more or less a straight line.
        bool res = pick_and_place::PlanStraightLineMotion(
            world_state_->get_iiwa_q(), 3, 1, X_WEndEffector0, X_WEndEffector1,
            kTightPosTol, kTightRotTol, planner_.get(), &ik_res, &times);
        DRAKE_DEMAND(res);

        internal_state.iiwa_action_initiated = true;
        iiwa_action_input.is_valid = true;
        iiwa_action_input.time = times;
        iiwa_action_input.q = ik_res.q_sol;

      } else if (iiwa_primitive_state == ActionPrimitiveState::WAITING) {
        // This means that the action was already intiated and primitive
        // has returned to WAITING state, i.e. it is complete.
        // resetting iiwa_action.

        internal_state.iiwa_action_initiated = false;
        internal_state.pick_and_place_state = PickAndPlaceState::GRASP;
      }
      break;
    case PickAndPlaceState::GRASP:
      // change output to gripper action primitive.
      if (!internal_state.wsg_action_initiated) {
        // Perform wsg action.
        drake::log()->info("StateMachine : GRASP at {}", context.get_time());
        wsg_action_input = GripperActionInput::CLOSE;
        internal_state.wsg_action_initiated = true;
      } else if (wsg_primitive_state == ActionPrimitiveState::WAITING) {
        // This means that the action was already intiated and primitive
        // has returned to WAITING state, i.e. it is complete.
        // resetting wsg_action.
        internal_state.wsg_action_initiated = false;
        internal_state.pick_and_place_state = PickAndPlaceState::LIFT_FROM_PICK;
      }
      break;
    case PickAndPlaceState::LIFT_FROM_PICK:

      if (!internal_state.iiwa_action_initiated) {
        drake::log()->info("StateMachine : LIFT_FROM_PICK at {}",
                           context.get_time());
        X_WEndEffector0 = X_WEndEffector1;
        X_WEndEffector1.translation()[2] += kPreGraspHeightOffset;

        // 1 seconds, 3 via points.
        bool res = pick_and_place::PlanStraightLineMotion(
            world_state_->get_iiwa_q(), 3, 1, X_WEndEffector0, X_WEndEffector1,
            kTightPosTol, kTightRotTol, planner_.get(), &ik_res, &times);
        DRAKE_DEMAND(res);

        internal_state.iiwa_action_initiated = true;
        iiwa_action_input.is_valid = true;
        iiwa_action_input.time = times;
        iiwa_action_input.q = ik_res.q_sol;

      } else if (iiwa_primitive_state == ActionPrimitiveState::WAITING) {
        // This means that the action was already initiated and primitive
        // has returned to WAITING state, i.e. it is complete.
        // resetting iiwa_action.

        internal_state.iiwa_action_initiated = false;
        internal_state.pick_and_place_state =
            PickAndPlaceState::APPROACH_PLACE_PREGRASP;
      }
      break;
    case PickAndPlaceState::APPROACH_PLACE_PREGRASP:

      if (!internal_state.iiwa_action_initiated) {
        drake::log()->info("StateMachine : APPROACH_PLACE_PREGRASP at {}",
                           context.get_time());
        int table = pick_and_place::get_table(world_state_->get_object_pose(),
                                              iiwa_base_);

        // Sets desired place location based on where we picked up the object.
        // Table 0 is in front of iiwa base, and table 1 is to the left.
        if (table == 0) {
          // Table 0 -> table 1.
          X_IiwaObj_desired.translation() = pick_and_place::kPlacePosition1;
          X_IiwaObj_desired.linear() = Matrix3<double>(
              AngleAxis<double>(M_PI / 2., Vector3<double>::UnitZ()));
        } else {
          // Table 1 -> table 0.
          X_IiwaObj_desired.translation() = pick_and_place::kPlacePosition0;
          X_IiwaObj_desired.linear().setIdentity();
        }
        X_WObj_desired = iiwa_base_ * X_IiwaObj_desired;

        // Recomputes gripper's pose relative the object since the object
        // probably moved during transfer.
        const Isometry3<double> X_ObjEndEffector =
            world_state_->get_object_pose().inverse() *
            world_state_->get_iiwa_end_effector_pose();

        X_WEndEffector0 = X_WEndEffector1;
        X_WEndEffector1 = X_WObj_desired * X_ObjEndEffector;
        X_WEndEffector1.translation()[2] += kPreGraspHeightOffset;

        // 2 seconds, 2 via points. This doesn't have to be a move straight
        // primitive. I did it this way because I have seen the IK gives a
        // wild motion that causes the gripper to lose the object.
        bool res = pick_and_place::PlanStraightLineMotion(
            world_state_->get_iiwa_q(), 2, 2, X_WEndEffector0, X_WEndEffector1,
            kLoosePosTol, kLooseRotTol, planner_.get(), &ik_res, &times);

        DRAKE_DEMAND(res);

        internal_state.iiwa_action_initiated = true;
        iiwa_action_input.is_valid = true;
        iiwa_action_input.time = times;
        iiwa_action_input.q = ik_res.q_sol;
      } else if (iiwa_primitive_state == ActionPrimitiveState::WAITING) {
        // This means that the action was already intiated and primitive
        // has returned to WAITING state, i.e. it is complete.
        // resetting iiwa_action.

        internal_state.iiwa_action_initiated = false;
        internal_state.pick_and_place_state = PickAndPlaceState::APPROACH_PLACE;
      }
      break;
    case PickAndPlaceState::APPROACH_PLACE:
      if (!internal_state.iiwa_action_initiated) {
        drake::log()->info("StateMachine : APPROACH_PLACE at {}",
                           context.get_time());

        // Recomputes gripper's pose relative the object since the object
        // probably moved during transfer.
        const Isometry3<double> X_ObjEndEffector =
            world_state_->get_object_pose().inverse() *
            world_state_->get_iiwa_end_effector_pose();

        // Computes the desired end effector pose in the world frame.
        X_WEndEffector0 = X_WEndEffector1;
        X_WEndEffector1 = X_WObj_desired * X_ObjEndEffector;
        // TODO(siyuan): This hack is to prevent the robot from forcefully
        // pushing the object into the table. Shouldn't be necessary once
        // we have guarded moves supported by the controller side.
        X_WEndEffector1.translation()[2] += 0.02;

        // 1 seconds, 3 via points.
        bool res = pick_and_place::PlanStraightLineMotion(
            world_state_->get_iiwa_q(), 3, 1, X_WEndEffector0, X_WEndEffector1,
            kTightPosTol, kTightRotTol, planner_.get(), &ik_res, &times);
        DRAKE_DEMAND(res);

        internal_state.iiwa_action_initiated = true;
        iiwa_action_input.is_valid = true;
        iiwa_action_input.time = times;
        iiwa_action_input.q = ik_res.q_sol;
      } else if (iiwa_primitive_state == ActionPrimitiveState::WAITING) {
        // This means that the action was already intiated and primitive
        // has returned to WAITING state, i.e. it is complete.
        // resetting iiwa_action.

        internal_state.iiwa_action_initiated = false;
        internal_state.pick_and_place_state = PickAndPlaceState::PLACE;
      }
      break;
    case PickAndPlaceState::PLACE:
      if (!internal_state.wsg_action_initiated) {
        drake::log()->info("StateMachine : PLACE at {}", context.get_time());
        // Perform wsg action.
        wsg_action_input = GripperActionInput::OPEN;
        internal_state.wsg_action_initiated = true;
      } else if (wsg_primitive_state == ActionPrimitiveState::WAITING) {
        // This means that the action was already intiated and primitive
        // has returned to WAITING state, i.e. it is complete.
        // resetting wsg_action.
        internal_state.wsg_action_initiated = false;
        internal_state.pick_and_place_state =
            PickAndPlaceState::LIFT_FROM_PLACE;
      }

      break;
    case PickAndPlaceState::LIFT_FROM_PLACE:
      if (!internal_state.iiwa_action_initiated) {
        drake::log()->info("StateMachine : LIFT_FROM_PLACE at {}",
                           context.get_time());

        X_WEndEffector0 = X_WEndEffector1;
        X_WEndEffector1.translation()[2] += kPreGraspHeightOffset;

        // 1 seconds, 3 via points.
        bool res = pick_and_place::PlanStraightLineMotion(
            world_state_->get_iiwa_q(), 3, 1, X_WEndEffector0, X_WEndEffector1,
            kTightPosTol, kTightRotTol, planner_.get(), &ik_res, &times);
        DRAKE_DEMAND(res);

        internal_state.iiwa_action_initiated = true;
        iiwa_action_input.is_valid = true;
        iiwa_action_input.time = times;
        iiwa_action_input.q = ik_res.q_sol;
      } else if (iiwa_primitive_state == ActionPrimitiveState::WAITING) {
        // This means that the action was already intiated and primitive
        // has returned to WAITING state, i.e. it is complete.
        // resetting iiwa_action.

        internal_state.iiwa_action_initiated = false;
        internal_state.pick_and_place_state = PickAndPlaceState::DONE;
      }
      break;
    case PickAndPlaceState::DONE:
      if (!internal_state.iiwa_action_initiated) {
        drake::log()->info("StateMachine : DONE at {}", context.get_time());
        const std::vector<double> time = {0, 2};
        std::vector<VectorX<double>> q(2, world_state_->get_iiwa_q());
        q[1].setZero();

        internal_state.iiwa_action_initiated = true;
        iiwa_action_input.is_valid = true;
        iiwa_action_input.time = times;
        iiwa_action_input.q = ik_res.q_sol;
      } else if (iiwa_primitive_state == ActionPrimitiveState::WAITING) {
        // This means that the action was already intiated and primitive
        // has returned to WAITING state, i.e. it is complete.
        // resetting iiwa_action.

        internal_state.iiwa_action_initiated = false;
        internal_state.pick_and_place_state = PickAndPlaceState::OPEN_GRIPPER;
      }
      break;
  }
}

}  // namespace monolithic_pick_and_place
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
