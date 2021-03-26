#include "sim/common/collision_remover.h"

#include <algorithm>
#include <functional>
#include <ostream>
#include <string>
#include <vector>

#include "drake/common/constants.h"
#include "drake/common/sorted_pair.h"
#include "drake/common/symbolic_variable.h"
#include "drake/geometry/query_object.h"
#include "drake/geometry/scene_graph.h"
#include "drake/math/saturate.h"
#include "drake/multibody/inverse_kinematics/inverse_kinematics.h"
#include "drake/multibody/inverse_kinematics/minimum_distance_constraint.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solve.h"
#include "common/mathematical_program_to_string.h"

using Eigen::VectorXd;
using drake::geometry::SceneGraph;
using drake::solvers::MathematicalProgram;
using drake::systems::Context;
using drake::systems::Diagram;
using drake::multibody::Body;
using drake::multibody::BodyIndex;
using drake::multibody::Joint;
using drake::multibody::JointIndex;
using drake::multibody::MultibodyPlant;

namespace anzu {
namespace sim {

namespace geom = drake::geometry;

namespace {
// How many iterations of bisect to try.  Setting this high doesn't help much.
constexpr static int kBisectIterations = 10;

// How far back from collision the IK MinimumDistanceConstraint should aim;
// setting this too low is bad due to numeric issues in the constraint function.
constexpr static double kIkMinDistance = 1e-2;

enum class IkMode {
  kInsideOut = 1,
  kOutsideIn = 2,
};

std::string PositionAsString(const MultibodyPlant<double>& plant,
                             VectorXd pos) {
  std::ostringstream output;
  for (int i = 0; i < pos.size(); ++i) {
    bool found = false;
    for (drake::multibody::ModelInstanceIndex m(0);
         m < plant.num_model_instances(); ++m) {
      for (JointIndex j : plant.GetJointIndices(m)) {
        const Joint<double>& joint = plant.get_joint(j);
        int index_in_joint = i - joint.position_start();
        if (index_in_joint >= 0 && index_in_joint < joint.num_positions()) {
          output << joint.name() << "[" << index_in_joint << "] == "
                 << pos[i] << std::endl;
          found = true;
          break;
        }
      }
      if (found) { break; }
    }
    if (found) { continue; }
    for (BodyIndex b : plant.GetFloatingBaseBodies()) {
      const Body<double>& body = plant.get_body(b);
      if (body.is_floating()) {
        // Per https://github.com/RobotLocomotion/drake/issues/12807 we don't
        // really know the number of positions for a free body without knowing
        // details of its underlying mobilizer.  If we have quaternion DoFs
        // then we guess that it's 7; otherwise we make no assumption.
        if (!body.has_quaternion_dofs()) {
          continue;
        }
        int index_in_body = i - body.floating_positions_start();
        if (index_in_body >= 0 && index_in_body < 7) {
          output << body.name() << "[" << index_in_body << "] == "
                 << pos[i] << std::endl;
          found = true;
          break;
        }
      }
    }
    if (!found) {
      output << "UNKNOWN q[" << i << "] == " << pos[i] << std::endl;
    }
  }
  return output.str();
}

}  // namespace


class CollisionRemover::Impl {
 public:
  Impl(const Diagram<double>* diagram,
       const MultibodyPlant<double>* plant,
       const SceneGraph<double>* scene_graph)
      : diagram_(diagram),
        plant_(plant),
        scene_graph_(scene_graph) {}

  ~Impl() {}

  bool AdjustPositions(
      Context<double>* root_context,
      const std::set<JointIndex>& joints_to_adjust,
      const std::set<BodyIndex>& floating_bodies_to_adjust,
      const std::optional<const std::set<BodyIndex>>& bodies_to_decollide,
      const VectorXd& known_valid_position) {
    // Preconditions:
    DRAKE_DEMAND(root_context->GetSystemPathname() ==
                 diagram_->GetSystemPathname());
    for (const BodyIndex id : floating_bodies_to_adjust) {
      DRAKE_DEMAND(plant_->get_body(id).is_floating());
    }

    // Short-circuit checks.
    const Context<double>& plant_context =
        diagram_->GetSubsystemContext(*plant_, *root_context);
    if (bodies_to_decollide && bodies_to_decollide->empty()) {
      drake::log()->debug("AdjustPositions called with an empty body set.");
      return true;
    } else if (!Collides(plant_context, bodies_to_decollide)) {
      drake::log()->debug("AdjustPositions but no collisions are present.");
      return true;
    }
    if (joints_to_adjust.empty() && floating_bodies_to_adjust.empty()) {
      return false;  // If we cannot adjust, failure is assured.
    }

    // Dispatch to our strategies.  Try IK first; if it fails then bisect.
    std::vector<std::function<VectorXd(void)>> methods = {
      [&]() {
        return AdjustPositionsUsingIk(*root_context, joints_to_adjust,
                                      floating_bodies_to_adjust,
                                      bodies_to_decollide,
                                      known_valid_position,
                                      IkMode::kInsideOut);
      },
      [&]() {
        return AdjustPositionsUsingIk(*root_context, joints_to_adjust,
                                      floating_bodies_to_adjust,
                                      bodies_to_decollide,
                                      known_valid_position,
                                      IkMode::kOutsideIn);
      },
      [&]() {
        return AdjustPositionsUsingBisect(*root_context, joints_to_adjust,
                                          floating_bodies_to_adjust,
                                          bodies_to_decollide,
                                          known_valid_position);
      }};
    for (auto method : methods) {
      VectorXd better_q;
      better_q = method();

      // Copy the result from the program into the (external) plant context.
      VectorXd new_positions = plant_->GetPositions(plant_context);
      std::vector<int> indexes_to_copy;
      for (const JointIndex j : joints_to_adjust) {
        const Joint<double>& joint = plant_->get_joint(j);
        for (int i = 0; i < joint.num_positions(); ++i) {
          int absolute_index = joint.position_start() + i;
          if (new_positions(absolute_index) != better_q(absolute_index)) {
            drake::log()->debug(
                "...repositioned {} (q[{}]) from {} to {} to avoid collisions.",
                joint.name(), absolute_index,
                new_positions(absolute_index), better_q(absolute_index));
          }
          new_positions[absolute_index] = better_q(absolute_index);
        }
      }
      for (const BodyIndex b : floating_bodies_to_adjust) {
        const Body<double>& body = plant_->get_body(b);
        if (!body.has_quaternion_dofs()) {
          // See above re: Drake#12807: we don't know the number of positions.
          continue;
        }
        for (int i = 0; i < 7; ++i) {
          int absolute_index = body.floating_positions_start() + i;
          if (new_positions(absolute_index) != better_q(absolute_index)) {
            drake::log()->debug(
                "...repositioned {} (q[{}]) from {} to {} to avoid collisions.",
                body.name(), absolute_index,
                new_positions(absolute_index), better_q(absolute_index));
          }
          new_positions[absolute_index] = better_q(absolute_index);
        }
      }

      // Copy our final result into a temporary context to verify that it is
      // collision-free.
      std::unique_ptr<Context<double>> working_root_context =
          root_context->Clone();
      Context<double>* working_plant_context =
          &diagram_->GetMutableSubsystemContext(*plant_,
                                                working_root_context.get());
      plant_->SetPositions(working_plant_context, new_positions);

      bool success = !Collides(*working_plant_context, bodies_to_decollide);
      if (success) {
        plant_->SetPositions(
            &diagram_->GetMutableSubsystemContext(*plant_, root_context),
            new_positions);
        return true;
      }
    }
    return false;
  }

  // Return the set of indexes in the state vector that *are not* adjustable
  // during optimization of the specified joint and body positions.
  std::set<int> ConstrainedStateIndexes(
      const std::set<JointIndex>& joints_to_adjust,
      const std::set<BodyIndex>& floating_bodies_to_adjust) {
    std::set<int> constrained_state_indexes;
    for (JointIndex j(0); j < plant_->num_joints(); ++j) {
      if (joints_to_adjust.count(j)) { continue; }
      const Joint<double>& joint = plant_->get_joint(j);
      int joint_begin = joint.position_start();
      int joint_end = joint.position_start() + joint.num_positions();
      for (int i = joint_begin; i < joint_end; ++i) {
        constrained_state_indexes.insert(i);
      }
    }
    for (BodyIndex b(0); b < plant_->num_bodies(); ++b) {
      if (floating_bodies_to_adjust.count(b)) { continue; }
      const Body<double>& body = plant_->get_body(b);
      if (!body.is_floating()) { continue; }
      int body_begin = body.floating_positions_start();
      int body_end = body.floating_positions_start() +
          (body.has_quaternion_dofs() ? 7 : 6);
      DRAKE_DEMAND(body.floating_positions_start() >= 0);
      for (int i = body_begin; i < body_end; ++i) {
        constrained_state_indexes.insert(i);
      }
    }
    return constrained_state_indexes;
  }

  // IK-based joint adjustment
  // @return a "q" value (vector of joint positions) better than that in `data`.
  //
  // The joint adjustment process here is a bit complicated.  There are in
  // principle two optimization approaches we could take to project to a valid
  // state:
  //
  //  * Inside-out:  Use the target condition as the initial conditions and
  //    adjustment distance as a cost function.
  //  * Outside-in: Use a known valid condition(*) as the initial conditions
  //    and distance-from-target-condition as a cost function.
  //
  // Empirically SNOPT is very bad at optimizing outward from an invalid
  // starting condition.  However the outside-in approach will often be slow
  // and can fail to find even a valid target condition if the constraints are
  // sufficiently nonconvex.
  //
  // For this reason we first attempt the inside-out problem.  This will
  // succeed quickly if the target is valid or almost valid and fail quickly
  // otherwise.  If it fails only then will we attempt the more expensive and
  // less accurate but likely-to-succeed outside-in approach.
  //
  // (* for joints subject to adjustment; insofar as the known valid
  // conditions don't match the target conditions for joints that are not
  // subject to adjustment, we must keep using the target conditions)
  VectorXd AdjustPositionsUsingIk(
      const Context<double>& root_context,
      const std::set<JointIndex>& joints_to_adjust,
      const std::set<BodyIndex>& floating_bodies_to_adjust,
      const std::optional<const std::set<BodyIndex>>& bodies_to_decollide,
      const VectorXd& known_valid_position,
      const IkMode mode) {
    // Contexts to use as scratch space, since Drake IK invalidates its
    // passed-in context argment if any Solve call fails.
    std::unique_ptr<Context<double>> working_root_context =
        root_context.Clone();
    Context<double>* working_plant_context =
        &diagram_->GetMutableSubsystemContext(*plant_,
                                              working_root_context.get());

    if (bodies_to_decollide) {
      // Reformulate our bodies of interest into vectors of Body* (because all
      // of the body-related methods take different types).
      std::vector<const Body<double>*> extraneous_bodies_vec;
      for (BodyIndex i(0); i < plant_->num_bodies(); ++i) {
        if (!bodies_to_decollide->count(i)) {
          extraneous_bodies_vec.push_back(&plant_->get_body(i));
        }
      }
      std::vector<const Body<double>*> bodies_to_decollide_vec;
      for (const BodyIndex body_index : *bodies_to_decollide) {
        bodies_to_decollide_vec.push_back(&plant_->get_body(body_index));
      }
      geom::GeometrySet decollide_geometries =
          plant_->CollectRegisteredGeometries(bodies_to_decollide_vec);
      geom::GeometrySet extraneous_geometries =
          plant_->CollectRegisteredGeometries(extraneous_bodies_vec);

      // Filter away all of the collisions not between members of `bodies`.
      Context<double>& working_sg_context =
          diagram_->GetMutableSubsystemContext(
              *scene_graph_, working_root_context.get());
      scene_graph_->ExcludeCollisionsWithin(&working_sg_context,
                                            extraneous_geometries);
      scene_graph_->ExcludeCollisionsBetween(&working_sg_context,
                                             decollide_geometries,
                                             extraneous_geometries);
    }

    // Build a MathematicalProgram for the IK solve.
    drake::multibody::InverseKinematics ik(*plant_, working_plant_context);
    ik.AddMinimumDistanceConstraint(kIkMinDistance);
    MathematicalProgram* program = ik.get_mutable_prog();
    VectorXd target_positions = plant_->GetPositions(*working_plant_context);
    const drake::solvers::VectorXDecisionVariable& vars = ik.q();

    // Constrain all variables other than positions of joints/bodies of concern.
    // Per Drake#12849 this has to warp to the joint limits.
    std::set<int> constrained_state_indexes =
        ConstrainedStateIndexes(joints_to_adjust, floating_bodies_to_adjust);
    for (const int i : constrained_state_indexes) {
      drake::symbolic::Variable var_in_program = vars(i);
      double target = drake::math::saturate(
          target_positions(i),
          plant_->GetPositionLowerLimits()(i),
          plant_->GetPositionUpperLimits()(i));
      program->AddConstraint(var_in_program == target);
    }

    // Constrain any unconstrained floating body quaternions to be unit-norm.
    // TODO(ggould) Possibly this belongs inside the drake::InverseKinematics
    // ctor rather than here?
    for (const BodyIndex id : floating_bodies_to_adjust) {
      if (plant_->get_body(id).has_quaternion_dofs()) {
        // Per `Body::floating_positions_start()`, the first four entries in
        // the floating position are guaranteed to be the quaternion.
        int quat_index = plant_->get_body(id).floating_positions_start();
        drake::symbolic::Expression norm = 0;
        for (int i = quat_index; i < quat_index + drake::kQuaternionSize; ++i) {
          norm += vars(i) * vars(i);
        }
        program->AddConstraint(norm == 1);
      }
    }

    // Add a cost function to prefer positions near the target.
    program->AddQuadraticErrorCost(
        drake::MatrixX<double>::Identity(vars.size(), vars.size()),
        target_positions, vars);

    // Configure the appropriate problem.
    switch (mode) {
      case IkMode::kInsideOut: {
        program->SetInitialGuess(vars, target_positions);
        drake::log()->debug(common::MathematicalProgramToString(
            "Inside-out program (start at target, hunt for valid)", *program));
        break;
      }
      case IkMode::kOutsideIn: {
        VectorXd guess = known_valid_position;
        for (const int i : constrained_state_indexes) {
          guess(i) = target_positions(i);
        }
        program->SetInitialGuess(vars, guess);
        drake::log()->debug(common::MathematicalProgramToString(
            "Outside-in program (start at valid, hunt for target)", *program));
        break;
      }
      default:
        DRAKE_UNREACHABLE();
    }

    // Try to solve.
    drake::solvers::MathematicalProgramResult result =
        drake::solvers::Solve(*program);
    if (!result.is_success()) {
      drake::log()->debug("Unable to solve AdjustPositions problem: {}",
                          result.get_solution_result());
    }
    return result.get_x_val();
  }

  // Bisect-based joint adjustment
  // @return a "q" value (vector of joint positions) better than that in `data`.
  //
  // Unlike IK, which can actually give us pretty and parsimonious adjustments
  // subject to interesting criteria, bisect is crude and nasty.  We take a
  // known valid position and simply bisect between it and the current context
  // value for a fixed number of iterations.  The result is demonstrably near
  // (but not likely *at*) some boundary of the constraint (but not
  // necessarily a particlarly *good* boundary or even the *outermost*
  // boundary for nonconvex constraints).
  //
  // The advantage is that it always works.  This makes it a very appealing
  // fallback when IK-based decollision is unavailable or unavailing.
  VectorXd AdjustPositionsUsingBisect(
      const Context<double>& root_context,
      const std::set<JointIndex>& joints_to_adjust,
      const std::set<BodyIndex>& floating_bodies_to_adjust,
      const std::optional<const std::set<BodyIndex>>& bodies_to_decollide,
      const VectorXd& known_valid_position) {
    // Context scratch space for collision checking.
    std::unique_ptr<Context<double>> working_root_context =
        root_context.Clone();
    Context<double>* working_plant_context =
        &diagram_->GetMutableSubsystemContext(*plant_,
                                              working_root_context.get());
    VectorXd original_q = plant_->GetPositions(*working_plant_context);

    // The validity function on which we will bisect:
    auto collides_at = [&](const VectorXd& q) -> bool {
      plant_->SetPositions(working_plant_context, q);
      return Collides(*working_plant_context, bodies_to_decollide);
    };

    // Start our search at `known_valid_position` for the `joints_to_adjust`
    // and at `root_context`'s current position for all other joints.
    VectorXd bad_value = original_q;
    VectorXd good_value = known_valid_position;
    std::set<int> constrained_state_indexes =
        ConstrainedStateIndexes(joints_to_adjust, floating_bodies_to_adjust);
    for (const int index : constrained_state_indexes) {
      good_value[index] = original_q[index];
    }

    drake::log()->debug("Attempting bisect between (bad)\n{} and (good)\n{}...",
                        PositionAsString(*plant_, bad_value),
                        PositionAsString(*plant_, good_value));
    for (int i = 0; i < kBisectIterations; ++i) {
      // Invariant for the bisect.
      DRAKE_DEMAND(collides_at(bad_value));
      // This is not guaranteed to always be true due to non-known-good values
      // in the constrained joints.  However we also don't have any useful
      // fallback from this state, so failing here gives a clearer crash than
      // returning false and letting the caller fail.
      // TODO(ggould) Find a better plan for this if it happens in practice.
      DRAKE_DEMAND(!collides_at(good_value));

      VectorXd halfway = (bad_value + good_value) / 2;

      // The "/ 2" there could have denormed our quaternions.  Fix them.
      for (const BodyIndex id : floating_bodies_to_adjust) {
        if (plant_->get_body(id).has_quaternion_dofs()) {
          // Per `Body::floating_positions_start()`, the first four entries in
          // the floating position are guaranteed to be the quaternion.
          int quat_index = plant_->get_body(id).floating_positions_start();
          Eigen::VectorBlock<VectorXd, drake::kQuaternionSize>(
              halfway, quat_index).normalize();
        }
      }

      if (collides_at(halfway)) {
        bad_value = halfway;
      } else {
        good_value = halfway;
      }
    }
    drake::log()->debug("...found non-colliding value {}.", good_value);
    return good_value;
  }

  bool Collides(const Context<double>& plant_context,
                const std::optional<const std::set<BodyIndex>>& bodies) const {
    // Precondition:
    DRAKE_DEMAND(plant_context.GetSystemPathname() ==
                 plant_->GetSystemPathname());

    geom::QueryObject<double> query_object =
        plant_->get_geometry_query_input_port().
        Eval<geom::QueryObject<double>>(plant_context);
    const geom::SceneGraphInspector<double>& inspector =
        query_object.inspector();
    const std::vector<geom::PenetrationAsPointPair<double>>& point_pairs =
        query_object.ComputePointPairPenetration();
    if (point_pairs.empty()) {
      drake::log()->debug("Configuration is collision-free ({} bodies).",
                          bodies ? std::to_string(bodies->size()) : "default");
    }

    for (const geom::PenetrationAsPointPair<double>& point_pair : point_pairs) {
      const geom::FrameId frame_id_A = inspector.GetFrameId(point_pair.id_A);
      const geom::FrameId frame_id_B = inspector.GetFrameId(point_pair.id_B);
      const Body<double>* body_A = plant_->GetBodyFromFrameId(frame_id_A);
      const Body<double>* body_B = plant_->GetBodyFromFrameId(frame_id_B);
      DRAKE_THROW_UNLESS(body_A);
      DRAKE_THROW_UNLESS(body_B);
      drake::log()->debug("Found collision between {} and {}...",
                          body_A->name(), body_B->name());
      if (!bodies || (
              bodies->count(body_A->index()) &&
              bodies->count(body_B->index()))) {
        drake::log()->debug("...which is flagged as a collision.");
        return true;
      }
      drake::log()->debug("...and ignored it.");
    }
    return false;
  }

  const Diagram<double>* diagram_;
  const MultibodyPlant<double>* plant_;
  const SceneGraph<double>* scene_graph_;
};

CollisionRemover::CollisionRemover(
    const Diagram<double>* diagram,
    const MultibodyPlant<double>* plant,
    const SceneGraph<double>* scene_graph)
  : impl_(new CollisionRemover::Impl(diagram, plant, scene_graph)) {}

CollisionRemover::~CollisionRemover() {}

bool CollisionRemover::AdjustPositions(
    Context<double>* root_context,
    const std::set<JointIndex>& joints_to_adjust,
    const std::set<BodyIndex>& floating_bodies_to_adjust,
    const std::optional<const std::set<BodyIndex>>& bodies_to_decollide,
    const VectorXd& known_valid_position) const {
  return impl_->AdjustPositions(root_context, joints_to_adjust,
                             floating_bodies_to_adjust, bodies_to_decollide,
                             known_valid_position);
}

bool CollisionRemover::Collides(
    const Context<double>& context,
    const std::optional<const std::set<BodyIndex>>& bodies_to_decollide) const {
  return impl_->Collides(context, bodies_to_decollide);
}

}  // namespace sim
}  // namespace anzu
