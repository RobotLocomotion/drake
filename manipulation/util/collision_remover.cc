#include "drake/manipulation/util/collision_remover.h"

#include <algorithm>
#include <functional>
#include <limits>
#include <ostream>
#include <string>
#include <vector>

#include "drake/common/constants.h"
#include "drake/common/sorted_pair.h"
#include "drake/common/symbolic_variable.h"
#include "drake/geometry/query_object.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/inverse_kinematics/inverse_kinematics.h"
#include "drake/multibody/inverse_kinematics/minimum_distance_constraint.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace manipulation {
namespace util {

using Eigen::VectorXd;
using geometry::SceneGraph;
using solvers::MathematicalProgram;
using systems::Context;
using systems::Diagram;
using multibody::Body;
using multibody::BodyIndex;
using multibody::Joint;
using multibody::JointIndex;
using multibody::MultibodyPlant;

namespace geom = drake::geometry;

namespace {
// How many iterations of bisect to try.  Setting this high doesn't help much.
constexpr static int kBisectIterations = 10;

// How far back from collision the IK MinimumDistanceConstraint should aim;
// setting this too low is bad due to numeric issues in the constraint function.
constexpr static double kIkMinDistance = 1e-2;

// An ad-hoc object that captures objects used by multiple `AdjustPositions`
// implementations to reduce copypasta.
struct CollisionRemoverWorkingData {
  // The original (from the input `root_context`) values of the joint positions.
  VectorXd original_q;

  // A throwaway copy of the diagram context and of its enclosed plant context.
  std::unique_ptr<Context<double>> working_root_context;
  Context<double>* working_plant_context;
};

std::string PositionAsString(const MultibodyPlant<double>& plant,
                             VectorXd pos) {
  std::ostringstream output;
  for (int i=0; i < pos.size(); i++) {
    bool found = false;
    for (drake::multibody::ModelInstanceIndex m{};
         m < plant.num_model_instances(); m++) {
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

// Describe the contents of a MathematicalProgram, for debugging purposes;
// this is intended for making sense of failing IK programs which are
// otherwise hard to interpret.
// TODO(ggould-tri) move this to its own header, or replace it with a method
// of MathematicalProgram when such exists.
std::string MathematicalProgramToString(
    const std::string& program_name,
    const drake::solvers::MathematicalProgram& program) {
  constexpr static double kInfinity = std::numeric_limits<double>::infinity();
  std::stringstream result;
  result << "MathematicalProgram for " << program_name << ":" << std::endl;
  result << "  Decision Variables:" << std::endl;
  Eigen::VectorXd guesses =
      program.GetInitialGuess(program.decision_variables());
  for (int i = 0; i < program.num_vars(); i++) {
    result << "    " << i << ": " << program.decision_variable(i)
           << " <- " << guesses(i) << std::endl;
  }
  result << "  Indeterminates:" << std::endl;
  for (int i = 0; i < program.num_indeterminates(); i++) {
    result << "    " << i << ": " << program.indeterminate(i) << std::endl;
  }
  result << "  Constraints:" << std::endl;
  for (const auto& binding : program.GetAllConstraints()) {
    result << "    ";
    auto constraint = binding.evaluator();
    result << "'" << constraint->get_description() << "': ";
    if (dynamic_cast<drake::multibody::MinimumDistanceConstraint*>(
            constraint.get())) {
      result << "MinimumDistanceConstraint on";
      for (int i = 0; i < binding.variables().size(); i++) {
        result << " " << binding.variables()(i);
      }
    } else {
      drake::VectorX<drake::symbolic::Expression> exprs;
      constraint->Eval(binding.variables(), &exprs);
      for (int i = 0; i < exprs.size(); i++) {
        if (constraint->lower_bound()(i) > -kInfinity) {
          result << constraint->lower_bound()(i) << "<=";
        }
        result << exprs(i);
        if (constraint->upper_bound()(i) < kInfinity) {
          result << "<=" << constraint->upper_bound()(i);
        }
        result << "; ";
      }
    }
    result << std::endl;
  }
  result << "  Costs:" << std::endl;
  for (const auto& binding : program.GetAllCosts()) {
    result << "    ";
    auto cost = binding.evaluator();
    result << "'" << cost->get_description() << "': ";
    drake::VectorX<drake::symbolic::Expression> exprs;
    cost->Eval(binding.variables(), &exprs);
    for (int i = 0; i < exprs.size(); i++) {
      result << exprs(i);
    }
    result << std::endl;
  }
  return result.str();
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
      drake::log()->info("AdjustPositions called with an empty body set.");
      return true;
    } else if (!Collides(plant_context, bodies_to_decollide)) {
      drake::log()->info("AdjustPositions but no collisions are present.");
      return true;
    }
    if (joints_to_adjust.empty() && floating_bodies_to_adjust.empty()) {
      return false;  // If we cannot adjust, failure is assured.
    }

    // Populate common data.
    CollisionRemoverWorkingData data;
    data.original_q = plant_->GetPositions(plant_context);
    data.working_root_context = root_context->Clone();
    data.working_plant_context =
        &diagram_->GetMutableSubsystemContext(*plant_,
                                              data.working_root_context.get());

    // Dispatch to our strategies.  Try IK first; if it fails then bisect.
    std::vector<std::function<VectorXd(void)>> methods = {
      [&]() {
        return AdjustPositionsUsingIk(&data, joints_to_adjust,
                                      floating_bodies_to_adjust,
                                      bodies_to_decollide,
                                      known_valid_position);
      },
      [&]() {
        return AdjustPositionsUsingBisect(&data, joints_to_adjust,
                                          floating_bodies_to_adjust,
                                          bodies_to_decollide,
                                          known_valid_position);
      }};
    for (auto method : methods) {
      VectorXd better_q;
      better_q = method();

      // Copy the result from the program into the (external) plant context.
      VectorXd new_positions = data.original_q;
      std::vector<int> indexes_to_copy;
      for (JointIndex j : joints_to_adjust) {
        const Joint<double>& joint = plant_->get_joint(j);
        for (int i = 0; i < joint.num_positions(); i++) {
          indexes_to_copy.push_back(joint.position_start() + i);
          if (new_positions(i) != better_q(i)) {
            drake::log()->info(
                "...repositioned {} (q[{}]) from {} to {} to avoid collisions.",
                joint.name(), i, new_positions(i), better_q(i));
          }
        }
      }
      for (BodyIndex b : floating_bodies_to_adjust) {
        const Body<double>& body = plant_->get_body(b);
        if (!body.has_quaternion_dofs()) {
          // See above re: Drake#12807: we don't know the number of positions.
          continue;
        }
        for (int i = 0; i < 7; i++) {
          indexes_to_copy.push_back(body.floating_positions_start() + i);
          if (new_positions(i) != better_q(i)) {
            drake::log()->info(
                "...repositioned {} (q[{}]) from {} to {} to avoid collisions.",
                body.name(), i, new_positions(i), better_q(i));
          }
        }
      }
      for (int i : indexes_to_copy) { new_positions[i] = better_q(i); }
      plant_->SetPositions(
          &diagram_->GetMutableSubsystemContext(*plant_, root_context),
          new_positions);

      bool success = !Collides(
          diagram_->GetSubsystemContext(*plant_, *root_context),
          bodies_to_decollide);
      if (success) { return true; }
    }
    return false;
  }

  std::set<int> ConstrainedStateIndexes(
      const std::set<JointIndex>& joints_to_adjust,
      const std::set<BodyIndex>& floating_bodies_to_adjust) {
    std::set<int> constrained_state_indexes;
    for (JointIndex j(0); j < plant_->num_joints(); j++) {
      if (joints_to_adjust.count(j)) { continue; }
      const Joint<double>& joint = plant_->get_joint(j);
      int joint_begin = joint.position_start();
      int joint_end = joint.position_start() + joint.num_positions();
      for (int i = joint_begin; i < joint_end; i++) {
        constrained_state_indexes.insert(i);
      }
    }
    for (BodyIndex b(0); b < plant_->num_bodies(); b++) {
      if (floating_bodies_to_adjust.count(b)) { continue; }
      const Body<double>& body = plant_->get_body(b);
      if (!body.is_floating()) { continue; }
      int body_begin = body.floating_positions_start();
      int body_end = body.floating_positions_start() +
          (body.has_quaternion_dofs() ? 7 : 6);
      DRAKE_DEMAND(body.floating_positions_start() >= 0);
      for (int i = body_begin; i < body_end; i++) {
        constrained_state_indexes.insert(i);
      }
    }
    return constrained_state_indexes;
  }

  // IK-based joint adjustment
  // @return a "q" value (vector of joint positions) better than that in `data`.
  //
  // The joint adjustment process here is a bit complicated.  There are in
  // principle two optimzation approaches we could take to project to a valid
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
      CollisionRemoverWorkingData* data,
      const std::set<JointIndex>& joints_to_adjust,
      const std::set<BodyIndex>& floating_bodies_to_adjust,
      const std::optional<const std::set<BodyIndex>>& bodies_to_decollide,
      const VectorXd& known_valid_position) {
    if (bodies_to_decollide) {
      // Reformulate our bodies of interest into vectors of Body* (because all
      // of the body-related methods take different types).
      std::vector<const Body<double>*> extraneous_bodies_vec;
      for (BodyIndex i(0); i < plant_->num_bodies(); i++) {
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
              *scene_graph_, data->working_root_context.get());
      scene_graph_->ExcludeCollisionsWithin(&working_sg_context,
                                            extraneous_geometries);
      scene_graph_->ExcludeCollisionsBetween(&working_sg_context,
                                             decollide_geometries,
                                             extraneous_geometries);
    }

    // Build a MathematicalProgram for the IK solve.
    drake::multibody::InverseKinematics ik(*plant_,
                                           data->working_plant_context);
    ik.AddMinimumDistanceConstraint(kIkMinDistance);
    MathematicalProgram* program = ik.get_mutable_prog();
    VectorXd target_positions =
        plant_->GetPositions(*data->working_plant_context);
    const drake::solvers::VectorXDecisionVariable& vars = ik.q();

    // Constrain all variables other than positions of joints/bodies of concern.
    std::set<int> constrained_state_indexes =
        ConstrainedStateIndexes(joints_to_adjust, floating_bodies_to_adjust);
    for (int i : constrained_state_indexes) {
      drake::symbolic::Variable var_in_program = vars(i);
      program->AddConstraint(var_in_program == target_positions(i));
    }

    // Constrain any unconstrained floating body quaternions to be unit-norm.
    // TODO(ggould-tri) Possibly this belongs inside the InverseKinematics
    // ctor instead of here?
    for (const BodyIndex id : floating_bodies_to_adjust) {
      if (plant_->get_body(id).has_quaternion_dofs()) {
        // Per `Body::floating_positions_start()`, the first four entries in
        // the floating position are guaranteed to be the quaternion.
        int quat_index = plant_->get_body(id).floating_positions_start();
        drake::symbolic::Expression norm = 0;
        for (int i = quat_index; i < quat_index + drake::kQuaternionSize; i++) {
          norm += vars(i) * vars(i);
        }
        program->AddConstraint(norm == 1);
      }
    }

    // Add a cost function to prefer positions near the target.
    program->AddQuadraticErrorCost(
        drake::MatrixX<double>::Identity(vars.size(), vars.size()),
        target_positions, vars);

    // Configure the "inside-out" problem.
    program->SetInitialGuess(vars, target_positions);
    drake::log()->info(MathematicalProgramToString(
        "Inside-out program (start at target, hunt for valid)", *program));

    // Try to solve.
    drake::solvers::MathematicalProgramResult result =
        drake::solvers::Solve(*program);
    if (!result.is_success()) {
      drake::log()->info(
          "Unable to solve AdjustPositions problem inside-out: {}",
          result.get_solution_result());

      // Try the "outside-in" problem.
      VectorXd guess = known_valid_position;
      for (int i : constrained_state_indexes) {
        guess(i) = target_positions(i);
      }
      program->SetInitialGuess(vars, guess);
      drake::log()->info(MathematicalProgramToString(
          "Outside-in program (start at valid, hunt for target)", *program));
      result = drake::solvers::Solve(*program);
    }
    if (!result.is_success()) {
      drake::log()->info("Unable to solve AdjustPositions problem: {}",
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
      CollisionRemoverWorkingData* data,
      const std::set<JointIndex>& joints_to_adjust,
      const std::set<BodyIndex>& floating_bodies_to_adjust,
      const std::optional<const std::set<BodyIndex>>& bodies_to_decollide,
      const VectorXd& known_valid_position) {
    // The validity function on which we will bisect:
    auto collides_at = [&](const VectorXd& q) -> bool {
      plant_->SetPositions(data->working_plant_context, q);
      return Collides(*data->working_plant_context, bodies_to_decollide);
    };

    // Start our search at `known_valid_position` for the `joints_to_adjust`
    // and at `root_context`'s current position for all other joints.
    VectorXd bad_value = data->original_q;
    VectorXd good_value = known_valid_position;
    std::set<int> constrained_state_indexes =
        ConstrainedStateIndexes(joints_to_adjust, floating_bodies_to_adjust);
    for (int index : constrained_state_indexes) {
      good_value[index] = data->original_q[index];
    }

    drake::log()->info("Attempting bisect between (bad)\n{} and (good)\n{}...",
                       PositionAsString(*plant_, bad_value),
                       PositionAsString(*plant_, good_value));
    for (int i = 0; i < kBisectIterations; i++) {
      // Invariant for the bisect.
      DRAKE_DEMAND(collides_at(bad_value));
      // This is not guaranteed to always be true due to non-known-good values
      // in the constrained joints.  However we also don't have any useful
      // fallback from this state, so failing here gives a clearer crash than
      // returning false and letting the caller fail.
      // TODO(ggould-tri) Find a better plan for this if it happens in practice.
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
    drake::log()->info("...found non-colliding value {}.", good_value);
    return good_value;
  }

  bool Collides(const Context<double>& plant_context,
                const std::optional<const std::set<BodyIndex>>& bodies) {
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
      drake::log()->info("Configuration is collision-free ({} bodies).",
                         bodies ? std::to_string(bodies->size()) : "default");
    }

    for (const geom::PenetrationAsPointPair<double>& point_pair : point_pairs) {
      const geom::FrameId frame_id_A = inspector.GetFrameId(point_pair.id_A);
      const geom::FrameId frame_id_B = inspector.GetFrameId(point_pair.id_B);
      const Body<double>* body_A = plant_->GetBodyFromFrameId(frame_id_A);
      const Body<double>* body_B = plant_->GetBodyFromFrameId(frame_id_B);
      DRAKE_THROW_UNLESS(body_A);
      DRAKE_THROW_UNLESS(body_B);
      drake::log()->info("Found collision between {} and {}...",
                         body_A->name(), body_B->name());
      if (!bodies || (
              bodies->count(body_A->index()) &&
              bodies->count(body_B->index()))) {
        drake::log()->info("...which is flagged as a collision.");
        return true;
      }
      drake::log()->info("...and ignored it.");
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
    const VectorXd& known_valid_position) {
  return impl_->AdjustPositions(root_context, joints_to_adjust,
                             floating_bodies_to_adjust, bodies_to_decollide,
                             known_valid_position);
}

}  // namespace util
}  // namespace manipulation
}  // namespace drake
