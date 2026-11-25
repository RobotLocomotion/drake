#include "drake/examples/multibody/four_bar/assemble_nearest_configuration.h"

#include <memory>
#include <stdexcept>

#include "drake/multibody/inverse_kinematics/position_constraint.h"
#include "drake/multibody/inverse_kinematics/unit_quaternion_constraint.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace examples {
namespace multibody {
namespace four_bar {

using drake::multibody::Frame;
using drake::multibody::Joint;
using drake::multibody::JointIndex;
using drake::multibody::MultibodyPlant;
using drake::multibody::PositionConstraint;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using solvers::MathematicalProgram;
using solvers::MathematicalProgramResult;
using solvers::VectorXDecisionVariable;
using systems::Context;

MathematicalProgramResult AssembleNearestConfiguration(
    const MultibodyPlant<double>& plant, Context<double>* plant_context,
    double tolerance) {
  if (plant.num_constraints() != plant.num_weld_constraints()) {
    throw std::logic_error(
        "AssembleNearestConfiguration() can satisfy only weld constraints.");
  }
  const VectorXd q_start = plant.GetPositions(*plant_context);
  const int nq = plant.num_positions();

  MathematicalProgram prog;
  const VectorXDecisionVariable q = prog.NewContinuousVariables(nq, "q");

  // Look for the assembled configuration nearest the starting one.
  prog.AddQuadraticErrorCost(MatrixXd::Identity(nq, nq), q_start, q);

  // Keep the joints within their limits and the locked joints where they are,
  // and keep any quaternions unit length.
  prog.AddBoundingBoxConstraint(plant.GetPositionLowerLimits(),
                                plant.GetPositionUpperLimits(), q);
  for (JointIndex index : plant.GetJointIndices()) {
    const Joint<double>& joint = plant.get_joint(index);
    if (joint.is_locked(*plant_context)) {
      const int start = joint.position_start();
      const int count = joint.num_positions();
      prog.AddBoundingBoxConstraint(q_start.segment(start, count),
                                    q_start.segment(start, count),
                                    q.segment(start, count));
    }
  }
  drake::multibody::AddUnitQuaternionConstraintOnPlant(plant, q, &prog);

  // A weld constraint holds frame P, fixed on body A, coincident with frame Q,
  // fixed on body B. We impose that by requiring three points R to coincide,
  // each at the same coordinates r in both P and Q: the frames' origins, and
  // points a meter along their x and y axes. When three points that are not
  // in a line coincide, so do the frames.
  //
  // Each point's coordinates are held within `tolerance` rather than exactly.
  // A weld amounts to six conditions, but in a planar mechanism three of them
  // hold automatically, so stating them all as equalities would be redundant.
  // Some solvers (IPOPT, for one) give up on a problem with more equalities
  // than unknowns; narrow inequalities like these don't trouble them.
  //
  // N.B. MultibodyPlant does not yet offer a public way to ask for the frames
  // of its weld constraints, so this uses get_weld_constraint_specs(), which is
  // marked "(Internal use only)".
  const Vector3d slack = Vector3d::Constant(tolerance);
  for (const auto& [id, weld] : plant.get_weld_constraint_specs()) {
    if (!plant.GetConstraintActiveStatus(*plant_context, id)) continue;
    const Frame<double>& A = plant.get_body(weld.body_A).body_frame();
    const Frame<double>& B = plant.get_body(weld.body_B).body_frame();
    for (const Vector3d& r :
         {Vector3d(0, 0, 0), Vector3d(1, 0, 0), Vector3d(0, 1, 0)}) {
      const Vector3d p_AR = weld.X_AP * r;  // Where R should be on body A.
      const Vector3d p_BR = weld.X_BQ * r;  // Where R is on body B.
      prog.AddConstraint(
          std::make_shared<PositionConstraint>(
              &plant, A, p_AR - slack, p_AR + slack, B, p_BR, plant_context),
          q);
    }
  }

  const MathematicalProgramResult result = solvers::Solve(prog, q_start);
  // The constraints were evaluated using `plant_context`, so it holds whatever
  // positions were tried last. Set them explicitly either way.
  plant.SetPositions(plant_context,
                     result.is_success() ? result.GetSolution(q) : q_start);
  return result;
}

}  // namespace four_bar
}  // namespace multibody
}  // namespace examples
}  // namespace drake
