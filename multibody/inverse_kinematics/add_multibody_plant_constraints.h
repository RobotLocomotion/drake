#pragma once

#include <memory>
#include <vector>

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace multibody {

/** For all kinematic constraints associated with `plant` adds a corresponding
solver::Constraint to `prog`, using decision variables `q` to represent the
generalized positions of the plant.

Adds joint limits constraints, unit quaternion constraints, and constraints for
any locked joints (via Joint::Lock()). Note that you must pass a valid
`plant_context` to use joint locking.

Adds constraints for coupler, distance, ball, and weld constraints. The
distance constraint is implemented here as a hard kinematic constraint (i.e.,
d(q) == d₀), instead of a soft "spring" force.

@see AddUnitQuaternionConstraintOnPlant() for the unit quaternion constraints.

@pre plant.is_finalized() == true.
@throws std::exception if `plant` has constraints registered that are not yet
supported by this method.
@throws std::exception if `prog` is nullptr.
@throws std::exception if `plant_context` is nullptr and one of the
MultibodyPlant constraints requires it. (unit quaternion constraints and coupler
constraints do not).
*/
std::vector<solvers::Binding<solvers::Constraint>> AddMultibodyPlantConstraints(
    const std::shared_ptr<const MultibodyPlant<double>>& plant,
    const solvers::VectorXDecisionVariable& q,
    solvers::MathematicalProgram* prog,
    systems::Context<double>* plant_context = nullptr);

}  // namespace multibody
}  // namespace drake
