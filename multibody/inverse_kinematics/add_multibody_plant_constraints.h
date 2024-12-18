#pragma once

#include <vector>

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/mathematical_program.h"

namespace drake {
namespace multibody {

/** For unit quaternion and (holonomic) constraints registered with `plant` adds
a corresponding solver::Constraint to `prog`, using decision variables `q` to
represent the generalized positions of the plant.

Adds constraints for coupler, distance, ball, and weld constraints. The
distance constraint is implemented here as a hard kinematic constraint (i.e.,
d(q) == d₀), instead of a soft "spring" force.

@see AddUnitQuaternionConstraintOnPlant() for the unit quaternion constraints.

@pre plant.is_finalized() == true. */
std::vector<solvers::Binding<solvers::Constraint>> AddMultibodyPlantConstraints(
    const MultibodyPlant<double>& plant,
    const solvers::VectorXDecisionVariable& q,
    solvers::MathematicalProgram* prog,
    systems::Context<double>* plant_context);

}  // namespace multibody
}  // namespace drake
