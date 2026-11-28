#pragma once

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/solvers/mathematical_program_result.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace examples {
namespace strandbeest_unassembled {

/** Assembles a model whose constraints are not yet satisfied, by moving the
generalized positions q in `plant_context` to the nearest configuration (the
smallest sum of squared changes to q) at which every active weld constraint in
`plant` is satisfied. That includes the welds that Finalize() adds to close
kinematic loops, as well as any you added yourself. Locked joints stay where
they are, joints stay within their position limits, and quaternions stay unit
length. The welds are satisfied to within about `tolerance` meters.

A mechanism with kinematic loops can usually be assembled in more than one way.
This function finds the way nearest the starting configuration, so describe
your model so that it starts near the configuration you intend.

Returns the solver's result. On failure, the positions in `plant_context` are
left as they were.

Nothing here is specific to the Strandbeest; you can copy this function for
your own models.

@throws std::exception if `plant` has constraints other than welds. Satisfying
the other kinds too is future work. */
solvers::MathematicalProgramResult AssembleNearestConfiguration(
    const drake::multibody::MultibodyPlant<double>& plant,
    systems::Context<double>* plant_context, double tolerance = 1.0e-6);

}  // namespace strandbeest_unassembled
}  // namespace examples
}  // namespace drake
