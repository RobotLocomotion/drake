#pragma once

// Note: the user should not include this header in their code. This header is
// created for internal use only.

#include <vector>

#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace multibody {
namespace internal {
/*
 * Finds the path on the kinematic tree from start to the end.
 * Here we make the following assumptions
 * 1. The `plant` kinematic topology is a tree with every link on being
 * reachable. Hence for any two bodies, there exists one and only one path
 * between the two bodies.
 * 2. When returning the path, the link that is closer to `start` comes before
 * the link farther to `start`.
 * 3. If the path goes through the world link, that world link is also included
 * in the path.
 * TODO(hongkai.dai): handle the case that there exists a kinematics loop, or
 * the path doesn't exists, and move this function out to
 * `MultibodyTreeTopology`.
 */
std::vector<BodyIndex> FindPath(const MultibodyPlant<double>& plant,
                                BodyIndex start, BodyIndex end);

/*
 * Finds all the mobilizer on the path from start to the end.
 */
std::vector<internal::MobodIndex> FindMobilizersOnPath(
    const MultibodyPlant<double>& plant, BodyIndex start, BodyIndex end);

/*
 * Finds the body in the middle of the kinematic chain that goes from the start
 * to the end. Notice that we ignore the welded joint, and only count revolute
 * joint as one step along the chain, we throw an error when hitting
 * non-revolute weld mobilizers.
 */
BodyIndex FindBodyInTheMiddleOfChain(const MultibodyPlant<double>& plant,
                                     BodyIndex start, BodyIndex end);
}  // namespace internal
}  // namespace multibody
}  // namespace drake
