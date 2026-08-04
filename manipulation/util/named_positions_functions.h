#pragma once

#include "drake/common/string_map.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace manipulation {

/** A map-of-maps {model_instance_name: {joint_name: joint_positions}} that
describes joint positions. Note that a given joint's positions are always
spelled as a vector, even though for the most common joint types (prismatic,
revolute, screw, etc.) the vector will only contain a single element. */
using NamedPositions = drake::string_map<drake::string_map<Eigen::VectorXd>>;

/** Sets default joint positions in the given `plant` according to the given
`input` (which specifies joints by their string name). The default positions
for joints not mentioned will remain unchanged.

@throws std::exception if the number of position values in the input and the
actual number of positions for the intended joint do not match. */
void ApplyNamedPositionsAsDefaults(
    const NamedPositions& input,
    drake::multibody::MultibodyPlant<double>* plant);

}  // namespace manipulation
}  // namespace drake
