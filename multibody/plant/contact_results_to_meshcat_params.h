#pragma once

#include "drake/common/drake_deprecated.h"
#include "drake/multibody/meshcat/contact_visualizer_params.h"

namespace drake {
namespace multibody {

using ContactResultsToMeshcatParams
    DRAKE_DEPRECATED("2022-05-01",
        "Spell as drake::multibody::meshcat::ContactVisualizerParams instead.")
    = meshcat::ContactVisualizerParams;

}  // namespace multibody
}  // namespace drake
