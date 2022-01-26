#pragma once

#include "drake/common/drake_deprecated.h"
#include "drake/multibody/meshcat/contact_visualizer.h"
#include "drake/multibody/plant/contact_results_to_meshcat_params.h"

namespace drake {
namespace multibody {

template <typename T>
using ContactResultsToMeshcat
    DRAKE_DEPRECATED("2022-05-01",
        "Spell as drake::multibody::meshcat::ContactVisualizer instead.")
    = meshcat::ContactVisualizer<T>;

}  // namespace multibody
}  // namespace drake
