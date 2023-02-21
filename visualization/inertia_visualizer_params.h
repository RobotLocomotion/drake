#pragma once

#include "drake/common/name_value.h"

namespace drake {
namespace visualization {

/** The set of parameters for configuring InertiaVisualizer. */
struct InertiaVisualizerParams {
  /** Passes this object to an Archive.
   Refer to @ref yaml_serialization "YAML Serialization" for background. */
  template <typename Archive>
  void Serialize(Archive* a) {}

  // XXX
};

}  // namespace visualization
}  // namespace drake
