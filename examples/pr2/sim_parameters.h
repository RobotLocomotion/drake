#pragma once

#include <string>

namespace drake {
namespace examples {
namespace pr2 {

/// A structure to hold values that are constant at runtime after
/// command-line processing.  Access the sole instance via `flags()`.
struct SimParameters {
  double gravity{};
  double target_realtime_rate{};
  double penetration_allowance{};
  double v_stiction_tolerance{};
  double inclined_plane_coef_static_friction{};
  double inclined_plane_coef_kinetic_friction{};
  double simulation_time{};
  double time_step{};
};

/// @return a constref to the singleton `SimParameters` instance.
const SimParameters& sim_flags();

}  // namespace pr2
}  // namespace examples
}  // namespace drake
