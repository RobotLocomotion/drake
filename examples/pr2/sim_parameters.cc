#include "drake/examples/pr2/sim_parameters.h"

#include <limits>

#include <gflags/gflags.h>

#include "drake/common/never_destroyed.h"

DEFINE_double(simulation_time, 20,
              "Desired duration of the simulation in seconds");

DEFINE_double(time_step, 1.0e-3,
              "Simulation time step used for the discrete systems.");

DEFINE_double(gravity, 9.8, "Value of gravity in the direction of -z.");

DEFINE_double(target_realtime_rate, 1,
              "Desired rate relative to real time.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

DEFINE_double(penetration_allowance, 5.0E-3, "Allowable penetration (meters).");
DEFINE_double(v_stiction_tolerance, 1.0E-2,
              "The maximum slipping speed allowed during stiction. [m/s]");
DEFINE_double(inclined_plane_coef_static_friction, 0.6,
              "Inclined plane's coefficient of static friction (no units).");
DEFINE_double(inclined_plane_coef_kinetic_friction, 0.6,
              "Inclined plane's coefficient of kinetic friction (no units).  "
              "When time_step > 0, this value is ignored.  Only the "
              "coefficient of static friction is used in fixed-time step.");

namespace drake {
namespace examples {
namespace pr2 {

namespace {
SimParameters CreateSimParameters() {
  SimParameters result;
  result.simulation_time = FLAGS_simulation_time;
  result.time_step = FLAGS_time_step;
  result.gravity = FLAGS_gravity;
  result.v_stiction_tolerance = FLAGS_v_stiction_tolerance;
  result.target_realtime_rate = FLAGS_target_realtime_rate;
  result.penetration_allowance = FLAGS_penetration_allowance;
  result.inclined_plane_coef_static_friction =
      FLAGS_inclined_plane_coef_static_friction;
  result.inclined_plane_coef_kinetic_friction =
      FLAGS_inclined_plane_coef_kinetic_friction;
  return result;
}

}  // namespace

const SimParameters& sim_flags() {
  static const drake::never_destroyed<SimParameters> global(
      CreateSimParameters());
  return global.access();
}

}  // namespace pr2
}  // namespace examples
}  // namespace drake
