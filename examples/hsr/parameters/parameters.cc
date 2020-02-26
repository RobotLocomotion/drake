#include "drake/examples/hsr/parameters/parameters.h"

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

DEFINE_double(integration_accuracy, 1.0E-6,
              "When time_step = 0 (plant is modeled as a continuous system), "
              "this is the desired integration accuracy.  This value is not "
              "used if time_step > 0 (fixed-time step).");
DEFINE_double(penetration_allowance, 5.0E-3, "Allowable penetration (meters).");
DEFINE_double(v_stiction_tolerance, 1.0E-2,
              "The maximum slipping speed allowed during stiction. [m/s]");
DEFINE_double(inclined_plane_coef_static_friction, 0.6,
              "Inclined plane's coefficient of static friction (no units).");
DEFINE_double(inclined_plane_coef_kinetic_friction, 0.6,
              "Inclined plane's coefficient of kinetic friction (no units).  "
              "When time_step > 0, this value is ignored.  Only the "
              "coefficient of static friction is used in fixed-time step.");

DEFINE_double(kp, 200.0,
              "The constant default p gain for the inverse dynamic controller");

DEFINE_double(kd, 40.0,
              "The constant default d gain for the inverse dynamic controller");

DEFINE_double(ki, 2.0,
              "The constant default i gain for the inverse dynamic controller");

namespace drake {
namespace examples {
namespace hsr {

namespace {
Parameters CreateParameters() {
  Parameters result;
  result.simulation_time = FLAGS_simulation_time;
  result.time_step = FLAGS_time_step;
  result.gravity = FLAGS_gravity;
  result.integration_accuracy = FLAGS_integration_accuracy;
  result.v_stiction_tolerance = FLAGS_v_stiction_tolerance;
  result.target_realtime_rate = FLAGS_target_realtime_rate;
  result.penetration_allowance = FLAGS_penetration_allowance;
  result.inclined_plane_coef_static_friction =
      FLAGS_inclined_plane_coef_static_friction;
  result.inclined_plane_coef_kinetic_friction =
      FLAGS_inclined_plane_coef_kinetic_friction;
  result.kp = FLAGS_kp;
  result.kd = FLAGS_kd;
  result.ki = FLAGS_ki;
  return result;
}

}  // namespace

const Parameters& hsr_sim_flags() {
  static const drake::never_destroyed<Parameters> global(CreateParameters());
  return global.access();
}

}  // namespace hsr
}  // namespace examples
}  // namespace drake
