#include <memory>

#include "drake/lcm/drake_lcm_interface.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"

namespace drake {
namespace examples {
namespace toyota_hsrb {

/**
 * Creates a systems::Simulator object that implements the Toyota HSRb passive
 * demo. The returned simulator can then be executed by the application. This
 * factory method is needed to separate the demo's executable from its unit
 * test's executable. Two separate executables are needed because ROS tests
 * implement a special way of reporting test results. This prevents ROS tests
 * from simply running the demo's executable for a limited duration.
 *
 * @param[in] lcm The LCM subsystem to be used by the simulation. It is used to
 * publish messages the Drake Visualizer.
 *
 * @param[out] demo_diagram A pointer to where the manufactured
 * systems::Diagram is to be stored. This is the systems::Diagram that's being
 * simulated by the returned systems::Simulator. Ownership of this
 * systems::Diagram is transferred to the calling method since it cannot be
 * owned by the returned systems::Simulator.
 *
 * @return The systems::Simulator that implements the Toyota HSRb passive demo.
 */
std::unique_ptr<systems::Simulator<double>> CreateSimulation(
    lcm::DrakeLcmInterface* lcm,
    std::unique_ptr<systems::Diagram<double>>* demo_diagram);

}  // namespace toyota_hsrb
}  // namespace examples
}  // namespace drake
