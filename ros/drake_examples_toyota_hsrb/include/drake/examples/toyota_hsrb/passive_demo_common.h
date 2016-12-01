#include <memory>

#include "drake/lcm/drake_lcm.h"
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
 * from simply running the demo's exectuable for a limited duration.
 */
std::unique_ptr<systems::Simulator<double>> CreateSimulation(lcm::DrakeLcm* lcm,
    std::unique_ptr<systems::Diagram<double>>* demo_diagram);

}  // namespace toyota_hsrb
}  // namespace examples
}  // namespace drake
