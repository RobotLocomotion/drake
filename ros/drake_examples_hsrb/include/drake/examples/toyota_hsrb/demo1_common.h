#include <memory>

#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"

namespace drake {
namespace examples {
namespace toyota_hsrb {

std::unique_ptr<systems::Simulator<double>> CreateSimulation(lcm::DrakeLcm* lcm,
    std::unique_ptr<systems::Diagram<double>>* demo_diagram);

}  // namespace toyota_hsrb
}  // namespace examples
}  // namespace drake