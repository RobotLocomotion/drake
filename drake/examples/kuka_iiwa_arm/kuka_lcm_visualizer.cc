
#include "drake/examples/kuka_iiwa_arm/iiwa_simulation.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_status.h"
#include "drake/Path.h"
#include "drake/systems/LCMSystem.h"
#include "drake/systems/plants/BotVisualizer.h"
#include "drake/systems/plants/RigidBodyTree.h"

#include "lcmtypes/drake/lcmt_iiwa_status.hpp"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

// This class exists to keep the LCM messages which are passing
// through BotVisualizer from being sent back out via LCM again.
template <template <typename> class Vector>
class SinkSystem {
 public:
  template <typename ScalarType>
  using StateVector = Drake::NullVector<ScalarType>;
  template <typename ScalarType>
  using InputVector = Vector<ScalarType>;
  template <typename ScalarType>
  using OutputVector = Drake::NullVector<ScalarType>;

  SinkSystem() {}

  template <typename ScalarType>
  StateVector<ScalarType> dynamics(const double &t,
                                   const StateVector<ScalarType> &x,
                                   const InputVector<ScalarType> &u) const {
    return StateVector<ScalarType>();
  }

  template <typename ScalarType>
  OutputVector<ScalarType> output(const double &t,
                                  const StateVector<ScalarType> &x,
                                  const InputVector<ScalarType> &u) const {
    return OutputVector<ScalarType>();
  }

  bool isTimeVarying() const { return false; }
};

int do_main(int argc, const char* argv[]) {
  std::shared_ptr<lcm::LCM> lcm = std::make_shared<lcm::LCM>();

  const std::shared_ptr<RigidBodyTree>& tree =
      CreateKukaIiwaSystem()->getRigidBodyTree();

  auto visualizer =
      std::make_shared<Drake::BotVisualizer<IiwaStatus>>(lcm, tree);
  auto sink = std::make_shared<SinkSystem<IiwaStatus>>();
  auto sys = Drake::cascade(visualizer, sink);
  Drake::runLCM(sys, lcm, 0, 1e9);
  return 0;
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake


int main(int argc, const char* argv[]) {
  return drake::examples::kuka_iiwa_arm::do_main(argc, argv);
}
