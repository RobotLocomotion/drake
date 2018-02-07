#include "drake/multibody/benchmarks/kuka_iiwa_robot/make_kuka_iiwa_plant.h"

#include "drake/multibody/benchmarks/kuka_iiwa_robot/make_kuka_iiwa_model.h"

namespace drake {
namespace multibody {
namespace benchmarks {
namespace kuka_iiwa_robot {

using drake::multibody::multibody_plant::MultibodyPlant;

std::unique_ptr<MultibodyPlant<double>> MakeKukaIiwaPlant() {
  auto plant = std::make_unique<MultibodyPlant<double>>(MakeKukaIiwaModel());
  // Communicate the plant that state, ports and other system specifications can
  // be declared.
  plant->Finalize();
  return plant;
}

}  // namespace kuka_iiwa_robot
}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake
