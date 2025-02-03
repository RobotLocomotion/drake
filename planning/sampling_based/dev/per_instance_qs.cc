#include "planning/per_instance_qs.h"

namespace anzu {
namespace planning {

using drake::multibody::MultibodyPlant;

void SetInstanceQsInFullQ(
    const MultibodyPlant<double>& plant,
    const PerInstanceQs& per_instance_qs,
    drake::EigenPtr<Eigen::VectorXd> full_q) {
  DRAKE_THROW_UNLESS(full_q != nullptr);
  DRAKE_THROW_UNLESS(plant.num_positions() == full_q->size());
  for (const auto& instance_q : per_instance_qs) {
    plant.SetPositionsInArray(instance_q.first, instance_q.second, full_q);
  }
}

}  // namespace planning
}  // namespace anzu
