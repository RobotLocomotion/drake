#include "drake/multibody/plant/test/robot_model.h"

#include "drake/common/test_utilities/maybe_pause_for_user.h"

namespace drake {
namespace multibody {
namespace test {

void VisualizeRobotModel() {
  // Visualize the contact configuration.
  RobotModelConfig config{
      .with_contact_geometry = true,
      .state_in_contact = true,
  };
  RobotModel<double> model(config, true /* add viz */);

  model.ForcedPublish();
  common::MaybePauseForUser();
}

}  // namespace test
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::test::RobotModel)
