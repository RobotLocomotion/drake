#include "drake/multibody/plant/test/iiwa7_model.h"

namespace drake {
namespace multibody {
namespace test {

void VisualizeRobotModel() {
  RobotModel<double> model(DiscreteContactApproximation::kSimilar,
                           ContactModel::kHydroelasticWithFallback,
                           true /* add viz */);
  // Visualize the contact configuration.
  const VectorX<double> x0 =
      (VectorX<double>(14) << 0, 1.17, 0, -1.33, 0, 0.58, 0,  // q
       0, 0, 0, 0, 0, 0, 0                                    // v
       )
          .finished();
  model.SetRobotState(RobotModel<double>::RobotStateWithOneContactStiction());

  model.ForcedPublish();
  common::MaybePauseForUser();
}

}  // namespace test
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::test::RobotModel)
