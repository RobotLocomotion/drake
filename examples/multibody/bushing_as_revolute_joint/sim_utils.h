#include "drake/lcmt_viewer_draw.hpp"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/lcm/drake_lcm.h"

namespace drake {
namespace examples {
namespace multibody {
namespace bushing_as_revolute_joint {

void PublishFramesToLcm(const std::string& channel_name,
                        const std::vector<math::RigidTransformd>& poses,
                        const std::vector<std::string>& names,
                        drake::lcm::DrakeLcmInterface* dlcm);

/// Publishes pre-defined body frames once.
void PublishBodyFrames(const drake::systems::Context<double>& plant_context,
                       const drake::multibody::MultibodyPlant<double>& plant,
                       drake::lcm::DrakeLcm* lcm);

/// A system that publishes frames at a specified period.
class FrameViz final : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FrameViz)

  FrameViz(const drake::multibody::MultibodyPlant<double>& plant,
           drake::lcm::DrakeLcm* lcm, double period, bool frames_input = false);

 private:
  drake::systems::EventStatus PublishFramePose(
      const drake::systems::Context<double>& context) const;

  const drake::multibody::MultibodyPlant<double>& plant_;
  std::unique_ptr<drake::systems::Context<double>> plant_context_;
  drake::lcm::DrakeLcm* lcm_;
  bool frames_input_{false};
};

}  // bushing_as_revolute_joint
}  // namespace multibody
}  // namespace examples
}  // namespace drake