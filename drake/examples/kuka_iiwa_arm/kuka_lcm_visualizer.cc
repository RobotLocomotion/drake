
#include "drake/Path.h"
#include "drake/systems/LCMSystem.h"
#include "drake/systems/plants/BotVisualizer.h"
#include "drake/systems/plants/RigidBodyTree.h"

#include "iiwa_status.h"

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

  auto tree = std::make_shared<RigidBodyTree>(
      Drake::getDrakePath() + "/examples/kuka_iiwa_arm/urdf/iiwa14.urdf",
      DrakeJoint::FIXED);

    // Adds the ground.
  {
    double box_width = 3;
    double box_depth = 0.2;
    DrakeShapes::Box geom(Eigen::Vector3d(box_width, box_width, box_depth));
    Eigen::Isometry3d T_element_to_link = Eigen::Isometry3d::Identity();
    T_element_to_link.translation() << 0, 0,
        -box_depth / 2.0;  // top of the box is at z = 0
    RigidBody& world = tree->world();
    Eigen::Vector4d color;
    color << 0.9297, 0.7930, 0.6758,
        1;  // was hex2dec({'ee','cb','ad'})'/256 in matlab
    world.addVisualElement(
        DrakeShapes::VisualElement(geom, T_element_to_link, color));
    tree->addCollisionElement(
        RigidBody::CollisionElement(geom, T_element_to_link, &world), world,
        "terrain");
    tree->updateStaticCollisionElements();
  }

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
