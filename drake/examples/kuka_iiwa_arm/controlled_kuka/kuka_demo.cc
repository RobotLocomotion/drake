#include <iostream>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/primitives/constant_vector_source.h"
#include "drake/systems/framework/primitives/gain.h"
#include "drake/systems/framework/primitives/pid_controller2.h"
#include "drake/systems/plants/parser_urdf.h"
#include "drake/systems/plants/rigid_body_system/rigid_body_plant.h"

using Eigen::VectorXd;
using std::make_unique;
using std::move;
using std::unique_ptr;

namespace drake {
namespace systems {
namespace plants {
namespace rigid_body_system {
namespace test {
namespace {

template<typename T>
class KukaDemo : public Diagram<T> {
 public:
  // Pass through to SpringMassSystem, except add sample rate in samples/s.
  KukaDemo() {
    // Instantiates an MBD model of the world.
    auto mbd_world = make_unique<RigidBodyTree>();
    drake::parsers::urdf::AddModelInstanceFromUrdfFile(
        drake::GetDrakePath() + "/examples/kuka_iiwa_arm/urdf/iiwa14.urdf",
        DrakeJoint::FIXED, nullptr /* weld to frame */, mbd_world.get());

    // Adds the ground.
    double kBoxWidth = 3;
    double kBoxDepth = 0.2;
    DrakeShapes::Box geom(Eigen::Vector3d(kBoxWidth, kBoxWidth, kBoxDepth));
    Eigen::Isometry3d T_element_to_link = Eigen::Isometry3d::Identity();
    // top of the box is at z = 0.
    T_element_to_link.translation() << 0, 0, -kBoxDepth / 2.0;

    RigidBody& world = mbd_world->world();
    Eigen::Vector4d color;
    color << 0.9297, 0.7930, 0.6758, 1;
    world.AddVisualElement(
        DrakeShapes::VisualElement(geom, T_element_to_link, color));
    mbd_world->addCollisionElement(
        RigidBodyCollisionElement(geom, T_element_to_link, &world), world,
        "terrain");
    mbd_world->updateStaticCollisionElements();

    // Instantiates a RigidBodyPlant from an MBD model of the world.
    plant_ = make_unique<RigidBodyPlant<T>>(move(mbd_world));
    torques_ = make_unique<ConstantVectorSource<T>>(
        VectorX<T>::Zero(plant_->get_num_actuators()));

    plant_->penetration_stiffness_ = 3000.0;

    DiagramBuilder<T> builder;
    builder.Connect(torques_->get_output_port(0),
                    plant_->get_input_port(0));
    builder.ExportOutput(plant_->get_output_port(0));
    builder.BuildInto(this);
  }

  const RigidBodyPlant<T>& get_kuka_plant() const { return *plant_; }

 private:
  // Publish t q u to standard output.
  void DoPublish(const ContextBase<double>& context) const override {
#if 0
    cout << context.get_time() << " "
         << get_position(context) << " "
         << get_velocity(context) << " "
         << get_conservative_work(context) << endl;
#endif
  }

 private:
  std::unique_ptr<RigidBodyPlant<T>> plant_;
  std::unique_ptr<PidController<T>> controller_;
  std::unique_ptr<Gain<T>> inverter_;
  std::unique_ptr<ConstantVectorSource<T>> torques_;
};

GTEST_TEST(KukaDemo, Testing) {

  KukaDemo<double> model;
  Simulator<double> simulator(model);  // Use default Context.

  // Zeroes the state.
  model.get_kuka_plant().ObtainZeroConfiguration(
      simulator.get_mutable_context());

  simulator.request_initial_step_size_attempt(0.002);

  // Take all the defaults.
  simulator.Initialize();

  EXPECT_TRUE(simulator.get_integrator_type_in_use() ==
      IntegratorType::RungeKutta2);

  // Simulate for 1 seconds.
  simulator.StepTo(1.);

  const auto& context = simulator.get_context();
  EXPECT_EQ(context.get_time(), 1.);  // Should be exact.

  EXPECT_EQ(simulator.get_num_steps_taken(), 500);
  EXPECT_EQ(simulator.get_num_samples_taken(), 0);
  EXPECT_LE(simulator.get_smallest_step_size_taken(),
            simulator.get_largest_step_size_taken());
}


}  // namespace
}  // namespace test
}  // namespace rigid_body_system
}  // namespace plants
}  // namespace systems
}  // namespace drake
