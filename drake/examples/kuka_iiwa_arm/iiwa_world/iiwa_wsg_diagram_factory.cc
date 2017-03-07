#include "drake/examples/kuka_iiwa_arm/iiwa_world/iiwa_wsg_diagram_factory.h"

#include <map>
#include <set>
#include <string>
#include <utility>
#include "drake/util/drakeGeometryUtil.h"

#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_world/world_sim_tree_builder.h"
#include "drake/examples/kuka_iiwa_arm/oracular_state_estimator.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"
#include "drake/systems/controllers/pid_controller.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/matrix_gain.h"

namespace drake {

using systems::Context;
using systems::Diagram;
using systems::DiagramBuilder;
using systems::InputPortDescriptor;
using systems::OutputPortDescriptor;
using systems::RigidBodyPlant;

namespace examples {
namespace kuka_iiwa_arm {

template <typename T>
IiwaAndWsgPlantWithStateEstimator<T>::IiwaAndWsgPlantWithStateEstimator(
    std::unique_ptr<RigidBodyPlant<T>> combined_plant,
    const ModelInstanceInfo<T>& iiwa_info, 
    const ModelInstanceInfo<T>& wsg_info,
    const ModelInstanceInfo<T>& box_info) {
  this->set_name("IiwaAndWsgPlantWithStateEstimator");
  DiagramBuilder<T> builder;

  plant_ =
      builder.template AddSystem<RigidBodyPlant<T>>(std::move(combined_plant));
  const auto& iiwa_input_port =
      plant_->model_instance_actuator_command_input_port(iiwa_info.instance_id);
  const auto& iiwa_output_port =
      plant_->model_instance_state_output_port(iiwa_info.instance_id);

  const auto& wsg_input_port =
      plant_->model_instance_actuator_command_input_port(wsg_info.instance_id);
  const auto& wsg_output_port =
      plant_->model_instance_state_output_port(wsg_info.instance_id);

  VectorX<double> iiwa_kp, iiwa_kd, iiwa_ki;
  SetPositionControlledIiwaGains(&iiwa_kp, &iiwa_ki, &iiwa_kd);
  // Uses integral gains to deal with the added mass from the grasped object.
  iiwa_ki << 1, 1, 1, 1, 1, 1, 1;

  // Exposing feedforward acceleration. Should help with more dynamic motions.
  iiwa_controller =
      builder.template AddSystem<systems::InverseDynamicsController<T>>(
          iiwa_info.model_path, iiwa_info.world_offset, iiwa_kp, iiwa_ki,
          iiwa_kd, true /* with feedforward acceleration */);

  // Sets a zero configuration and computes spatial inertia for the gripper
  // as well as the pose of the end effector link of iiwa using the world
  // tree.
  const RigidBodyTree<T>& world_tree = plant_->get_rigid_body_tree();
  KinematicsCache<T> world_cache = world_tree.CreateKinematicsCache();
  world_cache.initialize(world_tree.getZeroConfiguration());
  world_tree.doKinematics(world_cache);

  const RigidBody<T>* end_effector = world_tree.FindBody("iiwa_link_7");
  const Isometry3<T> X_WEE =
      world_tree.CalcBodyPoseInWorldFrame(world_cache, *end_effector);

  // The inertia of the added gripper is lumped into the last link of the
  // controller's iiwa arm model. This is motivated by the fact that the
  // gripper inertia is relatively large compared to the last couple links
  // in the iiwa arm model. And to completely rely on using feedback to cope
  // with added inertia, we need to either rely on larger gains (which will
  // cause simulation to explode without the gripper), or wait longer for
  // the integrator to kick in.

  // Computes the lumped inertia for the gripper.
  const std::set<int> gripper_instance_set = {wsg_info.instance_id};
  const Matrix6<T> lumped_gripper_inertia_W =
      world_tree.LumpedSpatialInertiaInWorldFrame(world_cache,
                                                  gripper_instance_set);
  // Transfer it to the last iiwa link's body frame.
  Matrix6<T> lumped_gripper_inertia_EE =
      transformSpatialInertia(X_WEE.inverse(), lumped_gripper_inertia_W);
  lumped_gripper_inertia_EE += end_effector->get_spatial_inertia();

  // Changes the controller's iiwa end effector's link to the lumped inertia.
  RigidBody<T>* controller_ee =
      iiwa_controller->get_robot_for_control().FindBody("iiwa_link_7");
  controller_ee->set_spatial_inertia(lumped_gripper_inertia_EE);

  // Connect iiwa controller and robot.
  builder.Connect(iiwa_output_port,
                  iiwa_controller->get_input_port_estimated_state());
  builder.Connect(iiwa_controller->get_output_port_control(), iiwa_input_port);

  // Export iiwa's desired state input, and state output.
  builder.ExportInput(iiwa_controller->get_input_port_desired_state());
  builder.ExportInput(iiwa_controller->get_input_port_desired_acceleration());
  builder.ExportOutput(iiwa_output_port);

  // Sets up the WSG gripper part.
  const std::map<std::string, int> index_map =
      world_tree.computePositionNameToIndexMap();
  const int left_finger_position_index =
      index_map.at("left_finger_sliding_joint");
  const int position_index = plant_->FindInstancePositionIndexFromWorldIndex(
      wsg_info.instance_id, left_finger_position_index);
  const int velocity_index =
      position_index + plant_->get_num_positions(wsg_info.instance_id);

  Eigen::MatrixXd feedback_matrix = Eigen::MatrixXd::Zero(
      2 * plant_->get_num_actuators(wsg_info.instance_id),
      2 * plant_->get_num_positions(wsg_info.instance_id));
  feedback_matrix(0, position_index) = 1.;
  feedback_matrix(1, velocity_index) = 1.;
  std::unique_ptr<systems::MatrixGain<T>> feedback_selector =
      std::make_unique<systems::MatrixGain<T>>(feedback_matrix);

  // TODO(sam.creasey) The choice of constants below is completely
  // arbitrary and may not match the performance of the actual
  // gripper.
  const T wsg_kp = 300.0;  // This seems very high, for some grasps
  // it's actually in the right power of
  // two.  We'll need to revisit this once
  // we're using the force command sent to
  // the gripper properly.
  const T wsg_ki = 0.0;
  const T wsg_kd = 5.0;
  const VectorX<T> wsg_v = VectorX<T>::Ones(wsg_input_port.size());

  wsg_controller = builder.template AddSystem<systems::PidController<T>>(
      std::move(feedback_selector), wsg_v * wsg_kp, wsg_v * wsg_ki,
      wsg_v * wsg_kd);

  // Connects WSG and controller.
  builder.Connect(wsg_output_port,
                  wsg_controller->get_input_port_estimated_state());
  builder.Connect(wsg_controller->get_output_port_control(), wsg_input_port);

  //  Export wsg's desired state input, and state output.
  builder.ExportInput(wsg_controller->get_input_port_desired_state());
  builder.ExportOutput(wsg_output_port);

  builder.ExportOutput(plant_->get_output_port(0));

  // Sets up a "state estimator" for iiwa that generates
  // bot_core::robot_state_t messages.
  iiwa_state_est = builder.template AddSystem<OracularStateEstimation<T>>(
      iiwa_controller->get_robot_for_control(),
      iiwa_controller->get_robot_for_control().get_body(1));
  builder.Connect(iiwa_output_port, iiwa_state_est->get_input_port_state());
  builder.ExportOutput(iiwa_state_est->get_output_port_msg());

  // Sets up a "state estimator" for the box that generates
  // bot_core::robot_state_t messages.
  // Make a box RBT for the fake state estimator.
  object_ = std::make_unique<RigidBodyTree<T>>();
  parsers::urdf::AddModelInstanceFromUrdfFile(
      box_info.model_path, multibody::joints::kQuaternion,
      box_info.world_offset, object_.get());
  box_state_est = builder.template AddSystem<OracularStateEstimation<T>>(
      *object_, object_->get_body(1));
  builder.Connect(
      plant_->model_instance_state_output_port(box_info.instance_id),
      box_state_est->get_input_port_state());
  builder.ExportOutput(box_state_est->get_output_port_msg());

  builder.BuildInto(this);
}
template class IiwaAndWsgPlantWithStateEstimator<double>;

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
