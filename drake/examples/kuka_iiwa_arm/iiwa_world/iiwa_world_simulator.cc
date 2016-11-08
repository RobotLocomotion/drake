#include <gflags/gflags.h>

#include "drake/common/drake_path.h"
#include "drake/common/text_logging_gflags.h"

#include "drake/math/roll_pitch_yaw.h"
#include "drake/systems/plants/RigidBodyFrame.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/systems/plants/RigidBodySystem.h"

#include "drake/common/drake_export.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_world/iiwa_world_simulator.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/primitives/constant_vector_source.h"
#include "drake/systems/framework/primitives/multiplexer.h"
#include "drake/systems/plants/rigid_body_plant/rigid_body_plant.h"

#include "drake/systems/plants/parser_model_instance_id_table.h"
#include "drake/systems/plants/parser_sdf.h"
#include "drake/systems/plants/parser_urdf.h"
#include "drake/systems/plants/rigid_body_plant/drake_visualizer.h"

namespace drake {

using lcm::DrakeLcm;
using systems::ConstantVectorSource;
using systems::Context;
using systems::ContinuousState;
using systems::Diagram;
using systems::DiagramBuilder;
using systems::DrakeVisualizer;
using systems::RigidBodyPlant;
using systems::Simulator;
using systems::VectorBase;
using systems::plants::joints::kFixed;

using parsers::ModelInstanceIdTable;
using parsers::urdf::AddModelInstanceFromUrdfFile;

namespace examples {
namespace kuka_iiwa_arm {

template <typename T>
IiwaWorldSimulator<T>::IiwaWorldSimulator() {}

template <typename T>
IiwaWorldSimulator<T>::~IiwaWorldSimulator() {}

template <typename T>
int IiwaWorldSimulator<T>::AddObjectFixedToTable(Eigen::Vector3d xyz,
                                                 Eigen::Vector3d rpy,
                                                 std::string object_name) {
  DRAKE_DEMAND(!started_);

  Eigen::Vector3d position_of_robot_base(-0.2, -0.2, 0.736);
  auto weld_to_frame = std::allocate_shared<RigidBodyFrame>(
      Eigen::aligned_allocator<RigidBodyFrame>(),
      "tabletop",
      GetTableBody(),
      position_of_robot_base,
      Eigen::Vector3d::Zero() /* rpy */);
//    parsers::ModelInstanceIdTable table = drake::parsers::urdf::AddModelInstanceFromUrdfFile(
//      drake::GetDrakePath() + object_name_map_["iiwa"],
//      kFixed, weld_to_frame, rigid_body_tree_.get());

//  return 0;//table.begin()->second;
  return AddObjectToFrame(xyz, rpy, object_name, weld_to_frame);
}

template <typename T>
RigidBody* IiwaWorldSimulator<T>::GetTableBody() {
   if(!table_loaded_) {
    AddObjectFixedToWorld(Eigen::Vector3d::Zero(),
                          Eigen::Vector3d::Zero(), "table");
  }
  return rigid_body_tree_->FindBody("link", "extra_heavy_duty_table");
}

template <typename T>
int IiwaWorldSimulator<T>::AddObjectFixedToWorld(Eigen::Vector3d xyz,
                                                 Eigen::Vector3d rpy,
                                                 std::string object_name) {
  DRAKE_DEMAND(!started_);


  Eigen::Isometry3d T_model_world_to_drake_world;
  T_model_world_to_drake_world.matrix() << drake::math::rpy2rotmat(rpy), xyz, 0,
      0, 0, 1;

  auto weld_to_frame = std::allocate_shared<RigidBodyFrame>(
      Eigen::aligned_allocator<RigidBodyFrame>(), "world",
      nullptr,  // not used since the robot is attached to the world
      T_model_world_to_drake_world);

  return AddObjectToFrame(xyz, rpy, object_name, weld_to_frame);
}

template <typename T>
int IiwaWorldSimulator<T>::AddObjectToFrame(Eigen::Vector3d xyz,
                                            Eigen::Vector3d rpy,
                                            std::string object_name,
                                            std::shared_ptr<RigidBodyFrame>
                                            weld_to_frame) {
  DRAKE_DEMAND(!started_);
  std::size_t extension_location =
      object_name_map_[object_name].find_last_of(".");
  if (extension_location >= object_name_map_[object_name].size()) {
    return -1;
  }
  std::string extension =
      object_name_map_[object_name].substr(extension_location + 1);

  parsers::ModelInstanceIdTable table;
  if (extension.compare("urdf") == 0) {
    std::cout << "\n" << object_name << " is a URDF\n";
    table = drake::parsers::urdf::AddModelInstanceFromUrdfFile(
        drake::GetDrakePath() + object_name_map_[object_name],
        systems::plants::joints::kFixed, weld_to_frame, rigid_body_tree_.get());

  } else if (extension.compare("sdf") == 0) {
    std::cout << "\n" << object_name << " is a SDF\n";
    table = drake::parsers::sdf::AddModelInstancesFromSdfFile(
        drake::GetDrakePath() + object_name_map_[object_name],
        systems::plants::joints::kFixed, weld_to_frame, rigid_body_tree_.get());
  } else {
    return -1;
  }

  const int model_instance_id = table.begin()->second;
  return model_instance_id;


}

template <typename T>
void IiwaWorldSimulator<T>::Build() {
  DRAKE_DEMAND(!started_);

  std::cout << "Trying to build the system\n";
  // if (!drake_visualizer_inputs_.empty()) {
  // Arithmetic for DrakeVisualizer input sizing.  We have systems that output
  // an Euler floating joint state.  We want to mux them together to feed
  // DrakeVisualizer.  We stack them up in joint order as the position input
  // to the publisher, and then also need to feed zeros for all of the joint
  // velocities.
  rigid_body_tree_->compile();

  std::cout << "Building the diagram\n";

  std::cout << "Tree has : ";
  std::cout << rigid_body_tree_->get_num_model_instances()
            << " model instances.\n";

  std::cout << "Tree has : ";
  std::cout << rigid_body_tree_->get_num_bodies() << " bodies.\n";

  std::cout << "Tree has : ";
  std::cout << rigid_body_tree_->get_num_positions() << " model positions.\n";

  //  auto plant = builder_.template AddSystem<systems::RigidBodyPlant<T>>();
  auto plant = builder_->template AddSystem<systems::RigidBodyPlant<T>>(
      move(rigid_body_tree_));
  plant->set_contact_parameters(3000 /* penetration stiffness */,
                                1.0 /* penetration damping */,
                                1.0 /* friction coefficient */);

  // Feed in constant inputs of zero into the RigidBodyPlant.
  VectorX<T> constant_value(plant->get_input_size());
  constant_value.setZero();

  auto const_source_ =
      builder_->template AddSystem<ConstantVectorSource<T>>(constant_value);

  // Creates and adds LCM publisher for visualization.
  auto viz_publisher_ = builder_->template AddSystem<DrakeVisualizer>(
      plant->get_rigid_body_tree(), &lcm_);

  // Connects the constant source output port to the RigidBodyPlant's input
  // port. This effectively results in the robot being uncontrolled.
  builder_->Connect(const_source_->get_output_port(), plant->get_input_port(0));

  // Connects to publisher for visualization.
  builder_->Connect(plant->get_output_port(0),
                    viz_publisher_->get_input_port(0));

  builder_->ExportOutput(plant->get_output_port(0));

  diagram_ = builder_->Build();

  std::cout << " Finished adding system";
  std::cout << " Input size : " << plant->get_input_size();

  std::cout << " Diagram built\n";
  simulator_ = std::make_unique<systems::Simulator<T>>(*diagram_);
  //  lcm_->StartReceiveThread();
  Context<T>* plant_context = diagram_->GetMutableSubsystemContext(
      simulator_->get_mutable_context(), plant);
  plant->SetZeroConfiguration(plant_context);

  simulator_->Initialize();

  std::cout << "Simulator initialised\n";

  started_ = true;
}

template <typename T>
const systems::System<T>& IiwaWorldSimulator<T>::GetDiagramSystemByName(
    std::string name) const {
  DRAKE_DEMAND(started_);
  // Ask the diagram.
  const systems::System<T>* result{nullptr};
  for (const systems::System<T>* system : diagram_->GetSystems()) {
    if (system->get_name() == name) {
      DRAKE_THROW_UNLESS(!result);
      result = system;
    }
  }
  DRAKE_THROW_UNLESS(result);
  return *result;
}

template <typename T>
void IiwaWorldSimulator<T>::StepBy(const T& time_step) {
  const T time = simulator_->get_context().get_time();
  SPDLOG_TRACE(drake::log(), "Time is now {}", time);
  simulator_->StepTo(time + time_step);
}

template <typename T>
int IiwaWorldSimulator<T>::allocate_object_number() {
  DRAKE_DEMAND(!started_);
  return next_object_number_++;
}

// template <typename T>
// std::string IiwaWorldSimulator<T>::ExtractExtension(std::string filename) {
//  std::size_t extension_location = filename.find_last_of(".");
//  return filename.substr(extension_location+1);
//}

template class DRAKE_EXPORT IiwaWorldSimulator<double>;
}
}
}