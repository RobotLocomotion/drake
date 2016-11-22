#pragma once

#include <map>
#include <string>

#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

/// A helper class to construct and run KUKA IIWA World simulations; i.e.
/// Simulation setting up the KUKA IIWA robot arm and various objects for
/// to manipulate.
/// @tparam T must be a valid Eigen ScalarType.
///
/// Instantiated templates for the following ScalarTypes are provided:
/// - double
///
template <typename T>
class IiwaWorldSimBuilder {
 public:
  /// A constructor configuring this object to use DrakeLcm, which encapsulates
  /// a _real_ LCM instance
  IiwaWorldSimBuilder();

  ~IiwaWorldSimBuilder();

  /// Adds a fixed object specified by its name `object_name` to the
  /// `RigidBodyTree^` at a pose specified by the position `xyz` and
  /// orientation specified by `rpy`. Wraps AddObjectToFrame.
  int AddObjectFixedToWorld(Eigen::Vector3d xyz, Eigen::Vector3d rpy,
                            std::string object_name);

  /// Adds a floating object specified by its name `object_name` to the
  /// `RigidBodyTree^` at a pose specified by the position `xyz` and
  /// orientation specified by `rpy`. Wraps AddObjectToFrame.
  int AddObjectFloatingToWorld(Eigen::Vector3d xyz, Eigen::Vector3d rpy,
                               std::string object_name);

  /// Adds an object specified by its name `object_name` to the
  /// `RigidBodyTree^` at a pose specified by the position `xyz` and
  /// orientation specified by `rpy`
  int AddObjectToFrame(
      Eigen::Vector3d xyz, Eigen::Vector3d rpy, std::string object_name,
      std::shared_ptr<RigidBodyFrame> weld_to_frame,
      const drake::multibody::joints::FloatingBaseType floating_base_type =
          drake::multibody::joints::kFixed);

  ///  Adds a flat terrain to the ground.
  ///  Wraps drake::multibody::AddFlatTerrainToWorld
  void AddGroundToTree();

  /// Builds a diagram and assigned it to a Simulator
  std::unique_ptr<systems::Diagram<T>> Build();
  //
  //  /// Set the zero configuration
  void SetZeroConfiguration(systems::Simulator<T>* simulator,
                            const systems::Diagram<T>* diagram);

  // We are neither copyable nor moveable.
  IiwaWorldSimBuilder(const IiwaWorldSimBuilder<T>& other) = delete;
  IiwaWorldSimBuilder& operator=(const IiwaWorldSimBuilder<T>& other) = delete;

  /// Set the parameters related to the penetration and friction throughout the
  /// world.
  void SetPenetrationContactParameters(double penetration_stiffness,
                                       double penetration_damping,
                                       double contact_friction);

  void AddObjectUrdf(const std::string& object_name,
                     const std::string& urdf_path);

  /// Returns the size of the input port for the plant being built in the
  /// diagram.
  int GetPlantInputSize(void);
 private:
  // For both building and simulation.
  std::unique_ptr<RigidBodyTree<T>> rigid_body_tree_{
      std::make_unique<RigidBodyTree<T>>()};
  lcm::DrakeLcm lcm_;
  systems::RigidBodyPlant<T>* plant_{nullptr};

  // Map between objects loadable in the simulation and a convenient string
  // name.
  std::map<std::string, std::string> object_urdf_map_;
  bool started_{false};

  double penetration_stiffness_{3000.0};
  double penetration_damping_{1.0};
  double contact_friction_{1.0};
};

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
