#pragma once

#include <map>
#include <string>

#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

/// A helper class to construct and run KUKA IIWA World simulations; i.e.
/// Simulations setting up the KUKA IIWA robot arm and various objects for
/// manipulation.
/// @tparam T must be a valid Eigen ScalarType.
///
/// Instantiated templates for the following ScalarTypes are provided:
/// - double
///
template <typename T>
class IiwaWorldSimulator {
 public:
  /// A constructor configuring this object to use DrakeLcm, which encapsulates
  /// a _real_ LCM instance
  IiwaWorldSimulator();

  ~IiwaWorldSimulator();

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

  /// Build the Diagram and initialize the Simulator.  No further changes to
  /// the diagram may occur after this has been called.
  /// @pre Start() has NOT been called.
  void Build();

  /// Advance simulated time by the given @p time_step increment in seconds.
  void StepBy(const T& time_step);

  /// Simulate until the time given by @p final_time
  void StepTo(const T& final_time);

  // We are neither copyable nor moveable.
  IiwaWorldSimulator(const IiwaWorldSimulator<T>& other) = delete;
  IiwaWorldSimulator& operator=(const IiwaWorldSimulator<T>& other) = delete;

  void SetPenetrationContactParameters(double penetration_stiffness,
                                       double penetration_damping,
                                       double contact_friction);

 private:
  // For both building and simulation.
  std::unique_ptr<RigidBodyTree<T>> rigid_body_tree_{
      std::make_unique<RigidBodyTree<T>>()};
  lcm::DrakeLcm lcm_;

  // For building.
  std::unique_ptr<systems::DiagramBuilder<T>> builder_{
      std::make_unique<systems::DiagramBuilder<T>>()};
  std::map<std::string, std::string> object_urdf_map_;
  int next_object_number_{0};
  bool started_{false};
  bool table_loaded_{false};

  // Map between objects loadable in the simulation and a convinient string
  // name.
  std::map<std::string, std::string> object_name_map_{
      {"iiwa", "/examples/kuka_iiwa_arm/urdf/iiwa14.urdf"},
      {"table",
       "/examples/kuka_iiwa_arm/models/table/extra_heavy_duty_table.sdf"},
      {"cylinder",
       "/examples/kuka_iiwa_arm/models/objects/simple_cylinder.urdf"},
      {"cuboid", "/examples/kuka_iiwa_arm/models/objects/simple_cuboid.urdf"}};

  // For simulation.
  std::unique_ptr<systems::Diagram<T>> diagram_;
  std::unique_ptr<systems::Simulator<T>> simulator_;

  double penetration_stiffness_{3000.0};
  double penetration_damping_{1.0};
  double contact_friction_{1.0};
};

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
