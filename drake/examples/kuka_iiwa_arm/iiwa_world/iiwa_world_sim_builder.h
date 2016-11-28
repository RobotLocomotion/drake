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

/// A helper class to construct and run KUKA iiwa World simulations; i.e.
/// a Simulation with a KUKA iiwa robot arm and various objects for
/// it to manipulate.
/// @tparam T must be a valid Eigen ScalarType.
///
/// Instantiated templates for the following ScalarTypes are provided:
/// - double
///
template <typename T>
class IiwaWorldSimBuilder {
 public:
  IiwaWorldSimBuilder();

  ~IiwaWorldSimBuilder();

  /// Adds a fixed object specified by its name @p object_name to the
  /// `RigidBodyTree` at the pose specified by the position @p xyz and
  /// orientation @p rpy.
  ///
  /// @return model_instance_id of the object that is added.
  int AddFixedObject(const std::string& object_name, const Eigen::Vector3d& xyz,
                     const Eigen::Vector3d& rpy = Eigen::Vector3d::Zero());

  /// Adds a floating object specified by its name @p object_name to the
  /// `RigidBodyTree` at the pose specified by the position @p xyz and
  /// orientation @p rpy.
  ///
  /// @return model_instance_id of the object that is added.
  int AddFloatingObject(const std::string& object_name,
                        const Eigen::Vector3d& xyz,
                        const Eigen::Vector3d& rpy = Eigen::Vector3d::Zero());

  /// Adds an object specified by its name @p object_name to the
  /// `RigidBodyTree` at a pose specified by the position @p xyz and
  /// orientation @p rpy.
  ///
  /// @return model_instance_id of the object that is added.
  int AddObjectToFrame(
      const std::string& object_name, const Eigen::Vector3d& xyz,
      const Eigen::Vector3d& rpy, std::shared_ptr<RigidBodyFrame> weld_to_frame,
      const drake::multibody::joints::FloatingBaseType floating_base_type =
          drake::multibody::joints::kFixed);

  ///  Adds a flat terrain to the simulation.
  void AddGround();

  /// Builds a diagram composed of a `RigidBodyPlant` and `DrakeVisualizer`
  /// and returns it. The output of the RigidBodyPlant containing the
  /// plant's generalized state is connected to the input of the
  /// DrakeVisualizer.
  std::unique_ptr<systems::Diagram<T>> Build();

  // TODO(naveenoid): Remove this method once issue #4191 is addressed.
  /// Sets the zero configuration of the plant.
  void SetZeroConfiguration(systems::Simulator<T>* simulator,
                            const systems::Diagram<T>* demo_diagram,
                            const systems::Diagram<T>* plant_diagram);

  // We are neither copyable nor moveable.
  IiwaWorldSimBuilder(const IiwaWorldSimBuilder<T>& other) = delete;
  IiwaWorldSimBuilder& operator=(const IiwaWorldSimBuilder<T>& other) = delete;

  /// Sets the parameters related to the penetration and friction throughout the
  /// world.
  void SetPenetrationContactParameters(double penetration_stiffness,
                                       double penetration_damping,
                                       double contact_friction);

  /// Allows the addition of objects to the iiwa World described by
  /// @p object_name coupled with URDF/SDF paths in @p urdf_sdf_path.
  void AddObjectUrdf(const std::string& object_name,
                     const std::string& urdf_sdf_path);

  /// Returns the size of the input port for the plant being built in the
  /// diagram.
  int GetPlantInputSize(void);

 private:
  // For both building and simulation.
  std::unique_ptr<RigidBodyTree<T>> rigid_body_tree_{
      std::make_unique<RigidBodyTree<T>>()};
  lcm::DrakeLcm lcm_;
  systems::RigidBodyPlant<T>* plant_{nullptr};

  // Maps between objects loadable in the simulation and a convenient string
  // name.
  std::map<std::string, std::string> object_urdf_map_;
  bool built_{false};

  double penetration_stiffness_{3000.0};
  double penetration_damping_{1.0};
  double contact_friction_{1.0};
};

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
