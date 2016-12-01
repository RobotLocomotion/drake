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
// TODO(naveenoid): If this could be made slightly more generic, it
// could be a good wrapper/template for building world demos for LCM
// controlled robots.

/// A helper class to construct KUKA iiwa world simulations; i.e. a Simulation
/// with a KUKA iiwa robot arm and various objects for it to manipulate.
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

  /// Adds a fixed model instance specified by its name, @p model_name, to the
  /// `RigidBodyTree` at the pose specified by position @p xyz and orientation
  /// @p rpy. The model name must have been previously loaded via a call to
  /// StoreModel().
  ///
  /// @return model_instance_id of the object that is added.
  int AddFixedModelInstance(
      const std::string& model_name, const Eigen::Vector3d& xyz,
      const Eigen::Vector3d& rpy = Eigen::Vector3d::Zero());

  /// Adds a floating model instance specified by its name, @p model_name, to
  /// the `RigidBodyTree` at the pose specified by position @p xyz and
  /// orientation @p rpy. The model name must have been previously loaded via
  /// a call to StoreModel().
  ///
  /// @return model_instance_id of the object that is added.
  int AddFloatingModelInstance(
      const std::string& model_name, const Eigen::Vector3d& xyz,
      const Eigen::Vector3d& rpy = Eigen::Vector3d::Zero());

  /// Adds a model instance specified by its model name, @p model_name, to
  /// the `RigidBodyTree` at a pose specified by position @p xyz and
  /// orientation @p rpy. The model instance is connected to the existing
  /// world based on @p weld_to_frame using a floating joint of type @p
  /// floating_base_type. The model name must have been previously loaded via
  /// a call to StoreModel().
  ///
  /// @return model_instance_id of the object that is added.
  int AddModelInstanceToFrame(
      const std::string& model_name, const Eigen::Vector3d& xyz,
      const Eigen::Vector3d& rpy,
      std::shared_ptr<RigidBodyFrame> weld_to_frame,
      const drake::multibody::joints::FloatingBaseType floating_base_type =
          drake::multibody::joints::kFixed);

  /// Adds a flat terrain to the simulation.
  void AddGround();

  /// Builds a diagram composed of a `RigidBodyPlant` and `DrakeVisualizer`
  /// and returns it. The output of the RigidBodyPlant containing the
  /// plant's generalized state is connected to the input of the
  /// DrakeVisualizer. Once this method is called, this builder should be
  /// discarded.
  std::unique_ptr<systems::Diagram<T>> Build();

  // TODO(naveenoid): Remove this method once issue #4191 is addressed.
  // TODO(naveenoid): Subsystem names should be documented here once #3556 is
  // resolved.
  /// Sets the zero configuration of the plant.
  /// Note that this method assumes that @p demo_diagram is composed of a
  /// `Diagram` containing the plant and visualizer systems as built by
  /// the `Build()` method.
  ///
  /// @see Build
  void SetZeroConfiguration(systems::Simulator<T>* simulator,
                            const systems::Diagram<T>* demo_diagram);

  // We are neither copyable nor moveable.
  IiwaWorldSimBuilder(const IiwaWorldSimBuilder<T>& other) = delete;
  IiwaWorldSimBuilder& operator=(const IiwaWorldSimBuilder<T>& other) = delete;

  /// Sets the parameters related to the penetration and friction throughout
  /// the world.
  void SetPenetrationContactParameters(double penetration_stiffness,
                                       double penetration_damping,
                                       double contact_friction);

  /// Adds a model to the internal model database. Models are described by
  /// @p model_name coupled with URDF/SDF paths in @p model_path. Instances
  /// of these models can then be added to the world via the various
  /// `AddFoo()` methods provided by this class.
  ///
  /// @see AddObjectToFrame
  /// @see AddFloatingObject
  /// @see AddFixedObject
  void StoreModel(const std::string& model_name,
                  const std::string& model_path);

  /// Returns the size of the input port for the plant being built in the
  /// diagram.
  int GetPlantInputSize(void);

 private:
  std::unique_ptr<RigidBodyTree<T>> rigid_body_tree_{
      std::make_unique<RigidBodyTree<T>>()};
  lcm::DrakeLcm lcm_;
  systems::RigidBodyPlant<T>* plant_{nullptr};

  // Maps between models and their names. Instances of these models can be
  // loaded into the simulation.
  std::map<std::string, std::string> model_map_;
  bool built_{false};

  double penetration_stiffness_{3000.0};
  double penetration_damping_{1.0};
  double contact_friction_{1.0};
};

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
