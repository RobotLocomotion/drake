#pragma once

#include <memory>
#include <string>

#include "drake/drake_rbs_export.h"
#include "drake/systems/framework/leaf_system.h"

#include "drake/systems/plants/parser_model_instance_id_table.h"
#include "drake/systems/plants/RigidBodyTree.h"

namespace tinyxml2 {
class XMLElement;
}

namespace drake {
namespace systems {

// TODO(amcastro-tri): Make RBS inherit from Diagram<T> once #3215 is solved.
// RigidBodySystem is a diagram containing the multibody dynamics engine system
// connected to forcing systems and sensor systems.
template<typename T>
class DRAKE_RBS_EXPORT RigidBodySystem : public LeafSystem<T> {
 public:
  /// Doc.
  RigidBodySystem();

  virtual ~RigidBodySystem() override;

  const RigidBodyTree& get_multibody_world() const;

  int get_num_generalized_positions() const;

  int get_num_generalized_velocities() const;

  int get_num_states() const;

  int get_num_generalized_forces() const;

  int get_num_inputs() const;

  int get_num_outputs() const;

  /**
   * Reads a model specification from a URDF file and adds an instance of the
   * model into this `RigidBodySystem`'s `RigidBodyTree`.
   *
   * @param[in] filename The name of the file containing the URDF
   * specification.
   *
   * @param[in] floating_base_type The type of joint that connects the model
   * instance's root to this `RigidBodySystem`'s `RigidBodyTree`.
   *
   * @return A table mapping the names of the models whose instances were just
   * added to the `RigidBodyTree` to their instance IDs, which are unique within
   * the `RigidBodyTree`.
   */
  drake::parsers::ModelInstanceIdTable AddModelInstanceFromUrdfFile(
      const std::string& filename,
      DrakeJoint::FloatingBaseType floating_base_type = DrakeJoint::QUATERNION);

 private:
  bool has_any_direct_feedthrough() const override;

  std::unique_ptr<ContinuousState<T>> AllocateContinuousState() const override;

  //void EvalTimeDerivatives(const ContextBase<T>& context,
  //                         ContinuousState<T>* derivatives) const override;

  void EvalOutput(const ContextBase<T>& context,
                  SystemOutput<T>* output) const override;

  // some parameters defining the contact.
  // TODO(amcastro-tri): Implement contact materials for the RBT engine.
  T penetration_stiffness_;
  T penetration_damping_;
  T friction_coefficient_;

 private:
  std::unique_ptr<RigidBodyTree> multibody_world_;
};

}  // namespace systems
}  // namespace drake
