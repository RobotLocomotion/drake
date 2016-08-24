#pragma once

#include <memory>

#include "drake/drakeRBS_export.h"
#include "drake/systems/framework/leaf_system.h"

//#include "drake/systems/plants/parser_model_instance_id_table.h"
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
class DRAKERBS_EXPORT RigidBodySystem : public LeafSystem<T> {
 public:
  /// Doc.
  RigidBodySystem() {
    penetration_stiffness_ = 150;
    penetration_damping_ = penetration_stiffness / 10.0;

    //this->DeclareInputPort(kVectorValued, length, kContinuousSampling);
    //this->DeclareOutputPort(kVectorValued, length, kContinuousSampling);

    // A default world with only the "world" body.
    multibody_world_ = std::make_unique<RigidBodyTree>();
  }

  virtual ~RigidBodySystem() {}

#if 0
  /**
   * Adds one instance of each model defined within an SDF file to this
   * `RigidBodySystem`'s `RigidBodyTree`.
   *
   * Each model instance is uniquely identified by a model instance ID that is
   * assigned at the time the model instance is added. Since model instances are
   * distinguished by a model instance ID, multiple instances of the models
   * within the SDF file can be added into a single `RigidBodySystem` and its
   * `RigidBodyTree`.
   *
   * @param[in] filename The name of the SDF file describing one or more models.
   * One instance of each of these models is added to this `RigidBodySystem`'s
   * `RigidBodyTree`.
   *
   * @param[in] floating_base_type The type of floating base to use to connect
   * the newly created model instances to the world.
   *
   * @param[in] weld_to_frame The frame used for connecting the new model
   * instances to the `RigidBodyTree` within this `RigidBodySystem`. Note that
   * this parameter specifies both an existing frame in the `RigidBodyTree` and
   * the offset from this frame to the frame belonging to the new model
   * instances' root bodies. This is an optional parameter. If it is `nullptr`,
   * the newly-created model instances are connected to the world with zero
   * offset and rotation relative to the world's frame.
   *
   * @return A table mapping the names of the models whose instances were just
   * added to the `RigidBodyTree` to their instance IDs, which are unique within
   * the `RigidBodyTree`.
   */
  drake::parsers::ModelInstanceIdTable AddModelInstancesFromSdfFile(
      const std::string& filename,
      const DrakeJoint::FloatingBaseType floating_base_type =
          DrakeJoint::QUATERNION,
      std::shared_ptr<RigidBodyFrame> weld_to_frame = nullptr);

  /**
   * Adds one instance of each model defined within an SDF string to this
   * `RigidBodySystem`'s `RigidBodyTree`.
   *
   * Each model instance is uniquely identified by a model instance ID that is
   * assigned at the time the model instance is added. Since model instances are
   * distinguished by a model instance ID, multiple instances of the models
   * within the SDF file can be added into a single `RigidBodySystem` and its
   * `RigidBodyTree`.
   *
   * @param[in] sdf_string The SDF string that describes one or more models. One
   * instance of each model is added to this `RigidBodySystem` and its
   * `RigidBodyTree`.
   *
   * @param[in] floating_base_type The type of floating base to use to connect
   * the newly created model instances to the world.
   *
   * @param[in] weld_to_frame The frame used for connecting the new model
   * instances to the `RigidBodyTree` within this `RigidBodySystem`. Note that
   * this parameter specifies both an existing frame in the `RigidBodyTree` and
   * the offset from this frame to the frame belonging to the new model
   * instances' root bodies. This is an optional parameter. If it is `nullptr`,
   * the newly-created model instances are connected to the world with zero
   * offset and rotation relative to the world's frame.
   *
   * @param[out] model_instance_id_table A pointer to a map storing model
   * names and their instance IDs. This parameter may not be `nullptr`. A
   * `std::runtime_error` is thrown if an instance is created of a model whose
   * name is already in this table.
   */
  drake::parsers::ModelInstanceIdTable AddModelInstancesFromSdfString(
      const std::string& sdf_string,
      const DrakeJoint::FloatingBaseType floating_base_type =
          DrakeJoint::QUATERNION,
      std::shared_ptr<RigidBodyFrame> weld_to_frame = nullptr);

  const RigidBodyTree& get_rigid_body_tree() const {
    return *tree.get();
  }

  // This class overrides the System method. See System's documentation for more
  // details. The state includes both joint position and velocity values. See
  // class overview description.
  int get_num_states() const;

  // This class overrides the System getNumInputs() method. See System's
  // documentation for more details.
  int get_num_generalized_forces() const;

  /**
   * An accessor to the number of position states outputted by this rigid body
   * system.
   */
  int number_of_positions() const;

  /**
   * An accessor to the number of velocity states outputted by this rigid body
   * system.
   */
  int number_of_velocities() const;

  /** dynamics
   * Formulates the forward dynamics of the rigid body system as an optimization
   *   find vdot, f  (feasibility problem ok for now => implicit objective is
   * min norm solution)
   *   subject to
   *       position equality constraints (differentiated twice + stabilization):
   * A vdot = b
   *       velocity equality constraints (differentiated once + stabilization):
   * A vdot = b
   *       forces from joint limits and contact OR
   *       contact force constraints on vdot, f.  can be linear, nonlinear, even
   * complementarity.  may have inequalities
   *   the trick is that each new constraint can add decision variables (the new
   * constraint forces and/or slack variables)
   *   to the problem, so the last constraint to add is
   *       equations of motion: H vdot + C(q, qdot, u, f_ext) = J^T(q, qdot) f
   *   where J is accumulated through the constraint logic
   *
   * The solver will then dispatch to the right tool for the job.  Note that for
   * many systems, especially those
   * without any contact constraints (or with simple friction models), the
   * formulation is linear and can be solved
   * with least-squares.
   */
  StateVector<double> dynamics(const double& t, const StateVector<double>& x,
                               const InputVector<double>& u) const;

  OutputVector<double> output(const double& t, const StateVector<double>& x,
                              const InputVector<double>& u) const;

  bool has_any_direct_feedthrough() const { return false; }

  // Replace by compute_initial_state()? return a ContinuousState or a Context?
  friend DRAKERBSYSTEM_EXPORT StateVector<double> getInitialState(
      const RigidBodySystem& sys);

#endif

  // some parameters defining the contact.
  // TODO(amcastro-tri): Implement contact materials for the RBT engine.
  T penetration_stiffness_;
  T penetration_damping_;
  T friction_coefficient;

 private:
  std::unique_ptr<RigidBodyTree> multibody_world_;
};

}  // namespace systems
}  // namespace drake
