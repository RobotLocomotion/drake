#pragma once

#include <string>
#include <vector>

#include "drake/common/drake_export.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/system1/System.h"
#include "drake/systems/plants/KinematicsCache.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/systems/plants/joints/floating_base_types.h"
#include "drake/systems/plants/parser_model_instance_id_table.h"

using drake::systems::plants::joints::FloatingBaseType;
using drake::systems::plants::joints::kQuaternion;
using drake::systems::plants::joints::kRollPitchYaw;

/** Rigid Body Dynamics Engine Class Design  (still needs to be implemented
 * below)
 *
 * RigidBodyTree (isa System)
 * Input: generalized forces (tau)
 * Output: generalized state (q, v)
 *
 * ContinuousTimeConstraintForce  (isa Constraint)
 * For forces that must be computed simultaneously with accelerations (not
 * simply a function of state)
 * Described as a vector phi(q, v, vdot, f)>=0, and forceJacobian(q) in terms of
 * vdot
 *    Note: tau_constraint = forceJacobian(q)^T*f.
 * also exposes an interface to be evaluated as phi(vdot, f)>=0, with the kinsol
 * set as a parameter.  Note that the complexity class of this constraint is
 * likely to be simpler than of the original constraint.
 * Example: position constraint:  J(q)*vdot + Jdot*v - stabilization terms = 0.
 * forceJacobian(q) = J // the jacobian of the position constraint
 * Example: stick-slip frictional contact (as a set of nonlinear complementarity
 * constraints imposing non-penetration + the friction cone)
 *
 * TimeSteppingConstraintForce (isa Constraint)
 * Writable as a vector phi(q, v, qn, vn, f)>=0.
 *    Note: tau_constraint = J^T(q)*f.
 * Example: stick-slip frictional contact w/ a linearized friction code (as
 * linear complementarity constraints)
 *
 * Sensor (isa System)
 * Input: generalized state
 * Output: sensor reading
 * (can have internal dynamics/state)
 * Examples: FullStateSensor, Encoder, IMU, Lidar, ...
 *
 * Actuator (isa System)    (anything that applies forces which can be computed
 * from the current state)
 * Input: generalized state, input command
 * Output: generalized force, tau_actuator
 * (can have internal dynamics/state)
 * Examples: GeneralizedForce, TorqueSource, SpatialForce, Linear
 * Spring/Dampers, Aerodynamic Forces, ...
 * Example: (no-stick) frictional contact: f_normal = max(-k*phi(q) -
 * b*phidot(q, v), 0).  f_tangent = min(b*norm(tangential_velocity),
 * mu*f_normal) *
 * tangential_velocity/norm(tangential_velocity).  forceJacobian is the contact
 * jacobian.
 *
 * The value/importance of thinking of sensors and actuators as systems is that
 * they can be implemented in a separate
 * executable using the signal abstraction. Note, however, that there is
 * efficiency to be gained by keeping it in the
 * same executable thanks to the kinematics cache.
 *
 * RigidBodySystem (isa System):
 * a RigidBodyTree + a list of Actuators and Sensors + list of
 * ContinuousTimeConstraintForces.  Limited to sensors/actuators w/o discrete
 * dynamics.
 * adds the additional constraints:
 * H(q) vdot + C(q, v) = \sum tau_actuators(q, v) + \sum
 * tau_constraints(q, v, vdot, f).
 * then solves for vdot and f, then computes qdot = vToQdot*v
 * Input: Actuator inputs (only; does not include all generalized forces by
 * default).
 * Output: Sensor outputs (only; does not include the entire generalized state
 * by default).
 *
 * TimeSteppingRigidBodySystem (isa System, with purely discrete-time dynamics):
 * a RigidBodyTree + a list of Actuators and Sensors + a list of
 * TimeSteppingConstraintForces.  Limited to sensors/actuators w/o continuous
 * dynamics.
 * adds the additional constraints:
 * H(q) (vn-v)/h + C(q, v) = \sum tau_actuators(q, v) + \sum
 * tau_constraints(q, v, qn, vn, f)
 * then solves for vn and f using qn = q + h*vToQdot(q)*vn
 * Input: Actuator inputs (only; does not include all generalized forces by
 * default).
 * Output: Sensor outputs (only; does not include the entire generalized state
 * by default).
 *
 */

namespace tinyxml2 {
class XMLElement;
}

namespace drake {

class RigidBodyForceElement;  // forward declaration
class RigidBodySensor;        // forward declaration

namespace RigidBodyConstraints {
/** @defgroup rigid_body_constraint RigidBodyConstraint Concept
 * @ingroup concepts
 * @{
 * @brief A constraint that can be updated using the state of the rigid body
 * system
 * @nbsp
 *
 * <table>
 * <tr><th colspan="2"> Valid Expressions (which must be implemented)
 * <tr><td><pre>
 * template &lt;typename ScalarType>
 * updateConstraint(const KinematicCache<ScalarType>& kinsol)</pre>
 *     <td> Updates the parameters of the constraint
 * <tr><td> Eigen::MatrixXd constraintForceJacobian()
 *     <td> returns the J used in J^T force for any constraint forces implied
 * <tr><td> size_t getNumConstraintForces() <td>
 * </table>
 * @}
 */

/* RigidBodyConstraint::LoopConstraint
 * @brief Implements Hvdot = C
 * @concept{rigid_body_constraint}
 */
/*
class LoopConstraint : public LinearEqualityConstraint {

};
*/
}  // namespace RigidBodyConstraints

/**
 * @brief Implements the System concept by wrapping the RigidBodyTree algorithms
 * with additional sensors and actuators/forces.
 *
 * Implements the System concept where state is both joint position and
 * velocity.
 *
 * @concept{system_concept}
 */
class DRAKE_EXPORT RigidBodySystem {
 public:
  template <typename ScalarType>
  using InputVector = Eigen::Matrix<ScalarType, Eigen::Dynamic, 1>;
  template <typename ScalarType>
  using StateVector = Eigen::Matrix<ScalarType, Eigen::Dynamic, 1>;
  template <typename ScalarType>
  using OutputVector = Eigen::Matrix<ScalarType, Eigen::Dynamic, 1>;

  explicit RigidBodySystem(std::shared_ptr<RigidBodyTree> rigid_body_tree)
      : use_multi_contact(false),
        penetration_stiffness(150.0),
        penetration_damping(penetration_stiffness / 10.0),
        friction_coefficient(1.0),
        tree(rigid_body_tree),
        direct_feedthrough(false) {}

  RigidBodySystem()
      : use_multi_contact(false),
        penetration_stiffness(150.0),
        penetration_damping(penetration_stiffness / 10.0),
        friction_coefficient(1.0),
        direct_feedthrough(false) {
    tree = std::shared_ptr<RigidBodyTree>(new RigidBodyTree());
  }

  virtual ~RigidBodySystem() {}

  /**
   * Reads a model specification from a URDF string and adds an instance of
   * the model into this `RigidBodySystem`'s `RigidBodyTree`.
   *
   * @param[in] urdf_string The URDF specification.
   *
   * @param[in] root_dir The root directory in which to search for files
   * mentioned in the URDF.
   *
   * @param[in] floating_base_type The type of joint that connects the model
   * instance's root to the existing `RigidBodyTree` within this
   * `RigidBodySystem`.
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
  drake::parsers::ModelInstanceIdTable AddModelInstanceFromUrdfString(
      const std::string& urdf_string, const std::string& root_dir = ".",
      const FloatingBaseType floating_base_type = kRollPitchYaw,
      std::shared_ptr<RigidBodyFrame> weld_to_frame = nullptr);

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
  drake::parsers::ModelInstanceIdTable AddModelInstanceFromUrdfFile(
      const std::string& filename,
      const FloatingBaseType floating_base_type = kQuaternion,
      std::shared_ptr<RigidBodyFrame> weld_to_frame = nullptr);

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
      const FloatingBaseType floating_base_type = kQuaternion,
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
      const FloatingBaseType floating_base_type = kQuaternion,
      std::shared_ptr<RigidBodyFrame> weld_to_frame = nullptr);

  /**
   * Adds one instance of each model defined within a SDF or URDF file to this
   * `RigidBodySystem`'s `RigidBodyTree`.
   *
   * @param[in] filename The name of the SDF of URDF file containing the models.
   * One instance of each of these models is added to this `RigidBodySystem` and
   * its `RigidBodyTree`.
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
  drake::parsers::ModelInstanceIdTable AddModelInstanceFromFile(
      const std::string& filename,
      const FloatingBaseType floating_base_type = kQuaternion,
      std::shared_ptr<RigidBodyFrame> weld_to_frame = nullptr);

  /**
   * Adds one instance of each model defined within a SDF or URDF string to this
   * `RigidBodySystem`'s `RigidBodyTree`.
   *
   * @param[in] string_description The SDF or URDF string of the model or
   * models. One instance of each of these models is added to this
   * `RigidBodySystem` and its `RigidBodyTree`.
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
  drake::parsers::ModelInstanceIdTable AddModelInstancesFromString(
      const std::string& string_description,
      const FloatingBaseType floating_base_type = kQuaternion,
      std::shared_ptr<RigidBodyFrame> weld_to_frame = nullptr);

  void addForceElement(std::shared_ptr<RigidBodyForceElement> f) {
    force_elements.push_back(f);
  }

  void addSensor(std::shared_ptr<RigidBodySensor> s);

  const std::shared_ptr<RigidBodyTree>& getRigidBodyTree(void) const {
    return tree;
  }

  // This class overrides the System method. See System's documentation for more
  // details. The state includes both joint position and velocity values. See
  // class overview description.
  size_t getNumStates() const;

  // This class overrides the System getNumInputs() method. See System's
  // documentation for more details.
  size_t getNumInputs() const;

  // This class overrides the System method. See System's documentation for more
  // details. The outputs include both joint state and the number of sensor
  // states. See class overview description for more details.
  size_t getNumOutputs() const;

  /**
   * An accessor to the number of position states outputted by this rigid body
   * system.
   */
  int get_num_positions() const;

#ifndef SWIG
  DRAKE_DEPRECATED("Please use get_num_positions().")
#endif
  int number_of_positions() const;

  /**
   * An accessor to the number of velocity states outputted by this rigid body
   * system.
   */
  int get_num_velocities() const;

#ifndef SWIG
  DRAKE_DEPRECATED("Please use get_num_velocities().")
#endif
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

  bool isTimeVarying() const { return false; }
  bool isDirectFeedthrough() const { return direct_feedthrough; }

  friend DRAKE_EXPORT StateVector<double> getInitialState(
      const RigidBodySystem& sys);

  /**
   * An accessor to the sensors within this rigid body system. This is useful
   * for downstream components to understand the meaning of the output signal
   * of this system.
   *
   * @return a const reference to the sensors vector within this rigid body
   * system.
   */
  std::vector<const RigidBodySensor*> GetSensors() const;

  // some parameters defining the contact
  bool use_multi_contact;
  double penetration_stiffness;  // k
  double penetration_damping;    // b
  double friction_coefficient;   // mu

 private:
  std::shared_ptr<RigidBodyTree> tree;
  std::vector<std::shared_ptr<RigidBodyForceElement>> force_elements;
  std::vector<std::shared_ptr<RigidBodySensor>> sensors;
  bool direct_feedthrough;

  /*
  mutable MathematicalProgram dynamics_program;
  */
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/** RigidBodyForceElement
 * @brief interface class for elements which define a generalized force acting
 * on the rigid body system
 */
class DRAKE_EXPORT RigidBodyForceElement {
 public:
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  RigidBodyForceElement(RigidBodySystem& sys_in, const std::string& name_in,
      int model_instance_id) :
          sys(sys_in),
          name(name_in),
          model_instance_id_(model_instance_id) {}
  virtual ~RigidBodyForceElement() {}

  virtual size_t getNumInputs() const { return 0; }
  virtual Eigen::VectorXd output(
      const double& t,
      /* todo: add force state here */ const Eigen::VectorXd& u,
      const KinematicsCache<double>& rigid_body_state) const = 0;

 protected:
  RigidBodySystem& sys;
  std::string name;
  int model_instance_id_{0};
};

/** spatialForceInFrameToJointTorque
 * @brief helper function for rigid body force elements.  todo: move this into
 * RigidBodyTree?
 */
Eigen::VectorXd spatialForceInFrameToJointTorque(
    const RigidBodyTree* tree, const KinematicsCache<double>& rigid_body_state,
    const RigidBodyFrame* frame, const Eigen::Matrix<double, 6, 1>& force);

// todo: insert a RigidBodyForceImpl with CRTP here once I go back and template
// these methods

/** RigidBodyPropellor
 * @brief Models the forces and moments produced by a simple propellor
 */
class DRAKE_EXPORT RigidBodyPropellor : public RigidBodyForceElement {
 public:
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  RigidBodyPropellor(RigidBodySystem& sys, tinyxml2::XMLElement* node,
                     const std::string& name, int model_instance_id);
  ~RigidBodyPropellor() override {}

  size_t getNumInputs() const override { return 1; }

  // some quick thoughts:
  // might want to be nonlinear in the robot state, but linear in the prop
  // input.
  // probably means I want to separate out those inputs
  // and that I want a more general way to specify the input-output
  // relationships for miso functions

  Eigen::VectorXd output(
      const double& t,
      /* todo: add force state here */ const Eigen::VectorXd& u,
      const KinematicsCache<double>& rigid_body_state) const override {
    Eigen::Matrix<double, 6, 1> force;
    force << scale_factor_moment * u(0) * axis,
        scale_factor_thrust * u(0) * axis;
    return spatialForceInFrameToJointTorque(
        sys.getRigidBodyTree().get(), rigid_body_state, frame.get(), force);
  }

 private:
  std::shared_ptr<RigidBodyFrame> frame;
  Eigen::Vector3d axis;
  double scale_factor_thrust;  // scale factor between input and thrust
  double scale_factor_moment;  // scale factor between input and moment due to
                               // aerodynamic drag
  double lower_limit;
  double upper_limit;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/** RigidBodySpringDamper
 * @brief Models the forces produced by a linear spring-damper
 */
class DRAKE_EXPORT RigidBodySpringDamper
    : public RigidBodyForceElement {
 public:
  // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
  RigidBodySpringDamper(RigidBodySystem& sys, tinyxml2::XMLElement* node,
                        const std::string& name, int model_instance_id);
  ~RigidBodySpringDamper() override {}

  Eigen::VectorXd output(
      const double& t,
      /* todo: add force state here */ const Eigen::VectorXd& u,
      const KinematicsCache<double>& rigid_body_state) const override {
    using namespace Eigen;  // NOLINT(build/namespaces)
    const Vector3d origin = Vector3d::Zero();
    Vector3d xA_in_B = sys.getRigidBodyTree()->transformPoints(
        rigid_body_state, origin, frameA->get_frame_index(),
        frameB->get_frame_index());
    Vector3d xB_in_A = sys.getRigidBodyTree()->transformPoints(
        rigid_body_state, origin, frameB->get_frame_index(),
        frameA->get_frame_index());
    auto JA_in_B = sys.getRigidBodyTree()->transformPointsJacobian(
        rigid_body_state, origin, frameA->get_frame_index(),
        frameB->get_frame_index(),
        false);

    double length = xA_in_B.norm();
    double vel =
        (JA_in_B * rigid_body_state.getV()).dot(xA_in_B) / (length + EPSILON);
    double force_magnitude = stiffness * (length - rest_length) + damping * vel;

    //      std::cout << "l=" << length << ", v=" << vel << ", f=" <<
    //      force_magnitude << std::endl;

    Matrix<double, 6, 1> force = Matrix<double, 6, 1>::Zero();

    // apply (force_magnitude/length)*xA_in_B to B
    force.tail<3>() = (force_magnitude / (length + EPSILON)) * xA_in_B;
    auto tau = spatialForceInFrameToJointTorque(
        sys.getRigidBodyTree().get(), rigid_body_state, frameB.get(), force);

    // apply (force_magnitude/length)*xB_in_A to A
    force.tail<3>() = (force_magnitude / (length + EPSILON)) * xB_in_A;
    tau += spatialForceInFrameToJointTorque(
        sys.getRigidBodyTree().get(), rigid_body_state, frameA.get(), force);
    return tau;
  }

 private:
  std::shared_ptr<RigidBodyFrame> frameA, frameB;
  double stiffness;
  double damping;
  double rest_length;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/** NoiseModel
 * @brief Represents generalized vector-valued noise
 */
template <typename ScalarType, int Dimension, typename Derived>
class NoiseModel {
 public:
  virtual Eigen::Matrix<ScalarType, Dimension, 1> generateNoise(
      Eigen::MatrixBase<Derived> const& input) = 0;
};

/** GaussianNoiseModel
 * @brief Implements the NoiseModel interface where the underlying noise
 * distribution is parameterized by a Gaussian
 */
template <typename ScalarType, int Dimension, typename Derived>
class AdditiveGaussianNoiseModel
    : public NoiseModel<ScalarType, Dimension, Derived> {
 public:
  AdditiveGaussianNoiseModel(double mean, double std_dev)
      : distribution(mean, std_dev), generator(rd()) {}

  Eigen::Matrix<ScalarType, Dimension, 1> generateNoise(
      Eigen::MatrixBase<Derived> const& input) override {
    Eigen::Matrix<ScalarType, Dimension, 1> noise_vector;
    for (std::size_t index = 0; index < Dimension; index++) {
      noise_vector[index] = distribution(generator);
    }
    return noise_vector + input;
  }

 private:
  std::random_device rd;
  std::normal_distribution<ScalarType> distribution;
  std::mt19937 generator;
};

/**
 * An abstract parent class of all sensors.
 *
 * This is an abstract top-level class of all rigid body sensors in Drake.
 */
class DRAKE_EXPORT RigidBodySensor {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * The constructor.
   *
   * @param[in] sys The rigid body system to which the sensor is attached.
   * @param[in] name The name of the sensor.
   * @param[in] frame The frame within the rigid body system's rigid body tree
   * to which the sensor is attached.
   */
  RigidBodySensor(const RigidBodySystem& sys, const std::string& name,
                  std::shared_ptr<RigidBodyFrame> frame)
      : sys_(sys), name_(name), frame_(frame) {}

  virtual ~RigidBodySensor() {}

  virtual bool isDirectFeedthrough() const { return false; }

  virtual size_t getNumOutputs() const { return 0; }

  virtual Eigen::VectorXd output(
      const double& t, const KinematicsCache<double>& rigid_body_state,
      const RigidBodySystem::InputVector<double>& u) const = 0;

  /// Returns the name of the sensor.
  const std::string& get_name() const { return name_; }

  /// Returns the name of the model (i.e., robot) that owns this sensor.
  const std::string& get_model_name() const;

  /// Returns the frame to which thi sensor is attached.
  const RigidBodyFrame& get_frame() const;

  /// Returns the rigid body system to which this sensor attaches.
  const RigidBodySystem& get_rigid_body_system() const;

 private:
  /// The rigid body tree to which the sensor is attached.
  const RigidBodySystem& sys_;

  /// The sensor's name.
  const std::string name_;

  /// The frame within the rigid body tree to which this sensor is attached.
  const std::shared_ptr<RigidBodyFrame> frame_;
};

/** RigidBodyDepthSensor
 * @brief Uses raycast to simulate a depth image at some evenly spaced pixel
 * rows and columns.
 */
class DRAKE_EXPORT RigidBodyDepthSensor : public RigidBodySensor {
 public:
  RigidBodyDepthSensor(RigidBodySystem const& sys, const std::string& name,
                       const std::shared_ptr<RigidBodyFrame> frame,
                       tinyxml2::XMLElement* node);

  RigidBodyDepthSensor(RigidBodySystem const& sys, const std::string& name,
                       const std::shared_ptr<RigidBodyFrame> frame,
                       std::size_t samples, double min_angle, double max_angle,
                       double range);

  ~RigidBodyDepthSensor() override {}

  size_t getNumOutputs() const override;

  /**
   * Returns `true` if this sensor scans horizontally.
   */
  virtual bool is_horizontal_scanner() const;

  /**
   * Returns `true` if this sensor scans vertically.
   */
  virtual bool is_vertical_scanner() const;

  /**
   * Returns the number of points in the image vertically (pitch).
   */
  virtual size_t num_pixel_rows() const;

  /**
   * Returns the number of points in the image horizontally (yaw).
   */
  virtual size_t num_pixel_cols() const;

  /**
   * Returns minimum pitch of this sensor's FOV in radians.
   */
  virtual double min_pitch() const;

  /**
   * Returns maximum pitch of this sensor's FOV in radians.
   */
  virtual double max_pitch() const;

  /**
   * Returns the minimum yaw of this sensor's FOV in radians.
   */
  virtual double min_yaw() const;

  /**
   * Returns the maximum yaw of this sensor's FOV in radians.
   */
  virtual double max_yaw() const;

  /**
   * Returns the minimum range of this sensor in meters.
   */
  virtual double min_range() const;

  /**
   * Returns the maximum range of this sensor in meters.
   */
  virtual double max_range() const;

  Eigen::VectorXd output(
      const double& t, const KinematicsCache<double>& rigid_body_state,
      const RigidBodySystem::InputVector<double>& u) const override;

 private:
  // Ensures that the configuration of this sensor is valid.
  // Throws an exception if it is not valid.
  void CheckValidConfiguration();

  // The depth sensor will cast a ray with its start point at (0,0,0) in the
  // sensor's frame (RigidBodyDepthSensor::frame_). Its end, in the sensor's
  // frame, is computed by this method and stored in raycast_endpoints at the
  // moment of construction. raycast_endpoints is only computed once at
  // construction since the end points are constant in the frame of the sensor.
  // The end points are computed by scanning in the yaw (pitch) direction
  // discretizing the yaw (pitch) range in num_pixel_cols (num_pixel_rows).
  // The final 3D end point then corresponds to a ray that starts at zero, with
  // length max_range, at the specific yaw (pitch) angle.
  void cacheRaycastEndpoints();

  // The minimum pitch of the camera FOV in radians.
  double min_pitch_{};

  // The maximum pitch of the camera FOV in radians.
  double max_pitch_{};

  // The minimum yaw of the sensor FOV in radians.
  double min_yaw_{};

  // The maximum yaw of the sensor FOV in radians
  double max_yaw_{};

  // The number of points in the image vertically (pitch).
  size_t num_pixel_rows_{1};

  // The number of points in the image horizontally (yaw).
  size_t num_pixel_cols_{1};

  // The minimum range of the sensor in meters.
  double min_range_{0};

  // The maximum range of the sensor in meters.
  double max_range_{10};

  // A cache to avoid repeated allocation.
  Eigen::Matrix3Xd raycast_endpoints;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/** RigidBodyAccelerometer
 * @brief Simulates a sensor that measures linear acceleration
 */
class DRAKE_EXPORT RigidBodyAccelerometer : public RigidBodySensor {
 public:
  RigidBodyAccelerometer(RigidBodySystem const& sys, const std::string& name,
                         const std::shared_ptr<RigidBodyFrame> frame);
  ~RigidBodyAccelerometer() override {}

  size_t getNumOutputs() const override { return 3; }
  Eigen::VectorXd output(
      const double& t, const KinematicsCache<double>& rigid_body_state,
      const RigidBodySystem::InputVector<double>& u) const override;
  bool isDirectFeedthrough() const override { return true; }
  void setNoiseModel(
      std::shared_ptr<NoiseModel<double, 3, Eigen::Vector3d>> model) {
    noise_model = model;
  }

  void setGravityCompensation(bool enable_compensation) {
    gravity_compensation = enable_compensation;
  }

 private:
  bool gravity_compensation;
  std::shared_ptr<NoiseModel<double, 3, Eigen::Vector3d>> noise_model;
};

/** RigidBodyGyroscope
 * @brief Simulates a sensor that measures angular rates
 */
class DRAKE_EXPORT RigidBodyGyroscope : public RigidBodySensor {
 public:
  RigidBodyGyroscope(RigidBodySystem const& sys, const std::string& name,
                     const std::shared_ptr<RigidBodyFrame> frame);
  ~RigidBodyGyroscope() override {}

  size_t getNumOutputs() const override { return 3; }
  Eigen::VectorXd output(
      const double& t, const KinematicsCache<double>& rigid_body_state,
      const RigidBodySystem::InputVector<double>& u) const override;

  void setNoiseModel(
      std::shared_ptr<NoiseModel<double, 3, Eigen::Vector3d>> model) {
    noise_model = model;
  }

 private:
  std::shared_ptr<NoiseModel<double, 3, Eigen::Vector3d>> noise_model;
};

/** RigidBodyMagnetometer
 * @brief Simulates a sensor that measures magnetic fields
 */
class DRAKE_EXPORT RigidBodyMagnetometer : public RigidBodySensor {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  RigidBodyMagnetometer(RigidBodySystem const& sys, const std::string& name,
                        const std::shared_ptr<RigidBodyFrame> frame,
                        double declination);
  ~RigidBodyMagnetometer() override {}

  size_t getNumOutputs() const override { return 3; }
  Eigen::VectorXd output(
      const double& t, const KinematicsCache<double>& rigid_body_state,
      const RigidBodySystem::InputVector<double>& u) const override;

  void setNoiseModel(
      std::shared_ptr<NoiseModel<double, 3, Eigen::Vector3d>> model) {
    noise_model = model;
  }

  void setDeclination(double magnetic_declination) {
    magnetic_north << cos(magnetic_declination), sin(magnetic_declination), 0;
  }

 private:
  Eigen::Vector3d magnetic_north;
  std::shared_ptr<NoiseModel<double, 3, Eigen::Vector3d>> noise_model;
};

}  // end namespace drake
