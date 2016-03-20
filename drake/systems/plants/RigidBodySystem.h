#ifndef DRAKE_RIGIDBODYSYSTEM_H
#define DRAKE_RIGIDBODYSYSTEM_H

#include "drake/systems/System.h"
#include "drake/solvers/Optimization.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "KinematicsCache.h"
#include "drake/drakeRBSystem_export.h"

/** Rigid Body Dynamics Engine Class Design  (still needs to be implemented
 *below)
 *
 * RigidBodyTree (isa System)
 * Input: generalized forces (tau)
 * Output: generalized state (q, v)
 *
 * ContinuousTimeConstraintForce  (isa Constraint)
 * For forces that must be computed simultaneously with accelerations (not
 *simply a function of state)
 * Described as a vector phi(q, v, vdot, f)>=0, and forceJacobian(q) in terms of
 *vdot
 *    Note: tau_constraint = forceJacobian(q)^T*f.
 * also exposes an interface to be evaluated as phi(vdot, f)>=0, with the kinsol
 *set as a parameter.  Note that the complexity class of this constraint is
 *likely to be simpler than of the original constraint.
 * Example: position constraint:  J(q)*vdot + Jdot*v - stabilization terms = 0.
 *forceJacobian(q) = J // the jacobian of the position constraint
 * Example: stick-slip frictional contact (as a set of nonlinear complementarity
 *constraints imposing non-penetration + the friction cone)
 *
 * TimeSteppingConstraintForce (isa Constraint)
 * Writable as a vector phi(q, v, qn, vn, f)>=0.
 *    Note: tau_constraint = J^T(q)*f.
 * Example: stick-slip frictional contact w/ a linearized friction code (as
 *linear complementarity constraints)
 *
 * Sensor (isa System)
 * Input: generalized state
 * Output: sensor reading
 * (can have internal dynamics/state)
 * Examples: FullStateSensor, Encoder, IMU, Lidar, ...
 *
 * Actuator (isa System)    (anything that applies forces which can be computed
 *from the current state)
 * Input: generalized state, input command
 * Output: generalized force, tau_actuator
 * (can have internal dynamics/state)
 * Examples: GeneralizedForce, TorqueSource, SpatialForce, Linear
 *Spring/Dampers, Aerodynamic Forces, ...
 * Example: (no-stick) frictional contact: f_normal = max(-k*phi(q) -
 *b*phidot(q, v), 0).  f_tangent = min(b*norm(tangential_velocity), mu*f_normal) *
 *tangential_velocity/norm(tangential_velocity).  forceJacobian is the contact
 *jacobian.
 *
 * The value/importance of thinking of sensors and actuators as systems is that
 *they can be implemented in a separate
 * executable using the signal abstraction. Note, however, that there is
 *efficiency to be gained by keeping it in the
 * same executable thanks to the kinematics cache.
 *
 * RigidBodySystem (isa System):
 * a RigidBodyTree + a list of Actuators and Sensors + list of
 *ContinuousTimeConstraintForces.  Limited to sensors/actuators w/o discrete
 *dynamics.
 * adds the additional constraints:
 * H(q) vdot + C(q, v) = \sum tau_actuators(q, v) + \sum
 *tau_constraints(q, v, vdot, f).
 * then solves for vdot and f, then computes qdot = vToQdot*v
 * Input: Actuator inputs (only; does not include all generalized forces by
 *default).
 * Output: Sensor outputs (only; does not include the entire generalized state
 *by default).
 *
 * TimeSteppingRigidBodySystem (isa System, with purely discrete-time dynamics):
 * a RigidBodyTree + a list of Actuators and Sensors + a list of
 *TimeSteppingConstraintForces.  Limited to sensors/actuators w/o continuous
 *dynamics.
 * adds the additional constraints:
 * H(q) (vn-v)/h + C(q, v) = \sum tau_actuators(q, v) + \sum
 *tau_constraints(q, v, qn, vn, f)
 * then solves for vn and f using qn = q + h*vToQdot(q)*vn
 * Input: Actuator inputs (only; does not include all generalized forces by
 *default).
 * Output: Sensor outputs (only; does not include the entire generalized state
 *by default).
 *
 */

namespace tinyxml2 {
class XMLElement;
}

namespace Drake {

class RigidBodyForceElement;  // forward declaration
class RigidBodySensor;        // forward declaration

namespace RigidBodyConstraints {
/** @defgroup rigid_body_constraint RigidBodyConstraint Concept
 * @ingroup concepts
 * @{
 * @brief A constraint that can be updated using the state of the rigid body
 *system
 * @nbsp
 *
 * | Valid Expressions (which must be implemented) |  |
 * ------------------|-------------------------------------------------------------|
 * | template <typename ScalarType> updateConstraint(const
 *KinematicCache<ScalarType>& kinsol)  | Updates the parameters of the
 *constraint |
 * | Eigen::MatrixXd constraintForceJacobian() | returns the J used in J^T force
 *for any constraint forces implied
 * | size_t getNumConstraintForces() |
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
}

/** RigidBodySystem
 * @brief implements the System concept by wrapping the RigidBodyTree algorithms
 * with additional sensors and actuators/forces
 * @concept{system_concept}
 */
class DRAKERBSYSTEM_EXPORT RigidBodySystem {
 public:
  template <typename ScalarType>
  using InputVector = Eigen::Matrix<ScalarType, Eigen::Dynamic, 1>;
  template <typename ScalarType>
  using StateVector = Eigen::Matrix<ScalarType, Eigen::Dynamic, 1>;
  template <typename ScalarType>
  using OutputVector = Eigen::Matrix<ScalarType, Eigen::Dynamic, 1>;

  RigidBodySystem(std::shared_ptr<RigidBodyTree> rigid_body_tree)
      : tree(rigid_body_tree),
        use_multi_contact(false),
        penetration_stiffness(150.0),
        penetration_damping(penetration_stiffness / 10.0),
        friction_coefficient(1.0),
        direct_feedthrough(false) {};
  RigidBodySystem()
      : use_multi_contact(false),
        penetration_stiffness(150.0),
        penetration_damping(penetration_stiffness / 10.0),
        friction_coefficient(1.0),
        direct_feedthrough(false) {
    // tree =
    // std::allocate_shared<RigidBodyTree>(Eigen::aligned_allocator<RigidBodyTree>());
    // // this crashed g++-4.7
    tree = std::shared_ptr<RigidBodyTree>(new RigidBodyTree());

  }
  virtual ~RigidBodySystem(){};

  void addRobotFromURDFString(
      const std::string& xml_string, const std::string& root_dir = ".",
      const DrakeJoint::FloatingBaseType floating_base_type =
          DrakeJoint::ROLLPITCHYAW);
  void addRobotFromURDF(const std::string& urdf_filename,
                        const DrakeJoint::FloatingBaseType floating_base_type =
                            DrakeJoint::QUATERNION);
  void addRobotFromSDF(const std::string& sdf_filename,
                       const DrakeJoint::FloatingBaseType floating_base_type =
                           DrakeJoint::QUATERNION);
  void addRobotFromFile(const std::string& filename,
                        const DrakeJoint::FloatingBaseType floating_base_type =
                            DrakeJoint::QUATERNION);

  void addForceElement(std::shared_ptr<RigidBodyForceElement> f) {
    force_elements.push_back(f);
  }

  void addSensor(std::shared_ptr<RigidBodySensor> s);

  const std::shared_ptr<RigidBodyTree>& getRigidBodyTree(void) const { return tree; }

    size_t getNumStates() const {
    return tree->num_positions + tree->num_velocities;
  }
  size_t getNumInputs() const;
  size_t getNumOutputs() const;

  /** dynamics
   * Formulates the forward dynamics of the rigid body system as an optimization
   *   find vdot, f  (feasibility problem ok for now => implicit objective is
   *min norm solution)
   *   subject to
   *       position equality constraints (differentiated twice + stabilization):
   *A vdot = b
   *       velocity equality constraints (differentiated once + stabilization):
   *A vdot = b
   *       forces from joint limits and contact OR
   *       contact force constraints on vdot, f.  can be linear, nonlinear, even
   *complementarity.  may have inequalities
   *   the trick is that each new constraint can add decision variables (the new
   *constraint forces and/or slack variables)
   *   to the problem, so the last constraint to add is
   *       equations of motion: H vdot + C(q, qdot, u, f_ext) = J^T(q, qdot) f
   *   where J is accumulated through the constraint logic
   *
   * The solver will then dispatch to the right tool for the job.  Note that for
   *many systems, especially those
   * without any contact constraints (or with simple friction models), the
   *formulation is linear and can be solved
   * with least-squares.
   */
  StateVector<double> dynamics(const double& t, const StateVector<double>& x,
                               const InputVector<double>& u) const;

  OutputVector<double> output(const double& t, const StateVector<double>& x,
                              const InputVector<double>& u) const;

  bool isTimeVarying() const { return false; }
  bool isDirectFeedthrough() const { return direct_feedthrough; }

  friend DRAKERBSYSTEM_EXPORT StateVector<double> getInitialState(
      const RigidBodySystem& sys);

  // some parameters defining the contact
  bool use_multi_contact;
  double penetration_stiffness;  // k
  double penetration_damping;    // b
  double friction_coefficient;   // mu

 private:
  std::shared_ptr<RigidBodyTree> tree;
  std::vector<std::shared_ptr<RigidBodyForceElement> > force_elements;
  std::vector<std::shared_ptr<RigidBodySensor> > sensors;
  size_t num_sensor_outputs;
  bool direct_feedthrough;

  /*
  mutable OptimizationProblem dynamics_program;
  */
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/** RigidBodyForceElement
 * @brief interface class for elements which define a generalized force acting
 * on the rigid body system
 */
class DRAKERBSYSTEM_EXPORT RigidBodyForceElement {
 public:
  RigidBodyForceElement(RigidBodySystem &sys, const std::string& name)
      : sys(sys), name(name) {}
  virtual ~RigidBodyForceElement() {}

  virtual size_t getNumInputs() const { return 0; }
  virtual Eigen::VectorXd output(
      const double& t,
      /* todo: add force state here */ const Eigen::VectorXd& u,
      const KinematicsCache<double>& rigid_body_state) const = 0;

 protected:
  RigidBodySystem &sys;
  std::string name;
};

/** spatialForceInFrameToJointTorque
 * @brief helper function for rigid body force elements.  todo: move this into
 * RigidBodyTree?
 */
Eigen::VectorXd spatialForceInFrameToJointTorque(
    const RigidBodyTree* tree, const KinematicsCache<double>& rigid_body_state,
    const RigidBodyFrame* frame, const Eigen::Matrix<double, 6, 1>& force) {
  auto T_frame_to_world =
      tree->relativeTransform(rigid_body_state, 0, frame->frame_index);
  auto force_in_world = transformSpatialForce(T_frame_to_world, force);
  std::vector<int> v_indices;
  auto J = tree->geometricJacobian(rigid_body_state, 0, frame->frame_index, 0,
                                   false, &v_indices);
  Eigen::VectorXd tau = Eigen::VectorXd::Zero(tree->num_velocities);
  for (int i = 0; i < v_indices.size(); i++) {
    tau(v_indices[i]) = J.col(i).dot(force_in_world);
    //      std::cout << " f_" << tree->getVelocityName(v_indices[i]) << " = "
    //      << tau(v_indices[i]) << std::endl;
  }
  return tau;
}

// todo: insert a RigidBodyForceImpl with CRTP here once I go back and template
// these methods

/** RigidBodyPropellor
 * @brief Models the forces and moments produced by a simple propellor
 */
class DRAKERBSYSTEM_EXPORT RigidBodyPropellor : public RigidBodyForceElement {
 public:
  RigidBodyPropellor(RigidBodySystem &sys, tinyxml2::XMLElement* node,
                     const std::string& name);
  virtual ~RigidBodyPropellor() {}

  virtual size_t getNumInputs() const override { return 1; }

  // some quick thoughts:
  // might want to be nonlinear in the robot state, but linear in the prop
  // input.
  // probably means I want to separate out those inputs
  // and that I want a more general way to specify the input-output
  // relationships for miso functions

  virtual Eigen::VectorXd output(
      const double& t,
      /* todo: add force state here */ const Eigen::VectorXd& u,
      const KinematicsCache<double>& rigid_body_state) const override {
    Eigen::Matrix<double, 6, 1> force;
    force << scale_factor_moment* u(0) * axis,
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
class DRAKERBSYSTEM_EXPORT RigidBodySpringDamper
    : public RigidBodyForceElement {
 public:
  RigidBodySpringDamper(RigidBodySystem &sys, tinyxml2::XMLElement* node,
                        const std::string& name);
  virtual ~RigidBodySpringDamper() {}

  virtual Eigen::VectorXd output(
      const double& t,
      /* todo: add force state here */ const Eigen::VectorXd& u,
      const KinematicsCache<double>& rigid_body_state) const override {
    using namespace Eigen;
    const Vector3d origin = Vector3d::Zero();
    Vector3d xA_in_B = sys.getRigidBodyTree()->transformPoints(
        rigid_body_state, origin, frameA->frame_index, frameB->frame_index);
    Vector3d xB_in_A = sys.getRigidBodyTree()->transformPoints(
        rigid_body_state, origin, frameB->frame_index, frameA->frame_index);
    auto JA_in_B = sys.getRigidBodyTree()->transformPointsJacobian(
        rigid_body_state, origin, frameA->frame_index, frameB->frame_index,
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
class DRAKERBSYSTEM_EXPORT NoiseModel {
  public:
    virtual Eigen::Matrix<ScalarType, Dimension, 1> generateNoise(Eigen::MatrixBase<Derived> const& input) = 0;
};

/** GaussianNoiseModel
 * @brief Implements the NoiseModel interface where the underlying noise distribution is parameterized by a Gaussian
 */
template <typename ScalarType, int Dimension, typename Derived>
class DRAKERBSYSTEM_EXPORT AdditiveGaussianNoiseModel : public NoiseModel<ScalarType, Dimension, Derived> {
  public:
    AdditiveGaussianNoiseModel(double mean, double std_dev) : distribution(mean, std_dev), generator(rd()) { }

    virtual Eigen::Matrix<ScalarType, Dimension, 1> generateNoise(Eigen::MatrixBase<Derived> const& input) override {
        Eigen::Matrix<ScalarType, Dimension, 1> noise_vector;
        for(std::size_t index = 0; index < Dimension; index++) {
            noise_vector[index] = distribution(generator);
        }
        return noise_vector + input;
    }
  private:
    std::random_device rd;
    std::normal_distribution<ScalarType> distribution;
    std::mt19937 generator;
};

/** RigidBodySensor
 * @brief interface class for elements which define a sensor which reads the state of a rigid body system
 */
class DRAKERBSYSTEM_EXPORT RigidBodySensor {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RigidBodySensor(RigidBodySystem const& sys, const std::string& name)
      : sys(sys), name(name) {}
  virtual ~RigidBodySensor() {}
  virtual bool isDirectFeedthrough() const { return false; }
  virtual size_t getNumOutputs() const { return 0; }
  virtual Eigen::VectorXd output(
    const double& t,
    const KinematicsCache<double>& rigid_body_state,
    const RigidBodySystem::InputVector<double>& u) const = 0;

 protected:
    RigidBodySystem const& sys;
    std::string name;
};

/** RigidBodyDepthSensor
 * @brief Uses raycast to simulate a depth image at some evenly spaced pixel rows and columns.
 */
class DRAKERBSYSTEM_EXPORT RigidBodyDepthSensor : public RigidBodySensor {
  public:
    RigidBodyDepthSensor(RigidBodySystem const& sys,
                         const std::string& name,
                         const std::shared_ptr<RigidBodyFrame> frame,
                         tinyxml2::XMLElement* node);
    RigidBodyDepthSensor(RigidBodySystem const& sys,
                         const std::string& name,
                         const std::shared_ptr<RigidBodyFrame> frame,
                         std::size_t samples,
                         double min_angle,
                         double max_angle,
                         double range);

    virtual ~RigidBodyDepthSensor() {}

    virtual size_t getNumOutputs() const override { return num_pixel_rows*num_pixel_cols; }
    virtual Eigen::VectorXd output(const double& t, const KinematicsCache<double>& rigid_body_state, const RigidBodySystem::InputVector<double>& u) const override;

  private:
    void cacheRaycastEndpoints();
    const std::shared_ptr<RigidBodyFrame> frame;
    double min_pitch; // minimum pitch of the camera FOV in radians
    double max_pitch; // maximum pitch of the camera FOV in radians
    double min_yaw; // minimum yaw of the sensor FOV in radians
    double max_yaw; // maximum yaw of the sensor FOV in radians
    size_t num_pixel_rows; // number of points in the image vertically (pitch)
    size_t num_pixel_cols; // number of points in the image horizontally (yaw)
    double min_range; // minimum range of the sensor in meters
    double max_range; // maximum range of the sensor in meters

    Eigen::Matrix3Xd raycast_endpoints;   // cache to avoid repeated allocation

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


  /** RigidBodyAccelerometer
   * @brief Simulates a sensor that measures linear acceleration
   */
class DRAKERBSYSTEM_EXPORT RigidBodyAccelerometer : public RigidBodySensor {
  public:
    RigidBodyAccelerometer(RigidBodySystem const& sys, const std::string& name, const std::shared_ptr<RigidBodyFrame> frame);
    virtual ~RigidBodyAccelerometer() {}

    virtual size_t getNumOutputs() const override { return 3; }
    virtual Eigen::VectorXd output(const double& t, const KinematicsCache<double>& rigid_body_state, const RigidBodySystem::InputVector<double>& u) const override;
    virtual bool isDirectFeedthrough() const override { return true; }
    void setNoiseModel(std::shared_ptr<NoiseModel<double, 3, Eigen::Vector3d>> model) {
        noise_model = model;
    }

    void setGravityCompensation(bool enable_compensation) {
      gravity_compensation = enable_compensation;
    }

  private:
    bool gravity_compensation;
    std::shared_ptr<NoiseModel<double, 3, Eigen::Vector3d>> noise_model;
    const std::shared_ptr<RigidBodyFrame> frame;
};


    /** RigidBodyGyroscope
     * @brief Simulates a sensor that measures angular rates
     */
class DRAKERBSYSTEM_EXPORT RigidBodyGyroscope : public RigidBodySensor {
  public:
    RigidBodyGyroscope(RigidBodySystem const& sys, const std::string& name, const std::shared_ptr<RigidBodyFrame> frame);
    virtual ~RigidBodyGyroscope() {}

    virtual size_t getNumOutputs() const override { return 3; }
    virtual Eigen::VectorXd output(const double& t, const KinematicsCache<double>& rigid_body_state, const RigidBodySystem::InputVector<double>& u) const override;

    void setNoiseModel(std::shared_ptr<NoiseModel<double, 3, Eigen::Vector3d>> model) {
        noise_model = model;
    }

  private:
    std::shared_ptr<NoiseModel<double, 3, Eigen::Vector3d>> noise_model;
    const std::shared_ptr<RigidBodyFrame> frame;
};

    /** RigidBodyGyroscope
     * @brief Simulates a sensor that measures angular rates
     */
class DRAKERBSYSTEM_EXPORT RigidBodyMagnetometer : public RigidBodySensor {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    RigidBodyMagnetometer(RigidBodySystem const& sys, const std::string& name, const std::shared_ptr<RigidBodyFrame> frame, double declination);
    virtual ~RigidBodyMagnetometer() {}

    virtual size_t getNumOutputs() const override { return 3; }
    virtual Eigen::VectorXd output(const double& t, const KinematicsCache<double>& rigid_body_state, const RigidBodySystem::InputVector<double>& u) const override;

    void setNoiseModel(std::shared_ptr<NoiseModel<double, 3, Eigen::Vector3d>> model) {
        noise_model = model;
    }

    void setDeclination(double magnetic_declination) {
      magnetic_north << cos(magnetic_declination), 
                        sin(magnetic_declination), 
                        0;
    }

  private:
    Eigen::Vector3d magnetic_north;
    std::shared_ptr<NoiseModel<double, 3, Eigen::Vector3d>> noise_model;
    const std::shared_ptr<RigidBodyFrame> frame;
};

 // end namespace Drake
}
#endif
