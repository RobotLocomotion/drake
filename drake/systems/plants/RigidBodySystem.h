#ifndef DRAKE_RIGIDBODYSYSTEM_H
#define DRAKE_RIGIDBODYSYSTEM_H

#include "drake/systems/System.h"
#include "drake/solvers/Optimization.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "KinematicsCache.h"
#include "drake/drakeRBSystem_export.h"

namespace tinyxml2 {
  class XMLElement;
}

namespace Drake {

  class RigidBodyForceElement; // forward declaration

  namespace RigidBodyConstraints {
    /** @defgroup rigid_body_constraint RigidBodyConstraint Concept
     * @ingroup concepts
     * @{
     * @brief A constraint that can be updated using the state of the rigid body system
     * @nbsp
     *
     * | Valid Expressions (which must be implemented) |  |
     * ------------------|-------------------------------------------------------------|
     * | template <typename ScalarType> updateConstraint(const KinematicCache<ScalarType>& kinsol)  | Updates the parameters of the constraint |
     * | Eigen::MatrixXd constraintForceJacobian() | returns the J used in J^T force for any constraint forces implied
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
   * @brief implements the System concept by wrapping the RigidBodyTree algorithms with additional sensors and actuators/forces
   * @concept{system_concept}
   */
  class DRAKERBSYSTEM_EXPORT RigidBodySystem {
  public:
    template <typename ScalarType> using InputVector = Eigen::Matrix<ScalarType,Eigen::Dynamic,1>;
    template <typename ScalarType> using StateVector = Eigen::Matrix<ScalarType,Eigen::Dynamic,1>;
    template <typename ScalarType> using OutputVector = Eigen::Matrix<ScalarType,Eigen::Dynamic,1>;

    RigidBodySystem(const std::shared_ptr<RigidBodyTree>& rigid_body_tree) : tree(rigid_body_tree),
                                                                             use_multi_contact(false),
                                                                             penetration_stiffness(150.0),
                                                                             penetration_damping(penetration_stiffness/10.0),
                                                                             friction_coefficient(1.0) {};
    RigidBodySystem(const std::string &urdf_filename, const DrakeJoint::FloatingBaseType floating_base_type = DrakeJoint::QUATERNION) :
            use_multi_contact(false),
            penetration_stiffness(150.0),
            penetration_damping(penetration_stiffness/10.0),
            friction_coefficient(1.0)
    {
      //tree = std::allocate_shared<RigidBodyTree>(Eigen::aligned_allocator<RigidBodyTree>()); // this crashed g++-4.7
      tree = std::shared_ptr<RigidBodyTree>(new RigidBodyTree());
      addRobotFromURDF(urdf_filename, floating_base_type);
    }
    virtual ~RigidBodySystem() {};

    void addRobotFromURDFString(const std::string &xml_string, const std::string &root_dir = ".", const DrakeJoint::FloatingBaseType floating_base_type = DrakeJoint::ROLLPITCHYAW);
    void addRobotFromURDF(const std::string &urdf_filename, const DrakeJoint::FloatingBaseType floating_base_type = DrakeJoint::QUATERNION);

    // note: will generalize this soon
    void addForceElement(const std::shared_ptr<RigidBodyForceElement>& f) { force_elements.push_back(f); }

    const std::shared_ptr<RigidBodyTree>& getRigidBodyTree(void) { return tree; }
    size_t getNumStates() const { return tree->num_positions + tree->num_velocities; }
    size_t getNumInputs() const;
    size_t getNumOutputs() const { return getNumStates(); }

    /** dynamics
     * Formulates the forward dynamics of the rigid body system as an optimization
     *   find vdot, f  (feasibility problem ok for now => implicit objective is min norm solution)
     *   subject to
     *       position equality constraints (differentiated twice + stabilization):   A vdot = b
     *       velocity equality constraints (differentiated once + stabilization):    A vdot = b
     *       forces from joint limits and contact OR
     *       contact force constraints on vdot,f.  can be linear, nonlinear, even complementarity.  may have inequalities
     *   the trick is that each new constraint can add decision variables (the new constraint forces and/or slack variables)
     *   to the problem, so the last constraint to add is
     *       equations of motion: H vdot + C(q,qdot,u,f_ext) = J^T(q,qdot) f
     *   where J is accumulated through the constraint logic
     *
     * The solver will then dispatch to the right tool for the job.  Note that for many systems, especially those
     * without any contact constraints (or with simple friction models), the formulation is linear and can be solved
     * with least-squares.
     */
    StateVector<double> dynamics(const double& t, const StateVector<double>& x, const InputVector<double>& u) const;

    template <typename ScalarType>
    OutputVector<ScalarType> output(const ScalarType& t, const StateVector<ScalarType>& x, const InputVector<ScalarType>& u) const {
      return x;
    }

    bool isTimeVarying() const  { return false; }
    bool isDirectFeedthrough() const { return false; }

    friend DRAKERBSYSTEM_EXPORT StateVector<double> getInitialState(const RigidBodySystem& sys);

    // some parameters defining the contact
    bool use_multi_contact;
    double penetration_stiffness;   // k
    double penetration_damping;     // b
    double friction_coefficient;    // mu

  private:
    std::shared_ptr<RigidBodyTree> tree;
    std::vector<std::shared_ptr<RigidBodyForceElement> > force_elements;

    /*
    mutable OptimizationProblem dynamics_program;
    std::list<std::shared_ptr<LinearEqualityConstraint>> dynamics_program_constraints;  // each must also implement the RigidBodyConstraint Concept
    std::list<std::shared_ptr<LinearEqualityConstraint>> conditional_dynamics_program_constraints;  // each must also implement the RigidBodyConstraint Concept
*/
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  /** RigidBodyForceElement
   * @brief interface class for elements which define a generalized force acting on the rigid body system
   */
  class DRAKERBSYSTEM_EXPORT RigidBodyForceElement {
  public:
    RigidBodyForceElement(RigidBodySystem* sys, std::string name) : sys(sys), name(name) {}
    virtual ~RigidBodyForceElement() {}

    virtual size_t getNumInputs() const { return 0; }
    virtual Eigen::VectorXd output(const double& t, /* todo: add force state here */ const Eigen::VectorXd& u, const KinematicsCache<double>& rigid_body_state) const = 0;

  protected:
    RigidBodySystem* sys;
    std::string name;
  };

  /** spatialForceInFrameToJointTorque
   * @brief helper function for rigid body force elements.  todo: move this into RigidBodyTree?
   */
  Eigen::VectorXd spatialForceInFrameToJointTorque(const RigidBodyTree* tree, const KinematicsCache<double>& rigid_body_state, const RigidBodyFrame* frame, const Eigen::Matrix<double,6,1>& force) {
    auto T_frame_to_world = tree->relativeTransform(rigid_body_state, 0, frame->frame_index);
    auto force_in_world = transformSpatialForce(T_frame_to_world, force);
    std::vector<int> v_indices;
    auto J = tree->geometricJacobian(rigid_body_state, 0, frame->frame_index, 0, false, &v_indices);
    Eigen::VectorXd tau = Eigen::VectorXd::Zero(tree->num_velocities);
    for (int i=0; i<v_indices.size(); i++) {
      tau(v_indices[i]) = J.col(i).dot(force_in_world);
//      std::cout << " f_" << tree->getVelocityName(v_indices[i]) << " = " << tau(v_indices[i]) << std::endl;
    }
    return tau;
  }

  // todo: insert a RigidBodyForceImpl with CRTP here once I go back and template these methods

  /** RigidBodyPropellor
   * @brief Models the forces and moments produced by a simple propellor
   */
  class DRAKERBSYSTEM_EXPORT RigidBodyPropellor : public RigidBodyForceElement {
  public:
    RigidBodyPropellor(RigidBodySystem* sys, tinyxml2::XMLElement* node, std::string name);
    virtual ~RigidBodyPropellor() {}

    virtual size_t getNumInputs() const override { return 1; }

    // some quick thoughts:
    // might want to be nonlinear in the robot state, but linear in the prop input.
    // probably means I want to separate out those inputs
    // and that I want a more general way to specify the input-output relationships for miso functions

    virtual Eigen::VectorXd output(const double& t, /* todo: add force state here */ const Eigen::VectorXd& u, const KinematicsCache<double>& rigid_body_state) const override {
      Eigen::Matrix<double,6,1> force;
      force << scale_factor_moment*u(0)*axis, scale_factor_thrust*u(0)*axis;
      return spatialForceInFrameToJointTorque(sys->getRigidBodyTree().get(),rigid_body_state,frame.get(),force);
    }

  private:
    std::shared_ptr<RigidBodyFrame> frame;
    Eigen::Vector3d axis;
    double scale_factor_thrust; // scale factor between input and thrust
    double scale_factor_moment; // scale factor between input and moment due to aerodynamic drag
    double lower_limit;
    double upper_limit;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  /** RigidBodySpringDamper
   * @brief Models the forces produced by a linear spring-damper
   */
  class DRAKERBSYSTEM_EXPORT RigidBodySpringDamper : public RigidBodyForceElement {
  public:
    RigidBodySpringDamper(RigidBodySystem* sys, tinyxml2::XMLElement* node, std::string name);
    virtual ~RigidBodySpringDamper() {}

    virtual Eigen::VectorXd output(const double& t, /* todo: add force state here */ const Eigen::VectorXd& u, const KinematicsCache<double>& rigid_body_state) const override {
      using namespace Eigen;
      const Vector3d origin = Vector3d::Zero();
      Vector3d xA_in_B = sys->getRigidBodyTree()->transformPoints(rigid_body_state,origin,frameA->frame_index,frameB->frame_index);
      Vector3d xB_in_A = sys->getRigidBodyTree()->transformPoints(rigid_body_state,origin,frameB->frame_index,frameA->frame_index);
      auto JA_in_B = sys->getRigidBodyTree()->transformPointsJacobian(rigid_body_state,origin,frameA->frame_index,frameB->frame_index,false);

      double length = xA_in_B.norm();
      double vel = (JA_in_B*rigid_body_state.getV()).dot(xA_in_B)/(length+EPSILON);
      double force_magnitude = stiffness*(length - rest_length) + damping*vel;

//      std::cout << "l=" << length << ", v=" << vel << ", f=" << force_magnitude << std::endl;

      Matrix<double,6,1> force = Matrix<double,6,1>::Zero();

      // apply (force_magnitude/length)*xA_in_B to B
      force.tail<3>() = (force_magnitude/(length+EPSILON))*xA_in_B;
      auto tau = spatialForceInFrameToJointTorque(sys->getRigidBodyTree().get(),rigid_body_state,frameB.get(),force);

      // apply (force_magnitude/length)*xB_in_A to A
      force.tail<3>() = (force_magnitude/(length+EPSILON))*xB_in_A;
      tau += spatialForceInFrameToJointTorque(sys->getRigidBodyTree().get(),rigid_body_state,frameA.get(),force);

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
} // end namespace Drake

#endif
