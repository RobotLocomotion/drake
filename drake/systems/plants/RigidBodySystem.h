#ifndef DRAKE_RIGIDBODYSYSTEM_H
#define DRAKE_RIGIDBODYSYSTEM_H

#include "System.h"
#include "Optimization.h"
#include "RigidBodyTree.h"
#include "KinematicsCache.h"

class TiXmlElement;

namespace Drake {

  class RigidBodySystem; // forward declaration

  /** RigidBodyPropellor
   * @concept{system_concept}
   * @brief System to model the forces and moments produced by a simple propellor
   *
   * note that some of this logic will move to a base class (or concept) for RigidBodyForceElements,
   * but I've chosen to handle the single type case before writing the general one
   */
  class RigidBodyPropellor {
  public:
    template <typename ScalarType> using StateVector = NullVector<ScalarType>;
    template <typename ScalarType> using InputVector = Eigen::Matrix<ScalarType,1,1>;
    template <typename ScalarType> using OutputVector = Eigen::Matrix<ScalarType,6,1>;

    RigidBodyPropellor(RigidBodySystem* sys, TiXmlElement* node, std::string name);

    // some quick thoughts:
    // might want to be nonlinear in the robot state, but linear in the prop input.
    // probably means I want to separate out those inputs
    // and that I want a more general way to specify the input-output relationships for miso functions

    // todo: replace the KinematicCache with RigidBodySystem::State once that transition occurs
    template <typename ScalarType> StateVector<ScalarType> dynamics(const ScalarType& t, const StateVector<ScalarType>& x, const InputVector<ScalarType>& u, const KinematicsCache<ScalarType>& rigid_body_state) {
      return StateVector<ScalarType>();  // no dynamics
    }

    template <typename ScalarType> OutputVector<ScalarType> output(const ScalarType& t, const StateVector<ScalarType>& x, const InputVector<ScalarType>& u, const KinematicsCache<ScalarType>& rigid_body_state) {
      OutputVector<ScalarType> f_ext;
      f_ext << scale_factor_moment*u(0)*axis, scale_factor_thrust*u(0)*axis;
      return f_ext;
    }

    RigidBodyFrame* getFrame() { return frame.get(); }

  private:
    RigidBodySystem* sys;
    std::shared_ptr<RigidBodyFrame> frame;
    Eigen::Vector3d axis;
    double scale_factor_thrust; // scale factor between input and thrust
    double scale_factor_moment; // scale factor between input and moment due to aerodynamic drag
    double lower_limit;
    double upper_limit;
    std::string name;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  /** RigidBodySystem
   * @brief implements the System concept by wrapping the RigidBodyTree algorithms with additional sensors and actuators/forces
   * @concept{system_concept}
   */
  class RigidBodySystem {
  public:
    template <typename ScalarType> using InputVector = Eigen::Matrix<ScalarType,Eigen::Dynamic,1>;
    template <typename ScalarType> using StateVector = Eigen::Matrix<ScalarType,Eigen::Dynamic,1>;
    template <typename ScalarType> using OutputVector = Eigen::Matrix<ScalarType,Eigen::Dynamic,1>;

    RigidBodySystem(const std::shared_ptr<RigidBodyTree>& rigid_body_tree) : tree(rigid_body_tree) {};
    RigidBodySystem(const std::string &urdf_filename, const DrakeJoint::FloatingBaseType floating_base_type = DrakeJoint::QUATERNION) {
      //tree = std::allocate_shared<RigidBodyTree>(Eigen::aligned_allocator<RigidBodyTree>()); // this crashed g++-4.7
      tree = std::shared_ptr<RigidBodyTree>(new RigidBodyTree());
      addRobotFromURDF(urdf_filename, floating_base_type);
    }
    virtual ~RigidBodySystem() {};

    void addRobotFromURDFString(const std::string &xml_string, const std::string &root_dir = ".", const DrakeJoint::FloatingBaseType floating_base_type = DrakeJoint::ROLLPITCHYAW);
    void addRobotFromURDF(const std::string &urdf_filename, const DrakeJoint::FloatingBaseType floating_base_type = DrakeJoint::QUATERNION);

    // note: will generalize this soon
    void addForceElement(const std::shared_ptr<RigidBodyPropellor>& prop) { props.push_back(prop); }

    const std::shared_ptr<RigidBodyTree>& getRigidBodyTree(void) { return tree; }
    size_t getNumStates(void) const { return tree->num_positions + tree->num_velocities; }
    size_t getNumInputs(void) const { return tree->actuators.size() + props.size(); }
    size_t getNumOutputs(void) const { return getNumStates(); }

    /** dynamics
     * Formulates the forward dynamics of the rigid body system as an optimization
     *   find vdot, f  (feasibility problem ok for now => implicit objective is min norm solution)
     *   subject to
     *       joint limit constraints (differentiated twice + stabilization term):    A vdot = b
     *       position equality constraints (differentiated twice + stabilization):   A vdot = b
     *       velocity equality constraints (differentiated once + stabilization):    A vdot = b
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

    friend StateVector<double> getInitialState(const RigidBodySystem& sys);

  private:
    std::shared_ptr<RigidBodyTree> tree;
    std::vector<std::shared_ptr<RigidBodyPropellor> > props;  // note: will generalize this soon

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // end namespace Drake

#endif
