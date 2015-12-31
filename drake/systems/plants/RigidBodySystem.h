#ifndef DRAKE_RIGIDBODYSYSTEM_H
#define DRAKE_RIGIDBODYSYSTEM_H

#include "System.h"
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

    template <typename ScalarType>
    StateVector<ScalarType> dynamics(const ScalarType& t, const StateVector<ScalarType>& x, const InputVector<ScalarType>& u) const {
      using namespace std;
      using namespace Eigen;
      eigen_aligned_unordered_map<const RigidBody *, Matrix<ScalarType, 6, 1> > f_ext;

      // todo: make kinematics cache once and re-use it (but have to make one per type)
      auto nq = tree->num_positions;
      auto nv = tree->num_velocities;
      auto num_actuators = tree->actuators.size();
      auto q = x.topRows(nq);
      auto v = x.bottomRows(nv);
      auto kinsol = tree->doKinematics(q,v);

      auto H = tree->massMatrix(kinsol);

      const NullVector<ScalarType> force_state;  // todo:  will have to handle this case

      { // loop through rigid body force elements and accumulate f_ext
        int u_index = 0;
        for (auto prop : props) {
          RigidBodyFrame* frame = prop->getFrame();
          RigidBody* body = frame->body.get();
          int num_inputs = 1;  // todo: generalize this
          RigidBodyPropellor::InputVector<ScalarType> u_i(u.middleRows(u_index,num_inputs));
          // todo: push the frame to body transform into the dynamicsBias method?
          Matrix<ScalarType,6,1> f_ext_i = transformSpatialForce(frame->transform_to_body,prop->output(t,force_state,u_i,kinsol));
          if (f_ext.find(body)==f_ext.end()) f_ext[body] = f_ext_i;
          else f_ext[body] = f_ext[body]+f_ext_i;
          u_index += num_inputs;
        }
      }

      MatrixXd Hinv = H.ldlt().solve(MatrixXd::Identity(nv,nv));
      VectorXd tau = -tree->dynamicsBiasTerm(kinsol,f_ext);
      if (num_actuators > 0) tau += tree->B*u.topRows(num_actuators);

      // Formulate the forward dynamics as an optimization
      //   find vdot, f  (feasibility problem ok for now => implicit objective is min norm solution)
      //   subject to
      //       position equality constraints (differentiated twice):   A vdot = b
      //       velocity equality constraints (differentiated once):   A vdot = b
      //       contact force constraints on vdot,f.  can be linear, nonlinear, even complementarity.  may have inequalities
      //   important (common) special case of all linear equality constraints can be solved with a pseudo-inverse

      { // contact dynamics
        VectorXd phi;
        Matrix3Xd normal, xA, xB;
        vector<int> bodyA_idx, bodyB_idx, bodies_idx;
        tree->collisionDetect(kinsol,phi,normal,xA,xB,bodyA_idx,bodyB_idx,bodies_idx);
      }


      if (tree->getNumPositionConstraints()) {
        int nc = tree->getNumPositionConstraints();
        const double alpha = 5.0;  // 1/time constant of position constraint satisfaction (see my latex rigid body notes)

        // then compute the constraint force
        auto phi = tree->positionConstraints(kinsol);
        auto J = tree->positionConstraintsJacobian(kinsol,false);
        auto Jdotv = tree->positionConstraintsJacDotTimesV(kinsol);

        MatrixXd tmp = JacobiSVD<MatrixXd>(J*Hinv*J.transpose(),ComputeThinU | ComputeThinV).solve(MatrixXd::Identity(nc,nc));  // computes the pseudo-inverse per the discussion at http://eigen.tuxfamily.org/bz/show_bug.cgi?id=257
        tau.noalias() += -J.transpose()*tmp*(J*Hinv*tau + Jdotv + 2*alpha*J*v + alpha*alpha*phi);  // adds in the computed constraint forces
      }
      StateVector<ScalarType> dot(nq+nv);
      dot << kinsol.transformPositionDotMappingToVelocityMapping(Eigen::Matrix<ScalarType, Eigen::Dynamic, Eigen::Dynamic>::Identity(nq, nq))*v, Hinv*tau;
      return dot;
    }

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
