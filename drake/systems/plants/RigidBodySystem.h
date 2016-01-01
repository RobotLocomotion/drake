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
        for (auto const & prop : props) {
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

      VectorXd C = tree->dynamicsBiasTerm(kinsol,f_ext);
      if (num_actuators > 0) C -= tree->B*u.topRows(num_actuators);

      { // apply contact forces
        VectorXd phi;
        Matrix3Xd normal, xA, xB;
        vector<int> bodyA_idx, bodyB_idx;
        tree->potentialCollisions(kinsol,phi,normal,xA,xB,bodyA_idx,bodyB_idx);  // note: can replace with collisionDetect for (faster) single point per collision pair

        for (int i=0; i<phi.rows(); i++) {
          if (phi(i)<0.0) { // then i have contact
            // spring law for normal force:  fA = (-k*phi - b*phidot)*normal
            double k = 500, b = k/10;  // todo: put these somewhere better... or make them parameters?
            auto JA = tree->forwardKinJacobian(kinsol,xA.col(i),bodyA_idx[i],0,0,false);
            auto JB = tree->forwardKinJacobian(kinsol,xB.col(i),bodyB_idx[i],0,0,false);
            auto relative_velocity = (JA-JB)*v;
            auto phidot = relative_velocity.dot(normal.col(i));
            auto fA_normal = -k*phi(i) - b*phidot;

            // Coulomb sliding friction (no static friction yet, because it is a complementarity problem):
            auto tangential_velocity = relative_velocity-phidot*normal.col(i);
            double mu = 1.0; // todo: make this a parameter
            auto fA = fA_normal*normal.col(i)- mu*fA_normal*tangential_velocity/(tangential_velocity.norm() + 1e-12);  // 1e-12 to avoid divide by zero

            // equal and opposite: fB = -fA.
            // tau = JA^T fA + JB^fB
            C -= JA.transpose()*fA - JB.transpose()*fA;
          }
        }
      }

      // todo: lots of code performance optimization possible here... I'm doing an exhorbitant amount of identical allocations on every function evaluation.  Just keeping it simple at first.
      OptimizationProblem prog;
      auto const & vdot = prog.addContinuousVariables(nv,"vdot");

      Eigen::MatrixXd H_and_neg_JT = H;
      if (tree->getNumPositionConstraints()) {
        int nc = tree->getNumPositionConstraints();
        const double alpha = 5.0;  // 1/time constant of position constraint satisfaction (see my latex rigid body notes)

        prog.addContinuousVariables(nc,"position constraint force");  // don't actually need to use the decision variable reference that would be returned...

        // then compute the constraint force
        auto phi = tree->positionConstraints(kinsol);
        auto J = tree->positionConstraintsJacobian(kinsol,false);
        auto Jdotv = tree->positionConstraintsJacDotTimesV(kinsol);

        // phiddot = -2 alpha phidot - alpha^2 phi  (0 + critically damped stabilization term)
        prog.addLinearEqualityConstraint(J,-(Jdotv + 2*alpha*J*v + alpha*alpha*phi),vdot);
        H_and_neg_JT.conservativeResize(NoChange,H_and_neg_JT.cols()+J.rows());
        H_and_neg_JT.rightCols(J.rows()) = -J.transpose();
      }

      // add [H,-J^T]*[vdot;f] = -C
      prog.addLinearEqualityConstraint(H_and_neg_JT,-C);

      prog.solve();
//      prog.printSolution();

      StateVector<ScalarType> dot(nq+nv);
      dot << kinsol.transformPositionDotMappingToVelocityMapping(Eigen::Matrix<ScalarType, Eigen::Dynamic, Eigen::Dynamic>::Identity(nq, nq))*v, vdot.value;
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
