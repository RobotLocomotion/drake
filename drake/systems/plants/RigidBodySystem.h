#ifndef DRAKE_RIGIDBODYSYSTEM_H
#define DRAKE_RIGIDBODYSYSTEM_H

#include "RigidBodyTree.h"
#include "RigidBodyIK.h"  // required for resolving initial conditions (could be moved to a cpp file)

namespace Drake {

  class RigidBodySystem {
  public:
    template <typename ScalarType> using InputVector = Eigen::Matrix<ScalarType,Eigen::Dynamic,1>;
    template <typename ScalarType> using StateVector = Eigen::Matrix<ScalarType,Eigen::Dynamic,1>;
    template <typename ScalarType> using OutputVector = Eigen::Matrix<ScalarType,Eigen::Dynamic,1>;

    RigidBodySystem(const std::shared_ptr<RigidBodyTree>& rigid_body_tree) : tree(rigid_body_tree) {};
    virtual ~RigidBodySystem() {};

    template <typename ScalarType>
    StateVector<ScalarType> dynamics(const ScalarType& t, const StateVector<ScalarType>& x, const InputVector<ScalarType>& u) const {
      using namespace Eigen;
      const eigen_aligned_unordered_map<const RigidBody *, Matrix<ScalarType, 6, 1> > f_ext;

      // todo: make kinematics cache once and re-use it (but have to make one per type)
      auto nq = tree->num_positions;
      auto nv = tree->num_velocities;
      auto q = x.topRows(nq);
      auto v = x.bottomRows(nv);
      auto kinsol = tree->doKinematics(q,v);

      auto H = tree->massMatrix(kinsol);
      auto Hinv = H.inverse();  // ldlt().solve(MatrixXd::Identity(nv,nv));
      VectorXd tau = -tree->dynamicsBiasTerm(kinsol,f_ext);
      if (size(u)>0) tau += tree->B*u;

      if (tree->getNumPositionConstraints()) {
        int nc = tree->getNumPositionConstraints();
        const double alpha = 5.0;  // 1/time constant of position constraint satisfaction (see my latex rigid body notes)

        // then compute the constraint force
        VectorXd phi = tree->positionConstraints(kinsol);
        MatrixXd J = tree->positionConstraintsJacobian(kinsol,false);
        VectorXd Jdotv = tree->positionConstraintsJacDotTimesV(kinsol);

        MatrixXd tmp = JacobiSVD<MatrixXd>(J*Hinv*J.transpose(),ComputeThinU | ComputeThinV).solve(MatrixXd::Identity(nc,nc));  // computes the pseudo-inverse per the discussion at http://eigen.tuxfamily.org/bz/show_bug.cgi?id=257
        VectorXd constraint_force = -J.transpose()*tmp*(J*Hinv*tau + Jdotv + 2*alpha*J*v + alpha*alpha*phi);  // adds in the computed constraint forces
        tau += constraint_force;
      }
      VectorXd vdot = Hinv*tau;

      StateVector<ScalarType> dot(nq+nv);
      dot << kinsol.transformPositionDotMappingToVelocityMapping(Eigen::Matrix<ScalarType, Eigen::Dynamic, Eigen::Dynamic>::Identity(nq, nq))*v,vdot;
      return dot;
    }

    template <typename ScalarType>
    OutputVector<ScalarType> output(const ScalarType& t, const StateVector<ScalarType>& x, const InputVector<ScalarType>& u) const {
      return x;
    }

    bool isTimeVarying() const  { return false; }
    bool isDirectFeedthrough() const { return false; }

    friend StateVector<double> getInitialState(const RigidBodySystem& sys) {

      Eigen::VectorXd x0 = Eigen::Matrix<double,Eigen::Dynamic,1>::Random(sys.tree->num_positions+sys.tree->num_velocities);

      if (sys.tree->getNumPositionConstraints()) {
        // todo: move this up to the system level?

        using namespace std;
        using namespace Eigen;

        std::vector<RigidBodyLoop,Eigen::aligned_allocator<RigidBodyLoop> > const& loops = sys.tree->loops;

        int nq = sys.tree->num_positions;
        int num_constraints = 2*loops.size();
        RigidBodyConstraint** constraint_array = new RigidBodyConstraint*[num_constraints];

        Matrix<double,7,1> bTbp = Matrix<double,7,1>::Zero();  bTbp(3)=1.0;
        Vector2d tspan; tspan<<-std::numeric_limits<double>::infinity(),std::numeric_limits<double>::infinity();
        Vector3d zero = Vector3d::Zero();
        for (int i=0; i<loops.size(); i++) {
          constraint_array[2*i] = new RelativePositionConstraint(sys.tree.get(), zero, zero, zero, loops[i].frameA->frame_index, loops[i].frameB->frame_index,bTbp,tspan);
          constraint_array[2*i+1] = new RelativePositionConstraint(sys.tree.get(), loops[i].axis, loops[i].axis, loops[i].axis, loops[i].frameA->frame_index, loops[i].frameB->frame_index,bTbp,tspan);
        }

        int info;
        vector<string> infeasible_constraint;
        IKoptions ikoptions(sys.tree.get());

        VectorXd q_guess = x0.topRows(nq);
        VectorXd q(nq);

        inverseKin(sys.tree.get(),q_guess,q_guess,num_constraints,constraint_array,q,info,infeasible_constraint,ikoptions);
        if (info>=10) {
          cout << "INFO = " << info << endl;
          cout << infeasible_constraint.size() << " infeasible constraints:";
          size_t limit = infeasible_constraint.size();
          if (limit>5) { cout << " (only printing first 5)" << endl; limit=5; }
          cout << endl;
          for (int i=0; i<limit; i++)
            cout << infeasible_constraint[i] << endl;
        }
        x0 << q, VectorXd::Zero(sys.tree->num_velocities);

        for (int i=0; i<num_constraints; i++) {
          delete constraint_array[i];
        }
        delete[] constraint_array;
      }
      return x0;
    }


  private:
    std::shared_ptr<RigidBodyTree> tree;

  };


} // end namespace Drake

#endif