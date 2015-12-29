#ifndef DRAKE_RIGIDBODYSYSTEM_H
#define DRAKE_RIGIDBODYSYSTEM_H

#include "RigidBodyTree.h"

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
      MatrixXd Hinv = H.ldlt().solve(MatrixXd::Identity(nv,nv));
      VectorXd tau = -tree->dynamicsBiasTerm(kinsol,f_ext);
      if (size(u)>0) tau += tree->B*u;

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

  };


} // end namespace Drake

#endif