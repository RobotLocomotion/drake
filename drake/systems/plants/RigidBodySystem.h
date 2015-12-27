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
      const eigen_aligned_unordered_map<const RigidBody *, Eigen::Matrix<ScalarType, 6, 1> > f_ext;

      // todo: make kinematics cache once and re-use it (but have to make one per type)
      auto kinsol = tree->doKinematics(x.topRows(tree->num_positions),x.bottomRows(tree->num_velocities));

      auto H = tree->massMatrix(kinsol);
      Eigen::VectorXd tau = -tree->dynamicsBiasTerm(kinsol,f_ext);
      if (size(u)>0) tau += tree->B*u;

      Eigen::VectorXd vdot = H.fullPivHouseholderQr().solve(tau);

      StateVector<ScalarType> dot(tree->num_positions+tree->num_velocities);
      dot << kinsol.transformVelocityMappingToPositionDotMapping(x.bottomRows(tree->num_velocities).transpose()).transpose(),vdot;
      return dot;
    }

    template <typename ScalarType>
    OutputVector<ScalarType> output(const ScalarType& t, const StateVector<ScalarType>& x, const InputVector<ScalarType>& u) const {
      return x;
    }

    bool isTimeVarying() const  { return false; }
    bool isDirectFeedthrough() const { return false; }

  private:
    std::shared_ptr<RigidBodyTree> tree;

  };

} // end namespace Drake

#endif