#include "drake/matlab/systems/plants/rigidBodyTreeMexFunctions.h"

#include <typeinfo>
#include <Eigen/Sparse>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/matlab/systems/plants/rigidBodyTreeMexConversions.h"
#include "drake/matlab/util/makeFunction.h"
#include "drake/matlab/util/standardMexConversions.h"
#include "drake/systems/plants/RigidBodyTree.h"

using namespace std;
using namespace Eigen;
using namespace drake;

typedef AutoDiffScalar<VectorXd> AutoDiffDynamicSize;
typedef DrakeJoint::AutoDiffFixedMaxSize AutoDiffFixedMaxSize;

/**
 * Mex function implementations
 */
void centerOfMassJacobianDotTimesVmex(int nlhs, mxArray* plhs[], int nrhs,
                                      const mxArray* prhs[]) {
  auto func_double =
      make_function(&RigidBodyTree::centerOfMassJacobianDotTimesV<double>);
  auto func_autodiff_fixed_max = make_function(
      &RigidBodyTree::centerOfMassJacobianDotTimesV<AutoDiffFixedMaxSize>);
  auto func_autodiff_dynamic = make_function(
      &RigidBodyTree::centerOfMassJacobianDotTimesV<AutoDiffDynamicSize>);
  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, true, func_double,
                        func_autodiff_fixed_max, func_autodiff_dynamic);
}

void centerOfMassmex(int nlhs, mxArray* plhs[], int nrhs,
                     const mxArray* prhs[]) {
  auto func_double = make_function(&RigidBodyTree::centerOfMass<double>);
  auto func_autodiff_fixed_max =
      make_function(&RigidBodyTree::centerOfMass<AutoDiffFixedMaxSize>);
  auto func_autodiff_dynamic =
      make_function(&RigidBodyTree::centerOfMass<AutoDiffDynamicSize>);
  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, true, func_double,
                        func_autodiff_fixed_max, func_autodiff_dynamic);
}

void centerOfMassJacobianmex(int nlhs, mxArray* plhs[], int nrhs,
                             const mxArray* prhs[]) {
  auto func_double =
      make_function(&RigidBodyTree::centerOfMassJacobian<double>);
  auto func_autodiff_fixed_max =
      make_function(&RigidBodyTree::centerOfMassJacobian<AutoDiffFixedMaxSize>);
  auto func_autodiff_dynamic =
      make_function(&RigidBodyTree::centerOfMassJacobian<AutoDiffDynamicSize>);
  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, true, func_double,
                        func_autodiff_fixed_max, func_autodiff_dynamic);
}

void centroidalMomentumMatrixDotTimesvmex(int nlhs, mxArray* plhs[], int nrhs,
                                          const mxArray* prhs[]) {
  auto func_double =
      make_function(&RigidBodyTree::centroidalMomentumMatrixDotTimesV<double>);
  auto func_autodiff_fixed_max = make_function(
      &RigidBodyTree::centroidalMomentumMatrixDotTimesV<AutoDiffFixedMaxSize>);
  auto func_autodiff_dynamic = make_function(
      &RigidBodyTree::centroidalMomentumMatrixDotTimesV<AutoDiffDynamicSize>);
  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, true, func_double,
                        func_autodiff_fixed_max, func_autodiff_dynamic);
}

void centroidalMomentumMatrixmex(int nlhs, mxArray* plhs[], int nrhs,
                                 const mxArray* prhs[]) {
  auto func_double =
      make_function(&RigidBodyTree::centroidalMomentumMatrix<double>);
  auto func_autodiff_fixed_max = make_function(
      &RigidBodyTree::centroidalMomentumMatrix<AutoDiffFixedMaxSize>);
  auto func_autodiff_dynamic = make_function(
      &RigidBodyTree::centroidalMomentumMatrix<AutoDiffDynamicSize>);
  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, true, func_double,
                        func_autodiff_fixed_max, func_autodiff_dynamic);
}

template <typename DerivedQ, typename DerivedV>
void doKinematicsTemp(const RigidBodyTree& model,
                      KinematicsCache<typename DerivedQ::Scalar>& cache,
                      const MatrixBase<DerivedQ>& q,
                      const MatrixBase<DerivedV>& v, bool compute_JdotV) {
  // temporary solution. Explicit doKinematics calls will not be necessary in
  // the near future.
  if (v.size() == 0 && model.get_num_velocities() != 0)
    cache.initialize(q);
  else
    cache.initialize(q, v);
  model.doKinematics(cache, compute_JdotV);
}

void doKinematicsmex(int nlhs, mxArray* plhs[], int nrhs,
                     const mxArray* prhs[]) {
  auto func_double = make_function(
      &doKinematicsTemp<Map<const VectorXd>, Map<const VectorXd>>);

  typedef Matrix<AutoDiffFixedMaxSize, Dynamic, 1> VectorXAutoDiffFixedMax;
  auto func_autodiff_fixed_max = make_function(
      &doKinematicsTemp<VectorXAutoDiffFixedMax, VectorXAutoDiffFixedMax>);

  typedef Matrix<AutoDiffDynamicSize, Dynamic, 1> VectorXAutoDiffDynamic;
  auto func_autodiff_dynamic = make_function(
      &doKinematicsTemp<VectorXAutoDiffDynamic, VectorXAutoDiffDynamic>);

  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, true, func_double,
                        func_autodiff_fixed_max, func_autodiff_dynamic);
}

void findKinematicPathmex(int nlhs, mxArray* plhs[], int nrhs,
                          const mxArray* prhs[]) {
  auto func = make_function(&RigidBodyTree::findKinematicPath);
  mexCallFunction(nlhs, plhs, nrhs, prhs, true, func);
}

template <typename Scalar, typename DerivedPoints>
Matrix<Scalar, Dynamic, DerivedPoints::ColsAtCompileTime>
forwardJacDotTimesVTemp(const RigidBodyTree& tree,
                        const KinematicsCache<Scalar>& cache,
                        const MatrixBase<DerivedPoints>& points,
                        int current_body_or_frame_ind,
                        int new_body_or_frame_ind, int rotation_type) {
  auto Jtransdot_times_v = tree.transformPointsJacobianDotTimesV(
      cache, points, current_body_or_frame_ind, new_body_or_frame_ind);
  if (rotation_type == 0) {
    return Jtransdot_times_v;
  } else {
    Matrix<Scalar, Dynamic, 1> Jrotdot_times_v(
        rotationRepresentationSize(rotation_type));
    if (rotation_type == 1) {
      Jrotdot_times_v = tree.relativeRollPitchYawJacobianDotTimesV(
          cache, current_body_or_frame_ind, new_body_or_frame_ind);
    } else if (rotation_type == 2) {
      Jrotdot_times_v = tree.relativeQuaternionJacobianDotTimesV(
          cache, current_body_or_frame_ind, new_body_or_frame_ind);
    } else {
      throw runtime_error("rotation type not recognized");
    }

    Matrix<Scalar, Dynamic, 1> Jdot_times_v(
        (3 + rotationRepresentationSize(rotation_type)) * points.cols(), 1);

    int row_start = 0;
    for (int i = 0; i < points.cols(); i++) {
      Jdot_times_v.template middleRows<3>(row_start) =
          Jtransdot_times_v.template middleRows<3>(3 * i);
      row_start += 3;

      Jdot_times_v.middleRows(row_start, Jrotdot_times_v.rows()) =
          Jrotdot_times_v;
      row_start += Jrotdot_times_v.rows();
    }
    return Jdot_times_v;
  }
}

void forwardJacDotTimesVmex(int nlhs, mxArray* plhs[], int nrhs,
                            const mxArray* prhs[]) {
  typedef Map<const Matrix3Xd> DerivedPoints;
  auto func_double =
      make_function(&forwardJacDotTimesVTemp<double, DerivedPoints>);
  auto func_autodiff_fixed_max = make_function(
      &forwardJacDotTimesVTemp<AutoDiffFixedMaxSize, DerivedPoints>);
  auto func_autodiff_dynamic = make_function(
      &forwardJacDotTimesVTemp<AutoDiffDynamicSize, DerivedPoints>);

  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, true, func_double,
                        func_autodiff_fixed_max, func_autodiff_dynamic);
}

template <typename Scalar, typename DerivedPoints>
Matrix<Scalar, Dynamic, DerivedPoints::ColsAtCompileTime> forwardKinTemp(
    const RigidBodyTree& tree, const KinematicsCache<Scalar>& cache,
    const MatrixBase<DerivedPoints>& points, int current_body_or_frame_ind,
    int new_body_or_frame_ind, int rotation_type) {
  Matrix<Scalar, Dynamic, DerivedPoints::ColsAtCompileTime> ret(
      3 + rotationRepresentationSize(rotation_type), points.cols());
  ret.template topRows<3>() = tree.transformPoints(
      cache, points, current_body_or_frame_ind, new_body_or_frame_ind);
  if (rotation_type == 1) {
    ret.template bottomRows<3>().colwise() = tree.relativeRollPitchYaw(
        cache, current_body_or_frame_ind, new_body_or_frame_ind);
  } else if (rotation_type == 2) {
    ret.template bottomRows<4>().colwise() = tree.relativeQuaternion(
        cache, current_body_or_frame_ind, new_body_or_frame_ind);
  }
  return ret;
}

void forwardKinmex(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
  auto func_double =
      make_function(&forwardKinTemp<double, Map<const Matrix3Xd>>);
  auto func_autodiff_fixed_max_double_points = make_function(
      &forwardKinTemp<AutoDiffFixedMaxSize, Map<const Matrix3Xd>>);
  auto func_autodiff_fixed_max_autodiff_points =
      make_function(&forwardKinTemp<AutoDiffFixedMaxSize,
                                    Matrix<AutoDiffFixedMaxSize, 3, Dynamic>>);
  auto func_autodiff_dynamic_double_points =
      make_function(&forwardKinTemp<AutoDiffDynamicSize, Map<const Matrix3Xd>>);
  auto func_autodiff_dynamic_autodiff_points =
      make_function(&forwardKinTemp<AutoDiffDynamicSize,
                                    Matrix<AutoDiffDynamicSize, 3, Dynamic>>);

  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, true, func_double,
                        func_autodiff_fixed_max_double_points,
                        func_autodiff_fixed_max_autodiff_points,
                        func_autodiff_dynamic_double_points,
                        func_autodiff_dynamic_autodiff_points);
}

template <typename Scalar, typename DerivedPoints>
Matrix<Scalar, Dynamic, Dynamic> forwardKinJacobianTemp(
    const RigidBodyTree& tree, const KinematicsCache<Scalar>& cache,
    const MatrixBase<DerivedPoints>& points, int current_body_or_frame_ind,
    int new_body_or_frame_ind, int rotation_type, bool in_terms_of_qdot) {
  auto Jtrans =
      tree.transformPointsJacobian(cache, points, current_body_or_frame_ind,
                                   new_body_or_frame_ind, in_terms_of_qdot);
  if (rotation_type == 0) {
    return Jtrans;
  } else {
    Matrix<Scalar, Dynamic, Dynamic> Jrot(
        rotationRepresentationSize(rotation_type), Jtrans.cols());
    if (rotation_type == 1)
      Jrot = tree.relativeRollPitchYawJacobian(cache, current_body_or_frame_ind,
                                               new_body_or_frame_ind,
                                               in_terms_of_qdot);
    else if (rotation_type == 2)
      Jrot = tree.relativeQuaternionJacobian(cache, current_body_or_frame_ind,
                                             new_body_or_frame_ind,
                                             in_terms_of_qdot);
    else
      throw runtime_error("rotation_type not recognized");
    Matrix<Scalar, Dynamic, Dynamic> J((3 + Jrot.rows()) * points.cols(),
                                       Jtrans.cols());
    int row_start = 0;
    for (int i = 0; i < points.cols(); i++) {
      J.template middleRows<3>(row_start) =
          Jtrans.template middleRows<3>(3 * i);
      row_start += 3;

      J.middleRows(row_start, Jrot.rows()) = Jrot;
      row_start += Jrot.rows();
    }
    return J;
  }
}

void forwardKinJacobianmex(int nlhs, mxArray* plhs[], int nrhs,
                           const mxArray* prhs[]) {
  typedef Map<const Matrix3Xd> DerivedPoints;
  auto func_double =
      make_function(&forwardKinJacobianTemp<double, DerivedPoints>);
  auto func_autodiff_fixed_max = make_function(
      &forwardKinJacobianTemp<AutoDiffFixedMaxSize, DerivedPoints>);
  auto func_autodiff_dynamic = make_function(
      &forwardKinJacobianTemp<AutoDiffDynamicSize, DerivedPoints>);

  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, true, func_double,
                        func_autodiff_fixed_max, func_autodiff_dynamic);
}

void forwardKinPositionGradientmex(int nlhs, mxArray* plhs[], int nrhs,
                                   const mxArray* prhs[]) {
  auto func_double =
      make_function(&RigidBodyTree::forwardKinPositionGradient<double>);
  auto func_autodiff_fixed_max = make_function(
      &RigidBodyTree::forwardKinPositionGradient<AutoDiffFixedMaxSize>);
  auto func_autodiff_dynamic = make_function(
      &RigidBodyTree::forwardKinPositionGradient<AutoDiffDynamicSize>);

  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, true, func_double,
                        func_autodiff_fixed_max, func_autodiff_dynamic);
}

void geometricJacobianDotTimesVmex(int nlhs, mxArray* plhs[], int nrhs,
                                   const mxArray* prhs[]) {
  auto func_double =
      make_function(&RigidBodyTree::geometricJacobianDotTimesV<double>);
  auto func_autodiff_fixed_max = make_function(
      &RigidBodyTree::geometricJacobianDotTimesV<AutoDiffFixedMaxSize>);
  auto func_autodiff_dynamic = make_function(
      &RigidBodyTree::geometricJacobianDotTimesV<AutoDiffDynamicSize>);

  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, true, func_double,
                        func_autodiff_fixed_max, func_autodiff_dynamic);
}

template <typename Scalar>
drake::TwistMatrix<Scalar> geometricJacobianTemp(
    const RigidBodyTree& model, const KinematicsCache<Scalar>& cache,
    int base_body_or_frame_ind, int end_effector_body_or_frame_ind,
    int expressed_in_body_or_frame_ind, bool in_terms_of_qdot) {
  // temporary solution. Gross v_or_qdot_indices pointer will be gone soon.
  return model.geometricJacobian(
      cache, base_body_or_frame_ind, end_effector_body_or_frame_ind,
      expressed_in_body_or_frame_ind, in_terms_of_qdot, nullptr);
}

void geometricJacobianmex(int nlhs, mxArray* plhs[], int nrhs,
                          const mxArray* prhs[]) {
  auto func_double = make_function(&geometricJacobianTemp<double>);
  auto func_autodiff_fixed_max =
      make_function(&geometricJacobianTemp<AutoDiffFixedMaxSize>);
  auto func_autodiff_dynamic =
      make_function(&geometricJacobianTemp<AutoDiffDynamicSize>);
  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, true, func_double,
                        func_autodiff_fixed_max, func_autodiff_dynamic);
}

void massMatrixmex(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
  auto func_double = make_function(&RigidBodyTree::massMatrix<double>);
  auto func_autodiff_fixed_max =
      make_function(&RigidBodyTree::massMatrix<AutoDiffFixedMaxSize>);
  auto func_autodiff_dynamic =
      make_function(&RigidBodyTree::massMatrix<AutoDiffDynamicSize>);
  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, true, func_double,
                        func_autodiff_fixed_max, func_autodiff_dynamic);
}

template <typename Scalar, typename DerivedF>
Matrix<Scalar, Dynamic, 1> dynamicsBiasTermTemp(
    const RigidBodyTree& model, KinematicsCache<Scalar>& cache,
    const MatrixBase<DerivedF>& f_ext_value) {
  // temporary solution.

  RigidBodyTree::BodyToWrenchMap<Scalar> external_wrenches;
  if (f_ext_value.size() > 0) {
    DRAKE_ASSERT(f_ext_value.cols() == model.bodies.size());
    for (Eigen::Index i = 0; i < f_ext_value.cols(); i++) {
      external_wrenches.insert({model.bodies[i].get(), f_ext_value.col(i)});
    }
  }

  return model.dynamicsBiasTerm(cache, external_wrenches);
}

void dynamicsBiasTermmex(int nlhs, mxArray* plhs[], int nrhs,
                         const mxArray* prhs[]) {
  auto func_double = make_function(
      &dynamicsBiasTermTemp<double, Map<const Matrix<double, 6, Dynamic>>>);
  auto func_autodiff_fixed_max = make_function(
      &dynamicsBiasTermTemp<AutoDiffFixedMaxSize,
                            Matrix<AutoDiffFixedMaxSize, 6, Dynamic>>);
  auto func_autodiff_dynamic = make_function(
      &dynamicsBiasTermTemp<AutoDiffDynamicSize,
                            Matrix<AutoDiffDynamicSize, 6, Dynamic>>);

  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, true, func_double,
                        func_autodiff_fixed_max, func_autodiff_dynamic);
}

template <typename Scalar>
Matrix<Scalar, Dynamic, Dynamic> velocityToPositionDotMapping(
    const KinematicsCache<Scalar>& cache) {
  auto nq = cache.get_num_positions();
  return cache.transformPositionDotMappingToVelocityMapping(
      Matrix<Scalar, Dynamic, Dynamic>::Identity(nq, nq));
}

template <typename Scalar>
Matrix<Scalar, Dynamic, Dynamic> positionDotToVelocityMapping(
    const KinematicsCache<Scalar>& cache) {
  auto nv = cache.get_num_velocities();
  return cache.transformVelocityMappingToPositionDotMapping(
      Matrix<Scalar, Dynamic, Dynamic>::Identity(nv, nv));
}

void velocityToPositionDotMappingmex(int nlhs, mxArray* plhs[], int nrhs,
                                     const mxArray* prhs[]) {
  auto func_double = make_function(&velocityToPositionDotMapping<double>);
  auto func_autodiff_fixed_max =
      make_function(&velocityToPositionDotMapping<AutoDiffFixedMaxSize>);
  auto func_autodiff_dynamic =
      make_function(&velocityToPositionDotMapping<AutoDiffDynamicSize>);
  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, true, func_double,
                        func_autodiff_fixed_max, func_autodiff_dynamic);
}

void positionDotToVelocityMappingmex(int nlhs, mxArray* plhs[], int nrhs,
                                     const mxArray* prhs[]) {
  auto func_double = make_function(&positionDotToVelocityMapping<double>);
  auto func_autodiff_fixed_max =
      make_function(&positionDotToVelocityMapping<AutoDiffFixedMaxSize>);
  auto func_autodiff_dynamic =
      make_function(&positionDotToVelocityMapping<AutoDiffDynamicSize>);
  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, true, func_double,
                        func_autodiff_fixed_max, func_autodiff_dynamic);
}
