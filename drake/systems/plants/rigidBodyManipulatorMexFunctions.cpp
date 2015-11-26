#include <typeinfo>
#include <Eigen/Sparse>
#include "rigidBodyManipulatorMexFunctions.h"
#include "RigidBodyManipulator.h"
#include "standardMexConversions.h"
#include "rigidBodyManipulatorMexConversions.h"

using namespace std;
using namespace Eigen;

/**
 * make_function
 * Note that a completely general make_function implementation is not possible due to ambiguities, but this works for all of the cases in this file
 * Inspired by http://stackoverflow.com/a/21740143/2228557
 */

//plain function pointers
template<typename... Args, typename ReturnType>
auto make_function(ReturnType(*p)(Args...))
-> std::function<ReturnType(Args...)> { return {p}; }

//nonconst member function pointers
// note the ClassType& as one of the arguments, which was erroneously omitted in the SO answer above
// also note the conversion to mem_fn, needed to work around an issue with MSVC 2013
template<typename... Args, typename ReturnType, typename ClassType>
auto make_function(ReturnType(ClassType::*p)(Args...))
-> std::function<ReturnType(ClassType&, Args...)> { return {mem_fn(p)}; }

//const member function pointers
// note the const ClassType& as one of the arguments, which was erroneously omitted in the SO answer above
// also note the conversion to mem_fn, needed to work around an issue with MSVC 2013
template<typename... Args, typename ReturnType, typename ClassType>
auto make_function(ReturnType(ClassType::*p)(Args...) const)
-> std::function<ReturnType(const ClassType&, Args...)> { return {mem_fn(p)}; }

typedef AutoDiffScalar<VectorXd> AutoDiffDynamicSize;
typedef DrakeJoint::AutoDiffFixedMaxSize AutoDiffFixedMaxSize;

/**
 * Mex function implementations
 */
void centerOfMassJacobianDotTimesVmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  auto func_double = make_function(&RigidBodyManipulator::centerOfMassJacobianDotTimesV<double>);
  auto func_autodiff_fixed_max = make_function(&RigidBodyManipulator::centerOfMassJacobianDotTimesV<AutoDiffFixedMaxSize>);
  auto func_autodiff_dynamic = make_function(&RigidBodyManipulator::centerOfMassJacobianDotTimesV<AutoDiffDynamicSize>);
  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, true, func_double, func_autodiff_fixed_max, func_autodiff_dynamic);
}

void centerOfMassmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  auto func_double = make_function(&RigidBodyManipulator::centerOfMass<double>);
  auto func_autodiff_fixed_max = make_function(&RigidBodyManipulator::centerOfMass<AutoDiffFixedMaxSize>);
  auto func_autodiff_dynamic = make_function(&RigidBodyManipulator::centerOfMass<AutoDiffDynamicSize>);
  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, true, func_double, func_autodiff_fixed_max, func_autodiff_dynamic);
}

void centerOfMassJacobianmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  auto func_double = make_function(&RigidBodyManipulator::centerOfMassJacobian<double>);
  auto func_autodiff_fixed_max = make_function(&RigidBodyManipulator::centerOfMassJacobian<AutoDiffFixedMaxSize>);
  auto func_autodiff_dynamic = make_function(&RigidBodyManipulator::centerOfMassJacobian<AutoDiffDynamicSize>);
  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, true, func_double, func_autodiff_fixed_max, func_autodiff_dynamic);
}

void centroidalMomentumMatrixDotTimesvmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  auto func_double = make_function(&RigidBodyManipulator::centroidalMomentumMatrixDotTimesV<double>);
  auto func_autodiff_fixed_max = make_function(&RigidBodyManipulator::centroidalMomentumMatrixDotTimesV<AutoDiffFixedMaxSize>);
  auto func_autodiff_dynamic = make_function(&RigidBodyManipulator::centroidalMomentumMatrixDotTimesV<AutoDiffDynamicSize>);
  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, true, func_double, func_autodiff_fixed_max, func_autodiff_dynamic);
}

void centroidalMomentumMatrixmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  auto func_double = make_function(&RigidBodyManipulator::centroidalMomentumMatrix<double>);
  auto func_autodiff_fixed_max = make_function(&RigidBodyManipulator::centroidalMomentumMatrix<AutoDiffFixedMaxSize>);
  auto func_autodiff_dynamic = make_function(&RigidBodyManipulator::centroidalMomentumMatrix<AutoDiffDynamicSize>);
  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, true, func_double, func_autodiff_fixed_max, func_autodiff_dynamic);
}

template <typename DerivedQ, typename DerivedV>
void doKinematicsTemp(const RigidBodyManipulator &model, KinematicsCache<typename DerivedQ::Scalar> &cache, const MatrixBase<DerivedQ> &q, const MatrixBase<DerivedV> &v, bool compute_JdotV) {
  // temporary solution. Explicit doKinematics calls will not be necessary in the near future.
  if (v.size() == 0 && model.num_velocities != 0)
    cache.initialize(q);
  else
    cache.initialize(q, v);
  model.doKinematics(cache, compute_JdotV);
}

void doKinematicsmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  auto func_double = make_function(&doKinematicsTemp<Map<const VectorXd>, Map<const VectorXd>>);

  typedef Matrix<AutoDiffFixedMaxSize, Dynamic, 1> VectorXAutoDiffFixedMax;
  auto func_autodiff_fixed_max = make_function(&doKinematicsTemp<VectorXAutoDiffFixedMax, VectorXAutoDiffFixedMax>);

  typedef Matrix<AutoDiffDynamicSize, Dynamic, 1> VectorXAutoDiffDynamic;
  auto func_autodiff_dynamic = make_function(&doKinematicsTemp<VectorXAutoDiffDynamic, VectorXAutoDiffDynamic>);

  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, true, func_double, func_autodiff_fixed_max, func_autodiff_dynamic);
}

void findKinematicPathmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  auto func = make_function(&RigidBodyManipulator::findKinematicPath);
  mexCallFunction(nlhs, plhs, nrhs, prhs, true, func);
}

void forwardJacDotTimesVmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  typedef Map<const Matrix3Xd> DerivedPoints;
  auto func_double = make_function(&RigidBodyManipulator::forwardJacDotTimesV<double, DerivedPoints>);
  auto func_autodiff_fixed_max = make_function(&RigidBodyManipulator::forwardJacDotTimesV<AutoDiffFixedMaxSize, DerivedPoints>);
  auto func_autodiff_dynamic = make_function(&RigidBodyManipulator::forwardJacDotTimesV<AutoDiffDynamicSize, DerivedPoints>);

  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, true, func_double, func_autodiff_fixed_max, func_autodiff_dynamic);
}

void forwardKinmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  auto func_double = make_function(&RigidBodyManipulator::forwardKin<double, Map<const Matrix3Xd>>);
  auto func_autodiff_fixed_max_double_points = make_function(&RigidBodyManipulator::forwardKin<AutoDiffFixedMaxSize, Map<const Matrix3Xd>>);
  auto func_autodiff_fixed_max_autodiff_points = make_function(&RigidBodyManipulator::forwardKin<AutoDiffFixedMaxSize, Matrix<AutoDiffFixedMaxSize, 3, Dynamic>>);
  auto func_autodiff_dynamic_double_points = make_function(&RigidBodyManipulator::forwardKin<AutoDiffDynamicSize, Map<const Matrix3Xd>>);
  auto func_autodiff_dynamic_autodiff_points = make_function(&RigidBodyManipulator::forwardKin<AutoDiffDynamicSize, Matrix<AutoDiffDynamicSize, 3, Dynamic>>);

  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, true, func_double, func_autodiff_fixed_max_double_points, func_autodiff_fixed_max_autodiff_points, func_autodiff_dynamic_double_points, func_autodiff_dynamic_autodiff_points);
}

void forwardKinJacobianmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  typedef Map<const Matrix3Xd> DerivedPoints;
  auto func_double = make_function(&RigidBodyManipulator::forwardKinJacobian<double, DerivedPoints>);
  auto func_autodiff_fixed_max = make_function(&RigidBodyManipulator::forwardKinJacobian<AutoDiffFixedMaxSize, DerivedPoints>);
  auto func_autodiff_dynamic = make_function(&RigidBodyManipulator::forwardKinJacobian<AutoDiffDynamicSize, DerivedPoints>);

  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, true, func_double, func_autodiff_fixed_max, func_autodiff_dynamic);
}

void forwardKinPositionGradientmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  auto func_double = make_function(&RigidBodyManipulator::forwardKinPositionGradient<double>);
  auto func_autodiff_fixed_max = make_function(&RigidBodyManipulator::forwardKinPositionGradient<AutoDiffFixedMaxSize>);
  auto func_autodiff_dynamic = make_function(&RigidBodyManipulator::forwardKinPositionGradient<AutoDiffDynamicSize>);

  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, true, func_double, func_autodiff_fixed_max, func_autodiff_dynamic);
}

void geometricJacobianDotTimesVmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  auto func_double = make_function(&RigidBodyManipulator::geometricJacobianDotTimesV<double>);
  auto func_autodiff_fixed_max = make_function(&RigidBodyManipulator::geometricJacobianDotTimesV<AutoDiffFixedMaxSize>);
  auto func_autodiff_dynamic = make_function(&RigidBodyManipulator::geometricJacobianDotTimesV<AutoDiffDynamicSize>);

  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, true, func_double, func_autodiff_fixed_max, func_autodiff_dynamic);
}

template <typename Scalar>
Matrix<Scalar, TWIST_SIZE, Eigen::Dynamic> geometricJacobianTemp(const RigidBodyManipulator &model, const KinematicsCache<Scalar> &cache, int base_body_or_frame_ind, int end_effector_body_or_frame_ind, int expressed_in_body_or_frame_ind, bool in_terms_of_qdot) {
  // temporary solution. Gross v_or_qdot_indices pointer will be gone soon.
  return model.geometricJacobian(cache, base_body_or_frame_ind, end_effector_body_or_frame_ind, expressed_in_body_or_frame_ind, in_terms_of_qdot, nullptr);
};

void geometricJacobianmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  auto func_double = make_function(&geometricJacobianTemp<double>);
  auto func_autodiff_fixed_max = make_function(&geometricJacobianTemp<AutoDiffFixedMaxSize>);
  auto func_autodiff_dynamic = make_function(&geometricJacobianTemp<AutoDiffDynamicSize>);
  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, true, func_double, func_autodiff_fixed_max, func_autodiff_dynamic);
}

void massMatrixmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  auto func_double = make_function(&RigidBodyManipulator::massMatrix<double>);
  auto func_autodiff_fixed_max = make_function(&RigidBodyManipulator::massMatrix<AutoDiffFixedMaxSize>);
  auto func_autodiff_dynamic = make_function(&RigidBodyManipulator::massMatrix<AutoDiffDynamicSize>);
  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, true, func_double, func_autodiff_fixed_max, func_autodiff_dynamic);
}

template <typename Scalar, typename DerivedF>
Matrix<Scalar, Eigen::Dynamic, 1>  dynamicsBiasTermTemp(const RigidBodyManipulator &model, KinematicsCache<Scalar> &cache, const MatrixBase<DerivedF> &f_ext_value) {
  // temporary solution.

  eigen_aligned_unordered_map<const RigidBody *, Matrix<Scalar, 6, 1> > f_ext;

  if (f_ext_value.size() > 0) {
    assert(f_ext_value.cols() == model.bodies.size());
    for (DenseIndex i = 0; i < f_ext_value.cols(); i++) {
      f_ext.insert({model.bodies[i].get(), f_ext_value.col(i)});
    }
  }

  return model.dynamicsBiasTerm(cache, f_ext);
};

void dynamicsBiasTermmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  auto func_double = make_function(&dynamicsBiasTermTemp<double, Map<const Matrix<double, 6, Dynamic> > >);
  auto func_autodiff_fixed_max = make_function(&dynamicsBiasTermTemp<AutoDiffFixedMaxSize, Matrix<AutoDiffFixedMaxSize, 6, Dynamic> >);
  auto func_autodiff_dynamic = make_function(&dynamicsBiasTermTemp<AutoDiffDynamicSize, Matrix<AutoDiffDynamicSize, 6, Dynamic> >);

  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, true, func_double, func_autodiff_fixed_max, func_autodiff_dynamic);
}
