#include "rigidBodyManipulatorMexFunctions.h"
#include "RigidBodyManipulator.h"
#include "standardMexConversions.h"
#include <typeinfo>

using namespace std;
using namespace Eigen;

/**
 * fromMex specializations
 */
RigidBodyManipulator &fromMex(const mxArray *source, RigidBodyManipulator *) {
  return *static_cast<RigidBodyManipulator *>(getDrakeMexPointer(source));
}

std::set<int> fromMex(const mxArray *source, std::set<int> *) {
  // for robotnum. this is kind of a weird one, but OK for now
  std::set<int> robotnum_set;
  int num_robot = static_cast<int>(mxGetNumberOfElements(source));
  double *robotnum = mxGetPrSafe(source);
  for (int i = 0; i < num_robot; i++) {
    robotnum_set.insert((int) robotnum[i] - 1);
  }
  return robotnum_set;
}

template<typename Scalar>
KinematicsCache<Scalar> &fromMex(const mxArray *mex, KinematicsCache<Scalar> *) {
  if (!mxIsClass(mex, "DrakeMexPointer")) {
    throw MexToCppConversionError("Expected DrakeMexPointer containing KinematicsCache");
  }
  auto name = mxGetStdString(mxGetPropertySafe(mex, "name"));
  if (name != typeid(KinematicsCache<Scalar>).name()) {
    ostringstream buf;
    buf << "Expected KinematicsCache of type " << typeid(KinematicsCache<Scalar>).name() << ", but got " << name;
    throw MexToCppConversionError(buf.str());
  }
  return *static_cast<KinematicsCache<Scalar> *>(getDrakeMexPointer(mex));
}

/**
 * toMex specializations
 */
void toMex(const KinematicPath &path, mxArray *dest[], int nlhs) {
  if (nlhs > 0) {
    dest[0] = stdVectorToMatlab(path.body_path);
  }
  if (nlhs > 1) {
    dest[1] = stdVectorToMatlab(path.joint_path);
  }
  if (nlhs > 2) {
    dest[2] = stdVectorToMatlab(path.joint_direction_signs);
  }
}

/**
 * make_function
 * Note that a completely general make_function implementation is not possible due to ambiguities, but this works for all of the cases in this file
 * Inspired by from http://stackoverflow.com/a/21740143/2228557
 */

//plain function pointers
template<typename... Args, typename ReturnType>
auto make_function(ReturnType(*p)(Args...))
-> std::function<ReturnType(Args...)> { return {p}; }

//nonconst member function pointers
template<typename... Args, typename ReturnType, typename ClassType>
auto make_function(ReturnType(ClassType::*p)(Args...))
-> std::function<ReturnType(ClassType&, Args...)> { return {p}; }

//const member function pointers
template<typename... Args, typename ReturnType, typename ClassType>
auto make_function(ReturnType(ClassType::*p)(Args...) const)
-> std::function<ReturnType(const ClassType&, Args...)> { return {p}; }

typedef AutoDiffScalar<VectorXd> AutoDiffDynamicSize;
typedef DrakeJoint::AutoDiffFixedMaxSize AutoDiffFixedMaxSize;

/**
 * Mex function implementations
 */
void centerOfMassJacobianDotTimesVmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  auto func_double = make_function(&RigidBodyManipulator::centerOfMassJacobianDotTimesV<double>);
  auto func_autodiff_fixed_max = make_function(&RigidBodyManipulator::centerOfMassJacobianDotTimesV<AutoDiffFixedMaxSize>);
  auto func_autodiff_dynamic = make_function(&RigidBodyManipulator::centerOfMassJacobianDotTimesV<AutoDiffDynamicSize>);
  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, func_double, func_autodiff_fixed_max, func_autodiff_dynamic);
}

void centerOfMassmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  auto func_double = make_function(&RigidBodyManipulator::centerOfMass<double>);
  auto func_autodiff_fixed_max = make_function(&RigidBodyManipulator::centerOfMass<AutoDiffFixedMaxSize>);
  auto func_autodiff_dynamic = make_function(&RigidBodyManipulator::centerOfMass<AutoDiffDynamicSize>);
  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, func_double, func_autodiff_fixed_max, func_autodiff_dynamic);
}

void centerOfMassJacobianmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  auto func_double = make_function(&RigidBodyManipulator::centerOfMassJacobian<double>);
  auto func_autodiff_fixed_max = make_function(&RigidBodyManipulator::centerOfMassJacobian<AutoDiffFixedMaxSize>);
  auto func_autodiff_dynamic = make_function(&RigidBodyManipulator::centerOfMassJacobian<AutoDiffDynamicSize>);
  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, func_double, func_autodiff_fixed_max, func_autodiff_dynamic);
}

void centroidalMomentumMatrixDotTimesvmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  auto func_double = make_function(&RigidBodyManipulator::centroidalMomentumMatrixDotTimesV<double>);
  auto func_autodiff_fixed_max = make_function(&RigidBodyManipulator::centroidalMomentumMatrixDotTimesV<AutoDiffFixedMaxSize>);
  auto func_autodiff_dynamic = make_function(&RigidBodyManipulator::centroidalMomentumMatrixDotTimesV<AutoDiffDynamicSize>);
  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, func_double, func_autodiff_fixed_max, func_autodiff_dynamic);
}

void centroidalMomentumMatrixmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  auto func_double = make_function(&RigidBodyManipulator::centroidalMomentumMatrix<double>);
  auto func_autodiff_fixed_max = make_function(&RigidBodyManipulator::centroidalMomentumMatrix<AutoDiffFixedMaxSize>);
  auto func_autodiff_dynamic = make_function(&RigidBodyManipulator::centroidalMomentumMatrix<AutoDiffDynamicSize>);
  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, func_double, func_autodiff_fixed_max, func_autodiff_dynamic);
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

  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, func_double, func_autodiff_fixed_max, func_autodiff_dynamic);
}

void findKinematicPathmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  auto func = make_function(&RigidBodyManipulator::findKinematicPath);
  mexCallFunction(func, nlhs, plhs, nrhs, prhs);
}

void forwardJacDotTimesVmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  typedef Map<const Matrix3Xd> DerivedPointsDouble;
  auto func_double = make_function(&RigidBodyManipulator::forwardJacDotTimesV<DerivedPointsDouble>);

  typedef Matrix<AutoDiffFixedMaxSize, 3, Dynamic> DerivedPointsAutoDiffFixedMax;
  auto func_autodiff_fixed_max = make_function(&RigidBodyManipulator::forwardJacDotTimesV<DerivedPointsAutoDiffFixedMax>);

  typedef Matrix<AutoDiffDynamicSize, 3, Dynamic> DerivedPointsAutoDiffDynamic;
  auto func_autodiff_dynamic = make_function(&RigidBodyManipulator::forwardJacDotTimesV<DerivedPointsAutoDiffDynamic>);

  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, func_double, func_autodiff_fixed_max, func_autodiff_dynamic);
}

void forwardKinmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  typedef Map<const Matrix3Xd> DerivedPointsDouble;
  auto func_double = make_function(&RigidBodyManipulator::forwardKin<DerivedPointsDouble>);

  typedef Matrix<AutoDiffFixedMaxSize, 3, Dynamic> DerivedPointsAutoDiffFixedMax;
  auto func_autodiff_fixed_max = make_function(&RigidBodyManipulator::forwardKin<DerivedPointsAutoDiffFixedMax>);

  typedef Matrix<AutoDiffDynamicSize, 3, Dynamic> DerivedPointsAutoDiffDynamic;
  auto func_autodiff_dynamic = make_function(&RigidBodyManipulator::forwardKin<DerivedPointsAutoDiffDynamic>);

  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, func_double, func_autodiff_fixed_max, func_autodiff_dynamic);
}

void forwardKinJacobianmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  typedef Map<const Matrix3Xd> DerivedPointsDouble;
  auto func_double = make_function(&RigidBodyManipulator::forwardKinJacobian<DerivedPointsDouble>);

  typedef Matrix<AutoDiffFixedMaxSize, 3, Dynamic> DerivedPointsAutoDiffFixedMax;
  auto func_autodiff_fixed_max = make_function(&RigidBodyManipulator::forwardKinJacobian<DerivedPointsAutoDiffFixedMax>);

  typedef Matrix<AutoDiffDynamicSize, 3, Dynamic> DerivedPointsAutoDiffDynamic;
  auto func_autodiff_dynamic = make_function(&RigidBodyManipulator::forwardKinJacobian<DerivedPointsAutoDiffDynamic>);

  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, func_double, func_autodiff_fixed_max, func_autodiff_dynamic);
}

void forwardKinPositionGradientmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  auto func_double = make_function(&RigidBodyManipulator::forwardKinPositionGradient<double>);
  auto func_autodiff_fixed_max = make_function(&RigidBodyManipulator::forwardKinPositionGradient<AutoDiffFixedMaxSize>);
  auto func_autodiff_dynamic = make_function(&RigidBodyManipulator::forwardKinPositionGradient<AutoDiffDynamicSize>);

  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, func_double, func_autodiff_fixed_max, func_autodiff_dynamic);
}

void geometricJacobianDotTimesVmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  auto func_double = make_function(&RigidBodyManipulator::geometricJacobianDotTimesV<double>);
  auto func_autodiff_fixed_max = make_function(&RigidBodyManipulator::geometricJacobianDotTimesV<AutoDiffFixedMaxSize>);
  auto func_autodiff_dynamic = make_function(&RigidBodyManipulator::geometricJacobianDotTimesV<AutoDiffDynamicSize>);

  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, func_double, func_autodiff_fixed_max, func_autodiff_dynamic);
}

template <typename Scalar>
GradientVar<Scalar, TWIST_SIZE, Eigen::Dynamic> geometricJacobianTemp(const RigidBodyManipulator &model, const KinematicsCache<Scalar> &cache, int base_body_or_frame_ind, int end_effector_body_or_frame_ind, int expressed_in_body_or_frame_ind, int gradient_order, bool in_terms_of_qdot) {
  // temporary solution. Gross v_or_qdot_indices pointer will be gone soon.
  return model.geometricJacobian(cache, base_body_or_frame_ind, end_effector_body_or_frame_ind, expressed_in_body_or_frame_ind, gradient_order, in_terms_of_qdot, nullptr);
};

void geometricJacobianmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  auto func_double = make_function(&geometricJacobianTemp<double>);
  auto func_autodiff_fixed_max = make_function(&geometricJacobianTemp<AutoDiffFixedMaxSize>);
  auto func_autodiff_dynamic = make_function(&geometricJacobianTemp<AutoDiffDynamicSize>);
  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, func_double, func_autodiff_fixed_max, func_autodiff_dynamic);
}

void massMatrixmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  auto func_double = make_function(&RigidBodyManipulator::massMatrix<double>);
  auto func_autodiff_fixed_max = make_function(&RigidBodyManipulator::massMatrix<AutoDiffFixedMaxSize>);
  auto func_autodiff_dynamic = make_function(&RigidBodyManipulator::massMatrix<AutoDiffDynamicSize>);
  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, func_double, func_autodiff_fixed_max, func_autodiff_dynamic);
}

template <typename Scalar, typename DerivedF, typename DerivedDF>
GradientVar<Scalar, Eigen::Dynamic, 1>  dynamicsBiasTermTemp(const RigidBodyManipulator &model, KinematicsCache<Scalar> &cache, const MatrixBase<DerivedF> &f_ext_value, const MatrixBase<DerivedDF> &f_ext_gradient, int gradient_order) {
  // temporary solution. GradientVar will disappear, obviating the need for the extra argument. integer body indices will be handled differently.

  unordered_map<const RigidBody *, GradientVar<Scalar, 6, 1> > f_ext;

  if (f_ext_value.size() > 0) {
    assert(f_ext_value.cols() == model.bodies.size());
    if (gradient_order > 0) {
      assert(f_ext_gradient.rows() == f_ext_value.size());
      assert(f_ext_gradient.cols() == model.num_positions + model.num_velocities);
    }

    for (DenseIndex i = 0; i < f_ext_value.cols(); i++) {
      GradientVar<Scalar, TWIST_SIZE, 1> f_ext_gradientvar(TWIST_SIZE, 1, model.num_positions + model.num_velocities, gradient_order);
      f_ext_gradientvar.value() = f_ext_value.col(i);

      if (gradient_order > 0) {
        f_ext_gradientvar.gradient().value() = f_ext_gradient.template middleRows<TWIST_SIZE>(TWIST_SIZE * i);
      }

      f_ext.insert({model.bodies[i].get(), f_ext_gradientvar});
    }
  }

  return model.dynamicsBiasTerm(cache, f_ext, gradient_order);
};

void dynamicsBiasTermmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  auto func_double = make_function(&dynamicsBiasTermTemp<double, Map<const Matrix<double, 6, Dynamic> >, Map<const Matrix<double, Dynamic, Dynamic> > >);
  auto func_autodiff_fixed_max = make_function(&dynamicsBiasTermTemp<AutoDiffFixedMaxSize, Matrix<AutoDiffFixedMaxSize, 6, Dynamic>, Matrix<AutoDiffFixedMaxSize, Dynamic, Dynamic> >);
  auto func_autodiff_dynamic = make_function(&dynamicsBiasTermTemp<AutoDiffDynamicSize, Matrix<AutoDiffDynamicSize, 6, Dynamic>, Matrix<AutoDiffDynamicSize, Dynamic, Dynamic> >);

  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, func_double, func_autodiff_fixed_max, func_autodiff_dynamic);
}
