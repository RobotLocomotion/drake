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

typedef AutoDiffScalar<VectorXd> AutoDiffDynamicSize;
//typedef AutoDiffScalar<Matrix<double, Dynamic, 1, 0, 72, 1> AutoDiffFixedMaxSize;

/**
 * Mex function implementations
 */
void centerOfMassJacobianDotTimesVmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  function<GradientVar<double, SPACE_DIMENSION, 1>(const RigidBodyManipulator &, KinematicsCache<double> &, int, const std::set<int> &)> func_double = mem_fn(
      &RigidBodyManipulator::centerOfMassJacobianDotTimesV<double>);
  function<GradientVar<AutoDiffDynamicSize, SPACE_DIMENSION, 1>(const RigidBodyManipulator &, KinematicsCache<AutoDiffDynamicSize> &, int, const std::set<int> &)> func_autodiff_dynamic = mem_fn(
      &RigidBodyManipulator::centerOfMassJacobianDotTimesV<AutoDiffDynamicSize>);
  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, func_double, func_autodiff_dynamic);
}

void centerOfMassmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  function<Matrix<double, SPACE_DIMENSION, 1>(const RigidBodyManipulator &, KinematicsCache<double> &, const std::set<int> &)> func_double = mem_fn(&RigidBodyManipulator::centerOfMass<double>);
  function<Matrix<AutoDiffDynamicSize, SPACE_DIMENSION, 1>(const RigidBodyManipulator &, KinematicsCache<AutoDiffDynamicSize> &, const std::set<int> &)> func_autodiff_dynamic = mem_fn(&RigidBodyManipulator::centerOfMass<AutoDiffDynamicSize>);
  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, func_double, func_autodiff_dynamic);
}

void centerOfMassJacobianmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  function<GradientVar<double, SPACE_DIMENSION, Dynamic>(const RigidBodyManipulator &, KinematicsCache<double> &, int, const std::set<int> &, bool)> func_double = mem_fn(
      &RigidBodyManipulator::centerOfMassJacobian<double>);
  function<GradientVar<AutoDiffDynamicSize, SPACE_DIMENSION, Dynamic>(const RigidBodyManipulator &, KinematicsCache<AutoDiffDynamicSize> &, int, const std::set<int> &, bool)> func_autodiff_dynamic = mem_fn(
      &RigidBodyManipulator::centerOfMassJacobian<AutoDiffDynamicSize>);
  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, func_double, func_autodiff_dynamic);
}

void centroidalMomentumMatrixDotTimesvmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  function<GradientVar<double, TWIST_SIZE, 1>(const RigidBodyManipulator &, KinematicsCache<double> &, int, const std::set<int> &)> func_double = mem_fn(
      &RigidBodyManipulator::centroidalMomentumMatrixDotTimesV<double>);
  function<GradientVar<AutoDiffDynamicSize, TWIST_SIZE, 1>
      (const RigidBodyManipulator &, KinematicsCache<AutoDiffDynamicSize> &, int, const std::set<int> &)> func_autodiff_dynamic = mem_fn(
      &RigidBodyManipulator::centroidalMomentumMatrixDotTimesV<AutoDiffDynamicSize>);
  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, func_double, func_autodiff_dynamic);
}

void centroidalMomentumMatrixmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  function<GradientVar<double, TWIST_SIZE, Dynamic> const(RigidBodyManipulator &, KinematicsCache<double> &, int, const std::set<int> &, bool)> func_double = mem_fn(
      &RigidBodyManipulator::centroidalMomentumMatrix<double>);
  function<GradientVar<AutoDiffDynamicSize, TWIST_SIZE, Dynamic> const(RigidBodyManipulator &, KinematicsCache<AutoDiffDynamicSize> &, int, const std::set<int> &, bool)> func_autodiff_dynamic = mem_fn(
      &RigidBodyManipulator::centroidalMomentumMatrix<AutoDiffDynamicSize>);
  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, func_double, func_autodiff_dynamic);
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
  typedef Matrix<AutoDiffScalar<VectorXd>, Dynamic, 1> VectorXAutoDiff;
  function<void(const RigidBodyManipulator &, KinematicsCache<double> &, const MatrixBase<Map<const VectorXd>> &, const MatrixBase<Map<const VectorXd>> &, bool)> func_double = &doKinematicsTemp<Map<const VectorXd>, Map<const VectorXd>>;
  function<void(const RigidBodyManipulator &, KinematicsCache<typename VectorXAutoDiff::Scalar> &, const MatrixBase<VectorXAutoDiff> &, const MatrixBase<VectorXAutoDiff> &, bool)> func_autodiff_dynamic = &doKinematicsTemp<VectorXAutoDiff, VectorXAutoDiff>;
  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, func_double, func_autodiff_dynamic);
}

void findKinematicPathmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  function<KinematicPath(const RigidBodyManipulator &, int, int)> func = mem_fn(&RigidBodyManipulator::findKinematicPath);
  mexCallFunction(func, nlhs, plhs, nrhs, prhs);
}

void forwardJacDotTimesVmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  typedef Map<const Matrix3Xd> DerivedPointsDouble;
  function<GradientVar<typename DerivedPointsDouble::Scalar, Dynamic, 1>(const RigidBodyManipulator &, const KinematicsCache<typename DerivedPointsDouble::Scalar> &, const MatrixBase<DerivedPointsDouble> &, int, int, int, int)> func_double = mem_fn(
      &RigidBodyManipulator::forwardJacDotTimesV<DerivedPointsDouble>);

  typedef Matrix<AutoDiffDynamicSize, 3, Dynamic> DerivedPointsAutoDiff;
  function<GradientVar<typename DerivedPointsAutoDiff::Scalar, Dynamic, 1>(const RigidBodyManipulator &, const KinematicsCache<typename DerivedPointsAutoDiff::Scalar> &, const MatrixBase<DerivedPointsAutoDiff> &, int, int, int, int)> func_autodiff_dynamic = mem_fn(
      &RigidBodyManipulator::forwardJacDotTimesV<DerivedPointsAutoDiff>);

  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, func_double, func_autodiff_dynamic);
}

void forwardKinmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  typedef Map<const Matrix3Xd> DerivedPointsDouble;
  function<Matrix<typename DerivedPointsDouble::Scalar, Dynamic, DerivedPointsDouble::ColsAtCompileTime>(RigidBodyManipulator &, const KinematicsCache<typename DerivedPointsDouble::Scalar> &, const MatrixBase<DerivedPointsDouble> &, int, int, int)> func_double = mem_fn(
      &RigidBodyManipulator::forwardKin<DerivedPointsDouble>);

  typedef Matrix<AutoDiffDynamicSize, 3, Dynamic> DerivedPointsAutoDiff;
  function<Matrix<typename DerivedPointsAutoDiff::Scalar, Dynamic, DerivedPointsAutoDiff::ColsAtCompileTime>(RigidBodyManipulator &, const KinematicsCache<typename DerivedPointsAutoDiff::Scalar> &, const MatrixBase<DerivedPointsAutoDiff> &, int, int, int)> func_autodiff_dynamic = mem_fn(
      &RigidBodyManipulator::forwardKin<DerivedPointsAutoDiff>);

  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, func_double, func_autodiff_dynamic);
}

void forwardKinJacobianmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  typedef Map<const Matrix3Xd> DerivedPointsDouble;
  function<GradientVar<typename DerivedPointsDouble::Scalar, Dynamic, Dynamic>(const RigidBodyManipulator &, const KinematicsCache<typename DerivedPointsDouble::Scalar> &, const MatrixBase<DerivedPointsDouble> &, int, int, int, bool, int)> func_double = mem_fn(
      &RigidBodyManipulator::forwardKinJacobian<DerivedPointsDouble>);

  typedef Matrix<AutoDiffDynamicSize, 3, Dynamic> DerivedPointsAutoDiff;
  function<GradientVar<typename DerivedPointsAutoDiff::Scalar, Dynamic, Dynamic>(const RigidBodyManipulator &, const KinematicsCache<typename DerivedPointsAutoDiff::Scalar> &, const MatrixBase<DerivedPointsAutoDiff> &, int, int, int, bool, int)> func_autodiff_dynamic = mem_fn(
      &RigidBodyManipulator::forwardKinJacobian<DerivedPointsAutoDiff>);

  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, func_double, func_autodiff_dynamic);
}

void forwardKinPositionGradientmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  function<GradientVar<double, Dynamic, Dynamic>(const RigidBodyManipulator &, const KinematicsCache<double> &, int, int, int, int)> func_double = mem_fn(
      &RigidBodyManipulator::forwardKinPositionGradient<double>);

  function<GradientVar<AutoDiffDynamicSize, Dynamic, Dynamic>(const RigidBodyManipulator &, const KinematicsCache<AutoDiffDynamicSize> &, int, int, int, int)> func_autodiff_dynamic = mem_fn(
      &RigidBodyManipulator::forwardKinPositionGradient<AutoDiffDynamicSize>);

  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, func_double, func_autodiff_dynamic);
}

void geometricJacobianDotTimesVmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  std::function<GradientVar<double, TWIST_SIZE, 1>(const RigidBodyManipulator &, const KinematicsCache<double> &, int, int, int, int)> func_double = mem_fn(
      &RigidBodyManipulator::geometricJacobianDotTimesV<double>);

  std::function<GradientVar<AutoDiffDynamicSize, TWIST_SIZE, 1>(const RigidBodyManipulator &, const KinematicsCache<AutoDiffDynamicSize> &, int, int, int, int)> func_autodiff_dynamic = mem_fn(
      &RigidBodyManipulator::geometricJacobianDotTimesV<AutoDiffDynamicSize>);

  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, func_double, func_autodiff_dynamic);
}

template <typename Scalar>
GradientVar<Scalar, TWIST_SIZE, Eigen::Dynamic> geometricJacobianTemp(const RigidBodyManipulator &model, const KinematicsCache<Scalar> &cache, int base_body_or_frame_ind, int end_effector_body_or_frame_ind, int expressed_in_body_or_frame_ind, int gradient_order, bool in_terms_of_qdot) {
  // temporary solution. Gross v_or_qdot_indices pointer will be gone soon.
  return model.geometricJacobian(cache, base_body_or_frame_ind, end_effector_body_or_frame_ind, expressed_in_body_or_frame_ind, gradient_order, in_terms_of_qdot, nullptr);
};

void geometricJacobianmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  function<GradientVar<double, TWIST_SIZE, Dynamic>(const RigidBodyManipulator &, const KinematicsCache<double> &, int, int, int, int, bool)> func_double = &geometricJacobianTemp<double>;
  function<GradientVar<AutoDiffDynamicSize, TWIST_SIZE, Dynamic>(const RigidBodyManipulator &, const KinematicsCache<AutoDiffDynamicSize> &, int, int, int, int, bool)> func_autodiff_dynamic = &geometricJacobianTemp<AutoDiffDynamicSize>;
  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, func_double, func_autodiff_dynamic);
}

void massMatrixmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  function<GradientVar<double, Dynamic, Dynamic>(const RigidBodyManipulator &, KinematicsCache<double> &, int)> func_double = mem_fn(&RigidBodyManipulator::massMatrix<double>);
  function<GradientVar<AutoDiffScalar<VectorXd>, Dynamic, Dynamic>(const RigidBodyManipulator &, KinematicsCache<AutoDiffScalar<VectorXd>> &, int)> func_autodiff_dynamic = mem_fn(&RigidBodyManipulator::massMatrix<AutoDiffScalar<VectorXd>>);
  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, func_double, func_autodiff_dynamic);
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
  function<GradientVar<double, Eigen::Dynamic, 1>(const RigidBodyManipulator &, KinematicsCache<double> &, const MatrixBase<Map<const Matrix<double, 6, Dynamic> > > &, const MatrixBase<Map<const Matrix<double, Dynamic, Dynamic> > >&, int)> func_double = &dynamicsBiasTermTemp<double, Map<const Matrix<double, 6, Dynamic> >, Map<const Matrix<double, Dynamic, Dynamic> > >;
//  function<GradientVar<AutoDiffDynamicSize, Eigen::Dynamic, 1>(const RigidBodyManipulator &, KinematicsCache<AutoDiffDynamicSize> &, const MatrixBase<Matrix<AutoDiffDynamicSize, 6, Dynamic>> &, const MatrixBase<Matrix<AutoDiffDynamicSize, Dynamic, Dynamic>> &, int)> func_autodiff_dynamic = &dynamicsBiasTermTemp<AutoDiffDynamicSize, Matrix<AutoDiffDynamicSize, 6, Dynamic>, Matrix<AutoDiffDynamicSize, Dynamic, Dynamic> >;

//  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, func_double, func_autodiff_dynamic);
}
