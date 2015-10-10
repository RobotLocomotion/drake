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
 * Mex function implementations
 */
void centerOfMassJacobianDotTimesVmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  typedef double Scalar;
  function<GradientVar<Scalar, SPACE_DIMENSION, 1>(const RigidBodyManipulator &, KinematicsCache<Scalar> &, int, const std::set<int> &)> func = mem_fn(
      &RigidBodyManipulator::centerOfMassJacobianDotTimesV<Scalar>);
  mexCallFunction(func, nlhs, plhs, nrhs, prhs);
}

void centerOfMassmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  typedef double Scalar;
  function<Matrix<Scalar, SPACE_DIMENSION, 1>(const RigidBodyManipulator &, KinematicsCache<Scalar> &, const std::set<int> &)> func = mem_fn(&RigidBodyManipulator::centerOfMass<Scalar>);
  mexCallFunction(func, nlhs, plhs, nrhs, prhs);
}

void centerOfMassJacobianmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  typedef double Scalar;
  function<GradientVar<Scalar, SPACE_DIMENSION, Dynamic>(const RigidBodyManipulator &, KinematicsCache<Scalar> &, int, const std::set<int> &, bool)> func = mem_fn(
      &RigidBodyManipulator::centerOfMassJacobian<Scalar>);
  mexCallFunction(func, nlhs, plhs, nrhs, prhs);
}

void centroidalMomentumMatrixDotTimesvmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  typedef double Scalar;
  function<GradientVar<Scalar, TWIST_SIZE, 1>(const RigidBodyManipulator &, KinematicsCache<Scalar> &, int, const std::set<int> &)> func = mem_fn(
      &RigidBodyManipulator::centroidalMomentumMatrixDotTimesV<Scalar>);
  mexCallFunction(func, nlhs, plhs, nrhs, prhs);
}

void centroidalMomentumMatrixmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  typedef double Scalar;
  function<GradientVar<Scalar, TWIST_SIZE, Dynamic> const(RigidBodyManipulator &, KinematicsCache<Scalar> &, int, const std::set<int> &, bool)> func = mem_fn(
      &RigidBodyManipulator::centroidalMomentumMatrix<Scalar>);
  mexCallFunction(func, nlhs, plhs, nrhs, prhs);
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
  function<void(const RigidBodyManipulator &, KinematicsCache<double> &, const MatrixBase<Map<const VectorXd>> &, const MatrixBase<Map<const VectorXd>> &, bool)> func_double = &doKinematicsTemp<Map<const VectorXd>, Map<const VectorXd>>;
  typedef Matrix<AutoDiffScalar<VectorXd>, Dynamic, 1> AutoDiffVector;
  function<void(const RigidBodyManipulator &, KinematicsCache<typename AutoDiffVector::Scalar> &, const MatrixBase<AutoDiffVector> &, const MatrixBase<AutoDiffVector> &, bool)> func_autodiff = &doKinematicsTemp<AutoDiffVector, AutoDiffVector>;
  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, func_double, func_autodiff);
}

void findKinematicPathmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  function<KinematicPath(const RigidBodyManipulator &, int, int)> func = mem_fn(&RigidBodyManipulator::findKinematicPath);
  mexCallFunction(func, nlhs, plhs, nrhs, prhs);
}

void forwardJacDotTimesVmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  typedef Map<const Matrix3Xd> DerivedPoints;
  typedef DerivedPoints::Scalar Scalar;
  function<GradientVar<Scalar, Dynamic, 1>(const RigidBodyManipulator &, const KinematicsCache<Scalar> &, const MatrixBase<DerivedPoints> &, int, int, int, int)> func = mem_fn(
      &RigidBodyManipulator::forwardJacDotTimesV<DerivedPoints>);
  mexCallFunction(func, nlhs, plhs, nrhs, prhs);
}

void forwardKinmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  typedef Map<const Matrix3Xd> DerivedPoints;
  typedef DerivedPoints::Scalar Scalar;
  function<Matrix<Scalar, Dynamic, DerivedPoints::ColsAtCompileTime>(RigidBodyManipulator &, const KinematicsCache<Scalar> &, const MatrixBase<DerivedPoints> &, int, int, int)> func = mem_fn(
      &RigidBodyManipulator::forwardKin<DerivedPoints>);
  mexCallFunction(func, nlhs, plhs, nrhs, prhs);
}

void forwardKinJacobianmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  typedef Map<const Matrix3Xd> DerivedPoints;
  typedef DerivedPoints::Scalar Scalar;
  function<GradientVar<Scalar, Dynamic, Dynamic>(const RigidBodyManipulator &, const KinematicsCache<Scalar> &, const MatrixBase<DerivedPoints> &, int, int, int, bool, int)> func = mem_fn(
      &RigidBodyManipulator::forwardKinJacobian<DerivedPoints>);
  mexCallFunction(func, nlhs, plhs, nrhs, prhs);
}

void forwardKinPositionGradientmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  typedef double Scalar;
  function<GradientVar<Scalar, Dynamic, Dynamic>(const RigidBodyManipulator &, const KinematicsCache<Scalar> &, int, int, int, int)> func = mem_fn(
      &RigidBodyManipulator::forwardKinPositionGradient<Scalar>);
  mexCallFunction(func, nlhs, plhs, nrhs, prhs);
}

void geometricJacobianDotTimesVmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  typedef double Scalar;
  std::function<GradientVar<Scalar, TWIST_SIZE, 1>(const RigidBodyManipulator &, const KinematicsCache<Scalar> &, int, int, int, int)> func = mem_fn(
      &RigidBodyManipulator::geometricJacobianDotTimesV<Scalar>);
  mexCallFunction(func, nlhs, plhs, nrhs, prhs);
}

void geometricJacobianmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  // temporary solution. Gross v_or_qdot_indices pointer will be gone soon.
  typedef double Scalar;
  auto lambda = [](const RigidBodyManipulator &model, const KinematicsCache<Scalar> &cache,
                   int base_body_or_frame_ind, int end_effector_body_or_frame_ind, int expressed_in_body_or_frame_ind, int gradient_order, bool in_terms_of_qdot) {
    return model.geometricJacobian(cache, base_body_or_frame_ind, end_effector_body_or_frame_ind, expressed_in_body_or_frame_ind, gradient_order, in_terms_of_qdot, nullptr);
  };
  function<GradientVar<Scalar, TWIST_SIZE, Dynamic>(const RigidBodyManipulator &, const KinematicsCache<Scalar> &, int, int, int, int, bool)> func{lambda};
  mexCallFunction(func, nlhs, plhs, nrhs, prhs);
}

void massMatrixmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  function<GradientVar<double, Dynamic, Dynamic>(const RigidBodyManipulator &, KinematicsCache<double> &, int)> func_double = mem_fn(&RigidBodyManipulator::massMatrix<double>);
  function<GradientVar<AutoDiffScalar<VectorXd>, Dynamic, Dynamic>(const RigidBodyManipulator &, KinematicsCache<AutoDiffScalar<VectorXd>> &, int)> func_autodiff = mem_fn(&RigidBodyManipulator::massMatrix<AutoDiffScalar<VectorXd>>);
  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, func_double, func_autodiff);
}

void dynamicsBiasTermmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  // temporary solution. GradientVar will disappear, obviating the need for the extra argument. integer body indices will be handled differently.
  typedef double Scalar;
  typedef Map<const Matrix<Scalar, 6, Dynamic> > DerivedF;
  typedef Map<const Matrix<Scalar, Dynamic, Dynamic> > DerivedDF;

  auto lambda = [](const RigidBodyManipulator &model, KinematicsCache<Scalar> &cache, const MatrixBase<DerivedF> &f_ext_value, const MatrixBase<DerivedDF> &f_ext_gradient, int gradient_order) {

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
          f_ext_gradientvar.gradient().value() = f_ext_gradient.middleRows<TWIST_SIZE>(TWIST_SIZE * i);
        }

        f_ext.insert({model.bodies[i].get(), f_ext_gradientvar});
      }
    }

    return model.dynamicsBiasTerm(cache, f_ext, gradient_order);
  };

  function<GradientVar<Scalar, Eigen::Dynamic, 1>(const RigidBodyManipulator &, KinematicsCache<Scalar> &, const MatrixBase<DerivedF> &, const MatrixBase<DerivedDF> &, int)> func{lambda};

  mexCallFunction(func, nlhs, plhs, nrhs, prhs);
}
