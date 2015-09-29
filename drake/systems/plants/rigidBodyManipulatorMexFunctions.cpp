#include "rigidBodyManipulatorMexFunctions.h"
#include "RigidBodyManipulator.h"

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

/*
 * unfortunately, due to an MSVC internal compiler error, can't have
 * template <int Rows, int Cols>
 * Map<const Matrix<double, Rows, Cols>> fromMex(const mxArray* mex, MatrixBase<Map<const Matrix<double, Rows, Cols>>>*)
 */

Map<const Matrix<double, Dynamic, Dynamic>> fromMex(const mxArray *mex, MatrixBase<Map<const Matrix<double, Dynamic, Dynamic>>> *) {
  return matlabToEigenMap<Dynamic, Dynamic>(mex);
}

Map<const Matrix<double, 3, Dynamic>> fromMex(const mxArray *mex, MatrixBase<Map<const Matrix<double, 3, Dynamic>>> *) {
  return matlabToEigenMap<3, Dynamic>(mex);
}

Map<const Matrix<double, 6, Dynamic>> fromMex(const mxArray *mex, MatrixBase<Map<const Matrix<double, 6, Dynamic>>> *) {
  return matlabToEigenMap<6, Dynamic>(mex);
}

Map<const Matrix<double, Dynamic, 1>> fromMex(const mxArray *mex, MatrixBase<Map<const Matrix<double, Dynamic, 1>>> *) {
  return matlabToEigenMap<Dynamic, 1>(mex);
}

template<typename Scalar>
KinematicsCache<Scalar> &fromMex(const mxArray *mex, KinematicsCache<Scalar> *) {
  return *static_cast<KinematicsCache<Scalar> *>(getDrakeMexPointer(mex));
}

/**
 * toMex specializations
 */
template<typename Scalar, int Rows, int Cols>
void toMex(const GradientVar<Scalar, Rows, Cols> &source, mxArray *dest[], int nlhs, bool top_level = true) {
  if (top_level) {
    // check number of output arguments
    if (nlhs > source.maxOrder() + 1) {
      std::ostringstream buf;
      buf << nlhs << " output arguments desired, which is more than the maximum number: " << source.maxOrder() + 1 << ".";
      mexErrMsgTxt(buf.str().c_str());
    }
  }

  if (nlhs != 0) {
    // set an output argument
    dest[0] = eigenToMatlab(source.value());

    // recurse
    if (source.hasGradient()) {
      toMex(source.gradient(), &dest[1], nlhs - 1, false);
    }
  }
};

/*
 * unfortunately, due to an MSVC internal compiler error, can't have
 * template <int Rows, int Cols>
 * void toMex(const Matrix<double, Rows, Cols> &source, mxArray *dest[], int nlhs)
 */

void toMex(const Matrix<double, 3, 1> &source, mxArray *dest[], int nlhs) {
  if (nlhs > 0)
    dest[0] = eigenToMatlab(source);
};

void toMex(const Matrix<double, Dynamic, Dynamic> &source, mxArray *dest[], int nlhs) {
  if (nlhs > 0)
    dest[0] = eigenToMatlab(source);
};

template<>
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

void doKinematicsmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  // temporary solution. Explicit doKinematics calls will not be necessary in the near future.
  typedef Map<const VectorXd> DerivedQ;
  typedef Map<const VectorXd> DerivedV;
  typedef DerivedQ::Scalar Scalar;
  auto lambda = [](const RigidBodyManipulator &model, KinematicsCache<Scalar> &cache, const MatrixBase<DerivedQ> &q, const MatrixBase<DerivedV> &v, bool compute_JdotV) {
    if (v.size() == 0 && model.num_velocities != 0)
      cache.initialize(q);
    else
      cache.initialize(q, v);
    model.doKinematics(cache, compute_JdotV);
  };
  function<void(const RigidBodyManipulator &, KinematicsCache<Scalar> &, const MatrixBase<DerivedQ> &, const MatrixBase<DerivedV> &, bool)> func{lambda};
  mexCallFunction(func, nlhs, plhs, nrhs, prhs);
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
  typedef double Scalar;
  function<GradientVar<Scalar, Dynamic, Dynamic>(const RigidBodyManipulator &, KinematicsCache<Scalar> &, int)> func = mem_fn(&RigidBodyManipulator::massMatrix<Scalar>);
  mexCallFunction(func, nlhs, plhs, nrhs, prhs);
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
