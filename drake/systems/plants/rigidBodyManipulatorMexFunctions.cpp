#include <typeinfo>
#include <Eigen/Sparse>
#include "rigidBodyManipulatorMexFunctions.h"
#include "RigidBodyManipulator.h"
#include "standardMexConversions.h"

using namespace std;
using namespace Eigen;

/**
 * fromMex specializations
 */
RigidBodyManipulator &fromMex(const mxArray *source, RigidBodyManipulator *) {
  return *static_cast<RigidBodyManipulator *>(getDrakeMexPointer(source));
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
  typedef Map<const Matrix3Xd> DerivedPoints;
  auto func_double = make_function(&RigidBodyManipulator::forwardJacDotTimesV<double, DerivedPoints>);
  auto func_autodiff_fixed_max = make_function(&RigidBodyManipulator::forwardJacDotTimesV<AutoDiffFixedMaxSize, DerivedPoints>);
  auto func_autodiff_dynamic = make_function(&RigidBodyManipulator::forwardJacDotTimesV<AutoDiffDynamicSize, DerivedPoints>);

  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, func_double, func_autodiff_fixed_max, func_autodiff_dynamic);
}

void forwardKinmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  typedef Map<const Matrix3Xd> DerivedPoints;
  auto func_double = make_function(&RigidBodyManipulator::forwardKin<double, DerivedPoints>);
  auto func_autodiff_fixed_max = make_function(&RigidBodyManipulator::forwardKin<AutoDiffFixedMaxSize, DerivedPoints>);
  auto func_autodiff_dynamic = make_function(&RigidBodyManipulator::forwardKin<AutoDiffDynamicSize, DerivedPoints>);

  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, func_double, func_autodiff_fixed_max, func_autodiff_dynamic);
}

void forwardKinJacobianmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  typedef Map<const Matrix3Xd> DerivedPoints;
  auto func_double = make_function(&RigidBodyManipulator::forwardKinJacobian<double, DerivedPoints>);
  auto func_autodiff_fixed_max = make_function(&RigidBodyManipulator::forwardKinJacobian<AutoDiffFixedMaxSize, DerivedPoints>);
  auto func_autodiff_dynamic = make_function(&RigidBodyManipulator::forwardKinJacobian<AutoDiffDynamicSize, DerivedPoints>);

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

template <typename Scalar, typename DerivedF>
GradientVar<Scalar, Eigen::Dynamic, 1>  dynamicsBiasTermTemp(const RigidBodyManipulator &model, KinematicsCache<Scalar> &cache, const MatrixBase<DerivedF> &f_ext_value) {
  // temporary solution. GradientVar will disappear, obviating the need for the extra argument. integer body indices will be handled differently.

  unordered_map<const RigidBody *, GradientVar<Scalar, 6, 1> > f_ext;

  if (f_ext_value.size() > 0) {
    assert(f_ext_value.cols() == model.bodies.size());
    for (DenseIndex i = 0; i < f_ext_value.cols(); i++) {
      GradientVar<Scalar, TWIST_SIZE, 1> f_ext_gradientvar(TWIST_SIZE, 1, model.num_positions + model.num_velocities, 0);
      f_ext_gradientvar.value() = f_ext_value.col(i);
      f_ext.insert({model.bodies[i].get(), f_ext_gradientvar});
    }
  }

  return model.dynamicsBiasTerm(cache, f_ext, 0);
};

void dynamicsBiasTermmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  auto func_double = make_function(&dynamicsBiasTermTemp<double, Map<const Matrix<double, 6, Dynamic> > >);
  auto func_autodiff_fixed_max = make_function(&dynamicsBiasTermTemp<AutoDiffFixedMaxSize, Matrix<AutoDiffFixedMaxSize, 6, Dynamic> >);
  auto func_autodiff_dynamic = make_function(&dynamicsBiasTermTemp<AutoDiffDynamicSize, Matrix<AutoDiffDynamicSize, 6, Dynamic> >);

  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, func_double, func_autodiff_fixed_max, func_autodiff_dynamic);
}

// TODO: move to a better place
inline void buildSparseMatrixForContactConstraints(Map<const Matrix3Xd> const &pts, SparseMatrix<double> &sparse) // TODO rename to something more specific
{
  typedef SparseMatrix<double>::Index SparseIndex;
  const SparseIndex m = static_cast<SparseIndex>(pts.cols());
  const SparseIndex numNonZero = 3 * m;

  sparse.resize(m, numNonZero);
  sparse.reserve(VectorXi::Constant(numNonZero, 1));

  SparseIndex j = 0;
  for (SparseIndex i = 0; i < m; i++) {
    for (SparseIndex k = 0; k < 3; k++) {
      sparse.insert(i, j) = pts(j); // yes, no reference to k
      j++;
    }
  }
}

// TODO: move to a better place
template<typename Scalar>
using MatrixX = Matrix<Scalar, Dynamic, Dynamic>;

// TODO: move to a better place
template<typename Scalar>
pair<MatrixX<Scalar>, vector<MatrixX<Scalar> > > contactConstraintsTemp(const RigidBodyManipulator &model, const KinematicsCache<Scalar> &cache, const Map<const Matrix3Xd> &normals, const Map<const VectorXi>& idxA,
                                                                    const Map<const VectorXi>& idxB, const Map<const Matrix3Xd> &xA, const Map<const Matrix3Xd> &xB, const vector<Map<const Matrix3Xd>> &d) {

  Matrix<Scalar, Dynamic, Dynamic> J;
  Matrix<Scalar, Dynamic, Dynamic> dJ;
  model.computeContactJacobians(cache, idxA, idxB, xA, xB, false, J, dJ);

  SparseMatrix<double> sparseNormals;
  buildSparseMatrixForContactConstraints(normals, sparseNormals);
  auto n = (sparseNormals.cast<Scalar>() * J).eval(); //dphi/dq

  size_t num_tangent_vectors = d.size();
  vector<MatrixX<Scalar> > D(2 * num_tangent_vectors, MatrixX<Scalar>(d.at(0).rows(), J.cols()));
  for (int k = 0; k < num_tangent_vectors; k++) { //for each friction cone basis vector
    const auto &dk = d.at(k);
    SparseMatrix<double> sparseTangents;
    buildSparseMatrixForContactConstraints(dk, sparseTangents);
    auto sparseTangents_cast = (sparseTangents.cast<Scalar>()).eval();
    D.at(k) = (sparseTangents_cast * J).eval(); //dd/dq;
    D.at(k + num_tangent_vectors) = -D.at(k);
  }

  return make_pair(n, D);
}

void contactConstraintsmex(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  auto func_double = make_function(&contactConstraintsTemp<double>);
  auto func_autodiff_fixed_max = make_function(&contactConstraintsTemp<AutoDiffFixedMaxSize>);
  auto func_autodiff_dynamic = make_function(&contactConstraintsTemp<AutoDiffDynamicSize>);
  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, func_double, func_autodiff_fixed_max, func_autodiff_dynamic);
}
