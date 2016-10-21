#include <Eigen/Sparse>

#include "drake/common/eigen_types.h"
#include "drake/matlab/util/mexify.h"
#include "drake/matlab/util/standardMexConversions.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/systems/plants/KinematicsCache.h"

#include "rigidBodyTreeMexConversions.h"

using namespace std;
using namespace Eigen;
using namespace drake;

typedef AutoDiffScalar<VectorXd> AutoDiffDynamicSize;
typedef DrakeJoint::AutoDiffFixedMaxSize AutoDiffFixedMaxSize;

template <typename... Args, typename ReturnType>
auto make_function(ReturnType (*p)(Args...))
    -> std::function<ReturnType(Args...)> {
  return {p};
}

inline void buildSparseMatrixForContactConstraints(
    Map<const Matrix3Xd> const &pts, SparseMatrix<double> &sparse) {
  const Index m = pts.cols();
  const Index numNonZero = 3 * m;

  sparse.resize(m, numNonZero);
  sparse.reserve(VectorXi::Constant(numNonZero, 1));

  Index j = 0;
  for (Index i = 0; i < m; i++) {
    for (Index k = 0; k < 3; k++) {
      sparse.insert(i, j) = pts(j);  // yes, no reference to k
      j++;
    }
  }
}

template <typename Scalar>
pair<MatrixX<Scalar>, vector<MatrixX<Scalar>>> contactConstraints(
    const RigidBodyTree &model, const KinematicsCache<Scalar> &cache,
    const Map<const Matrix3Xd> &normals, const Map<const VectorXi> &idxA,
    const Map<const VectorXi> &idxB, const Map<const Matrix3Xd> &xA,
    const Map<const Matrix3Xd> &xB, const vector<Map<const Matrix3Xd>> &d) {
  Matrix<Scalar, Dynamic, Dynamic> J;
  model.computeContactJacobians(cache, idxA, idxB, xA, xB, J);

  SparseMatrix<double> sparseNormals;
  buildSparseMatrixForContactConstraints(normals, sparseNormals);
  auto n = (sparseNormals.cast<Scalar>() * J).eval();  // dphi/dq

  size_t num_tangent_vectors = d.size();
  vector<MatrixX<Scalar>> D(2 * num_tangent_vectors,
                            MatrixX<Scalar>(d.at(0).rows(), J.cols()));
  for (int k = 0; k < num_tangent_vectors;
       k++) {  // for each friction cone basis vector
    const auto &dk = d.at(k);
    SparseMatrix<double> sparseTangents;
    buildSparseMatrixForContactConstraints(dk, sparseTangents);
    auto sparseTangents_cast = (sparseTangents.cast<Scalar>()).eval();
    D.at(k) = (sparseTangents_cast * J).eval();  // dd/dq;
    D.at(k + num_tangent_vectors) = -D.at(k);
  }

  return make_pair(n, D);
}

DLL_EXPORT_SYM
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  auto func_double = make_function(&contactConstraints<double>);
  auto func_autodiff_fixed_max =
      make_function(&contactConstraints<AutoDiffFixedMaxSize>);
  auto func_autodiff_dynamic =
      make_function(&contactConstraints<AutoDiffDynamicSize>);
  mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, true, func_double,
                        func_autodiff_fixed_max, func_autodiff_dynamic);
}
