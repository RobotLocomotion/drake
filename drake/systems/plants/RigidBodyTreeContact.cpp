#include "drake/systems/plants/RigidBodyTree.h"

#include <algorithm>
#include <iostream>
#include <set>
#include <vector>

using Eigen::Dynamic;
using Eigen::Map;
using Eigen::Matrix3Xd;
using Eigen::Matrix;
using Eigen::Ref;
using Eigen::Vector3d;
using Eigen::VectorXi;
using std::vector;

// Computes surface tangent vectors for a single normal vector
// INPUTS:
//   normal: (3 x 1) normal vector in world coordinates
// OUTPUTS:
//   d: (3 x k) matrix where the k columns contain the surface tangents
// NOTE:
//  k = BASIS_VECTOR_HALF_COUNT is defined as a preprocessor directive so that
//      Eigen templates can be optimized at compile time
// TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
void surfaceTangentsSingle(Vector3d const &normal, Matrix3kd &d) {
  Vector3d t1, t2;
  double theta;

  if (1.0 - normal(2) < EPSILON) {  // handle the unit-normal case (since it's
                                    // unit length, just check z)
    t1 << 1.0, 0.0, 0.0;
  } else if (1 + normal(2) < EPSILON) {
    t1 << -1.0, 0.0, 0.0;  // same for the reflected case
  } else {                 // now the general case
    t1 << normal(1), -normal(0), 0.0;
    t1 /= sqrt(normal(1) * normal(1) + normal(0) * normal(0));
  }

  t2 = t1.cross(normal);

  for (size_t k = 0; k < BASIS_VECTOR_HALF_COUNT; k++) {
    theta = k * M_PI / BASIS_VECTOR_HALF_COUNT;
    d.col(k) = cos(theta) * t1 + sin(theta) * t2;
  }
}

// Consolidates unique body indexes from idXA and idxB into a sorted set
// INPUTS:
//   idxA mx1 vector of body indexes for body A in m possible contacts
//   idxB mx1 vector of bodyh indexes for body B in m possible contacts
// OUTPUTS:
//   bodyIndsSorted a set of unique, sorted(ascending) body indexes
//   participating in contact pairs
void getUniqueBodiesSorted(
    VectorXi const &idxA, VectorXi const &idxB,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    std::vector<int> &bodyIndsSorted) {
  size_t m = idxA.size();
  std::set<int> bodyInds;

  for (size_t i = 0; i < m; i++) {
    bodyInds.insert(idxA[i]);
    bodyInds.insert(idxB[i]);
  }

  bodyIndsSorted.clear();

  for (std::set<int>::const_iterator citer = bodyInds.begin();
       citer != bodyInds.end(); citer++) {
    if (*citer > 0) {
      bodyIndsSorted.push_back(*citer);
    }
  }

  sort(bodyIndsSorted.begin(), bodyIndsSorted.end());
}

// Finds the n occurences of the body index in the original list of m contact
// indexes
// INPUTS:
//   idxList: (m x 1) Indexes of m bodies for possible contact pairs
//   bodyIdx: the body index to search for
// OUTPUTS:
//   contactIdx: the list of n indexes into idxList where bodyIdx occurred
void findContactIndexes(VectorXi const &idxList, const size_t bodyIdx,
                        // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
                        std::vector<size_t> &contactIdx) {
  size_t m = idxList.size();
  contactIdx.clear();
  for (size_t i = 0; i < m; i++) {
    if (idxList[i] == static_cast<int>(bodyIdx)) {
      contactIdx.push_back(i);  // zero-based index
    }
  }
}

// Gets a concatenated list of contact points corresponding to a single body
// regardless of whether it is body A or body B
//  This allows all the points to be transformed into world space using a single
//  forwardKin call.
// INPUTS:
//   cindA: list of indexes into the original list of m contact pairs where the
//   body appeared as body A
//   cindB: list of iindexes into the original list of m contact pairs where the
//   body appeared as body B
//   xA: (3 x m) matrix where each column represents a contact point on body A
//   xB: (3 x m) matrix where each column represents a contact point on body B
// OUTPUTS:
//   bodyPoints: (4 x n) matrix of contact points containing occurrences of body
//   A contact points followed
//     by body B contact points where n = size(cindA) + size(cindB)
// NOTE: the output is a matrix of 4-vector columns in homogeneous coordinates
// (x, y, z, 1)'
void getBodyPoints(std::vector<size_t> const &cindA,
                   std::vector<size_t> const &cindB, Matrix3Xd const &xA,
                   Matrix3Xd const &xB,
                   // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
                   Matrix3Xd &bodyPoints) {
  size_t i = 0;
  size_t numPtsA = cindA.size();
  size_t numPtsB = cindB.size();

  bodyPoints.resize(3, numPtsA + numPtsB);

  for (i = 0; i < numPtsA; i++) {
    bodyPoints.col(i) = xA.col(cindA[i]);
  }

  for (i = 0; i < numPtsB; i++) {
    bodyPoints.col(numPtsA + i) = xB.col(cindB[i]);
  }
}

// Builds the part of the contact Jacobian matrix corresponding to bodyInd
//  by accumulating positive contributions  from points on body A and negative
//  contributions from the points on body B.
//  This is because the normal vectors(n = dphi/dq) are defined to be pointing
//  from B to A
// INPUTS
//   bodyInd: the index of the body currently being processed
//   bodyPoints: (3 x n) matrix where each column is a point on the body
//   cindA: indexes into the original set of m contact pairs where the body
//   appears as Body A
//   cindB: indexes into the original set of m contact pairs where the body
//   appears as Body B
// NOTE
//   cindA and cindB are gotten by calling findContactIndexes in
//   drakeContactConstraintsUtil
//   cols(bodyPoints) = size(cindA) + size(cindB)
// OUTPUTS:
//   J: (3m x nq) The partial contact Jacobian matrix
// NOTE
//  After one call to the function, the n rows of the Jacobian matrix
//  corresponding to bodyInd will be completed
//  This function must be called with all bodyInds to finish the total
//  accumulation of the contact Jacobian
template <typename Scalar>
void RigidBodyTree::accumulateContactJacobian(
    const KinematicsCache<Scalar> &cache, const int bodyInd,
    Matrix3Xd const &bodyPoints, std::vector<size_t> const &cindA,
    std::vector<size_t> const &cindB,
    Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> &J) const {
  const auto nq = J.cols();
  const size_t numCA = cindA.size();
  const size_t numCB = cindB.size();
  const size_t offset = 3 * numCA;

  auto J_tmp = transformPointsJacobian(cache, bodyPoints, bodyInd, 0, true);

  // add contributions from points in xA
  for (size_t x = 0; x < numCA; x++) {
    J.block(3 * cindA[x], 0, 3, nq) += J_tmp.block(3 * x, 0, 3, nq);
  }

  // subtract contributions from points in xB
  for (size_t x = 0; x < numCB; x++) {
    J.block(3 * cindB[x], 0, 3, nq) -= J_tmp.block(offset + 3 * x, 0, 3, nq);
  }
}

// Computes the full contact Jacobian and, optionally, the second order contact
// Jacobian.
//  This can be used to compute the contact normals in joint coordinates (n =
//  dphi/dq), the surface tangents
//  in joint coordinates (D), and their respective second derivatives with
//  respect to q (dn, dD)
// INPUTS
//   idxA: (m x 1) an integer list of body indexes of body A for m possible
//   contact pairs
//   idxB: (m x 1) an integer list of body indexes of body B for m possible
//   contact pairs
//   xA: (3 x m) each column of the matrix is a contact point in the body A
//   frame for that contact pair
//   xB: (3 x m) each column of the matrix is a contact point in the body B
//   frame for that contact pair
//   compute_second_derivatives: boolean flag to indicate that the second order
//   contact Jacobian should also be computed
// OUTPUTS
//   J: (3m x nq)
//  (optional outputs if compute_second_derivatives is true)
//  dJ: (3m x nq^2) Second order contact Jacobian
// TODO(tkoolen): change output to be 3m * nq x nq (or possibly 3m * nv x nq)

template <typename Scalar>
void RigidBodyTree::computeContactJacobians(
    const KinematicsCache<Scalar> &cache, Ref<const VectorXi> const &idxA,
    Ref<const VectorXi> const &idxB, Ref<const Matrix3Xd> const &xA,
    Ref<const Matrix3Xd> const &xB, Matrix<Scalar, Dynamic, Dynamic> &J) const {
  std::vector<int> bodyInds;
  const size_t nq = num_positions_;
  const size_t numContactPairs = xA.cols();

  typedef Matrix<Scalar, Dynamic, Dynamic> MatrixX;

  J = MatrixX::Zero(3 * numContactPairs, nq);

  getUniqueBodiesSorted(idxA, idxB, bodyInds);

  const size_t numUniqueBodies = bodyInds.size();

  for (size_t i = 0; i < numUniqueBodies; i++) {
    const int bodyInd = bodyInds[i];
    vector<size_t> cindA, cindB;
    Matrix3Xd bodyPoints;
    findContactIndexes(idxA, bodyInd, cindA);
    findContactIndexes(idxB, bodyInd, cindB);
    getBodyPoints(cindA, cindB, xA, xB, bodyPoints);
    accumulateContactJacobian(cache, bodyInd, bodyPoints, cindA, cindB, J);
  }
}

// Computes surface tangent vectors for many normal vectors at once
// INPUTS:
//   normals: (3 x m) matrix where each column is a normal vector in world
//   coordinates
// OUTPUTS:
//   tangents: list of k (3 x m) matrices where each column contains one of the
//   k tangent vectors
//              for the corresponding normal.
// NOTE:
//  k = BASIS_VECTOR_HALF_COUNT is defined as a preprocessor directive so that
//      Eigen templates can be optimized at compile time
void RigidBodyTree::surfaceTangents(
    Map<Matrix3Xd> const &normals,
    std::vector<Map<Matrix3Xd> > &tangents) const {
  const size_t numContactPairs = normals.cols();
  for (size_t curNormal = 0; curNormal < numContactPairs; curNormal++) {
    Matrix3kd d;
    surfaceTangentsSingle(normals.col(curNormal), d);
    for (size_t k = 0; k < BASIS_VECTOR_HALF_COUNT; k++) {
      tangents[k].col(curNormal) = d.col(k);
    }
  }
}

template DRAKE_EXPORT void RigidBodyTree::computeContactJacobians<
    Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, 73, 1> > >(
    KinematicsCache<Eigen::AutoDiffScalar<
        Eigen::Matrix<double, -1, 1, 0, 73, 1> > > const &,
    Eigen::Ref<Eigen::Matrix<int, -1, 1, 0, -1, 1> const, 0,
               Eigen::InnerStride<1> > const &,
    Eigen::Ref<Eigen::Matrix<int, -1, 1, 0, -1, 1> const, 0,
               Eigen::InnerStride<1> > const &,
    Eigen::Ref<Eigen::Matrix<double, 3, -1, 0, 3, -1> const, 0,
               Eigen::OuterStride<-1> > const &,
    Eigen::Ref<Eigen::Matrix<double, 3, -1, 0, 3, -1> const, 0,
               Eigen::OuterStride<-1> > const &,
    Eigen::Matrix<
        Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, 73, 1> >, -1, -1,
        0, -1, -1> &) const;
template DRAKE_EXPORT void RigidBodyTree::computeContactJacobians<
    Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, -1, 1> > >(
    KinematicsCache<Eigen::AutoDiffScalar<
        Eigen::Matrix<double, -1, 1, 0, -1, 1> > > const &,
    Eigen::Ref<Eigen::Matrix<int, -1, 1, 0, -1, 1> const, 0,
               Eigen::InnerStride<1> > const &,
    Eigen::Ref<Eigen::Matrix<int, -1, 1, 0, -1, 1> const, 0,
               Eigen::InnerStride<1> > const &,
    Eigen::Ref<Eigen::Matrix<double, 3, -1, 0, 3, -1> const, 0,
               Eigen::OuterStride<-1> > const &,
    Eigen::Ref<Eigen::Matrix<double, 3, -1, 0, 3, -1> const, 0,
               Eigen::OuterStride<-1> > const &,
    Eigen::Matrix<
        Eigen::AutoDiffScalar<Eigen::Matrix<double, -1, 1, 0, -1, 1> >, -1, -1,
        0, -1, -1> &) const;
template DRAKE_EXPORT void RigidBodyTree::computeContactJacobians<double>(
    KinematicsCache<double> const &,
    Eigen::Ref<Eigen::Matrix<int, -1, 1, 0, -1, 1> const, 0,
               Eigen::InnerStride<1> > const &,
    Eigen::Ref<Eigen::Matrix<int, -1, 1, 0, -1, 1> const, 0,
               Eigen::InnerStride<1> > const &,
    Eigen::Ref<Eigen::Matrix<double, 3, -1, 0, 3, -1> const, 0,
               Eigen::OuterStride<-1> > const &,
    Eigen::Ref<Eigen::Matrix<double, 3, -1, 0, 3, -1> const, 0,
               Eigen::OuterStride<-1> > const &,
    Eigen::Matrix<double, -1, -1, 0, -1, -1> &) const;
