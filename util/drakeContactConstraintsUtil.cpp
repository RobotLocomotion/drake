#include "drakeContactConstraintsUtil.h"
#include <iostream>

using namespace Eigen;
using namespace std;


// Computes surface tangent vectors for many normal vectors at once
// INPUTS:
//   normals: (3 x m) matrix where each column is a normal vector in world coordinates
// OUTPUTS:
//   tangents: list of k (3 x m) matrices where each column contains one of the k tangent vectors
//              for the corresponding normal.
// NOTE:
//  k = BASIS_VECTOR_HALF_COUNT is defined as a preprocessor directive so that 
//      Eigen templates can be optimized at compile time
void surfaceTangents(Map<Matrix3xd> const & normals, std::vector< Map<Matrix3xd> > & tangents)
{
  const int numContactPairs = normals.cols();
  for (int curNormal = 0 ; curNormal < numContactPairs; curNormal++) {
    Matrix3kd d;
    surfaceTangentsSingle(normals.col(curNormal), d);
    for (int k = 0 ; k < BASIS_VECTOR_HALF_COUNT ; k++) {
      tangents[k].col(curNormal) = d.col(k);
    }
  }
}

// Computes surface tangent vectors for a single normal vector
// INPUTS:
//   normal: (3 x 1) normal vector in world coordinates
// OUTPUTS:
//   d: (3 x k) matrix where the k columns contain the surface tangents
// NOTE:
//  k = BASIS_VECTOR_HALF_COUNT is defined as a preprocessor directive so that 
//      Eigen templates can be optimized at compile time
void surfaceTangentsSingle(Vector3d const & normal, Matrix3kd & d)
{
  Vector3d t1,t2;
  double theta;
  
  if (1.0 - normal(2) < EPSILON) { // handle the unit-normal case (since it's unit length, just check z)
    t1 << 1.0, 0.0, 0.0;
  } else if (1 + normal(2) < EPSILON) {
    t1 << -1.0, 0.0, 0.0;  //same for the reflected case
  } else {// now the general case
    t1 << normal(1), -normal(0), 0.0;
    t1 /= sqrt(normal(1)*normal(1) + normal(0)*normal(0));
  }
  
  t2 = t1.cross(normal);

  for (int k = 0 ; k < BASIS_VECTOR_HALF_COUNT ; k++) {
    theta = k*M_PI/BASIS_VECTOR_HALF_COUNT;
    d.col(k)=cos(theta)*t1 + sin(theta)*t2;
  }
}


// Consolidates unique body indexes from idXA and idxB into a sorted set
// INPUTS:
//   idxA mx1 vector of body indexes for body A in m possible contacts
//   idxB mx1 vector of bodyh indexes for body B in m possible contacts
// OUTPUTS:
//   bodyIndsSorted a set of unique, sorted(ascending) body indexes participating in contact pairs
void getUniqueBodiesSorted(VectorXi const & idxA, VectorXi const & idxB, std::vector<int> & bodyIndsSorted)
{
  size_t m = idxA.size();
  std::set<int> bodyInds;
  
  for (int i = 0 ; i < m ; i++) {  
    bodyInds.insert(idxA[i]);
    bodyInds.insert(idxB[i]);
  }
  
  bodyIndsSorted.clear();

  for (std::set<int>::const_iterator citer = bodyInds.begin() ; citer != bodyInds.end() ; citer++) {
    if ( *citer > 1 ) {
      bodyIndsSorted.push_back(*citer);
    }
  }
  
  sort(bodyIndsSorted.begin(), bodyIndsSorted.end());
}


//Finds the n occurences of the body index in the original list of m contact indexes
// INPUTS:
//   idxList: (m x 1) Indexes of m bodies for possible contact pairs
//   bodyIdx: the body index to search for
// OUTPUTS:
//   contactIdx: the list of n indexes into idxList where bodyIdx occurred
void findContactIndexes(VectorXi const & idxList, const int bodyIdx, std::vector<int> & contactIdx)
{
  int m = idxList.size();
  contactIdx.clear();
  for (int i = 0 ; i < m ; i++) {
    if (idxList[i] == bodyIdx) {
      contactIdx.push_back(i); //zero-based index 
    }
  }
}

// Gets a concatenated list of contact points corresponding to a single body regardless of whether it is body A or body B
//  This allows all the points to be transformed into world space using a single forwardKin call.
// INPUTS:
//   cindA: list of indexes into the original list of m contact pairs where the body appeared as body A
//   cindB: list of iindexes into the original list of m contact pairs where the body appeared as body B
//   xA: (3 x m) matrix where each column represents a contact point on body A
//   xB: (3 x m) matrix where each column represents a contact point on body B
// OUTPUTS:
//   bodyPoints: (4 x n) matrix of contact points containing occurences of body A contact points followed
//     by body B contactpoints where n = size(cindA) + size(cindB)
// NOTE: the output is a matrix of 4-vector columns in homogeneous coordinates (x,y,z,1)'
void getBodyPoints(std::vector<int> const & cindA, std::vector<int> const & cindB, Matrix3xd const & xA, Matrix3xd const & xB, MatrixXd & bodyPoints)
{
  int i = 0;
  int numPtsA = cindA.size();
  int numPtsB = cindB.size();

  bodyPoints.resize(4, numPtsA + numPtsB);

  for (i = 0 ; i < numPtsA ; i++ ) {
    bodyPoints.col(i) << xA.col(cindA[i]), 1.0; //homogeneous coordinates
  }

  for (i = 0 ; i < numPtsB ; i++ ) {
    bodyPoints.col(numPtsA + i) << xB.col(cindB[i]), 1.0;
  }
}