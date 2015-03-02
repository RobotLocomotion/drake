#ifndef __DRAKE_CONTACTCONSTRAINTS_UTIL_H__
#define __DRAKE_CONTACTCONSTRAINTS_UTIL_H__

#include <Eigen/Dense>
#include <set>
#include <vector>

#undef DLLEXPORT
#if defined(WIN32) || defined(WIN64)
  #if defined(drakeGeometryUtil_EXPORTS)
    #define DLLEXPORT __declspec( dllexport )
  #else
    #define DLLEXPORT __declspec( dllimport )
  #endif
#else
  #define DLLEXPORT
#endif

#define BASIS_VECTOR_HALF_COUNT 2  //number of basis vectors over 2 (i.e. 4 basis vectors in this case)
#define EPSILON 10e-8

typedef Eigen::Matrix<double, 3, BASIS_VECTOR_HALF_COUNT> Matrix3kd;
typedef Eigen::Matrix<double, 3, Eigen::Dynamic> Matrix3xd;

DLLEXPORT void surfaceTangents(Eigen::Map<Matrix3xd> const & normals, std::vector< Eigen::Map<Matrix3xd> > & tangents);
DLLEXPORT void surfaceTangentsSingle(Eigen::Vector3d const & normal, Matrix3kd & d);
DLLEXPORT void getUniqueBodiesSorted(Eigen::VectorXi const & idxA, Eigen::VectorXi const & idxB, std::vector<int> & bodyIndsSorted);
DLLEXPORT void findContactIndexes(Eigen::VectorXi const & idxList, const int bodyIdx, std::vector<int> & contactIdx);
DLLEXPORT void getBodyPoints(std::vector<int> const & cindA, std::vector<int> const & cindB, Matrix3xd const & xA, Matrix3xd const & xB, Eigen::MatrixXd & bodyPoints);

#endif
