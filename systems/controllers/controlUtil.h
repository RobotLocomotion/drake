#ifndef _CONTROL_UTIL_H_
#define _CONTROL_UTIL_H_

#include <mex.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <set>
#include <vector>
#include <Eigen/Dense>

#ifdef USE_MAPS
#include "mexmaps/MapLib.hpp"
#include <maps/ViewBase.hpp>
#endif

#include "drake/RigidBodyManipulator.h"

const int m_surface_tangents = 2;  // number of faces in the friction cone approx

typedef Matrix<double, 6,1> Vector6d;

typedef struct _support_state_element
{
  int body_idx;
  std::set<int> contact_pt_inds;
  int contact_surface;
} SupportStateElement;

template <typename DerivedA, typename DerivedB>
void getRows(std::set<int> &rows, MatrixBase<DerivedA> const &M, MatrixBase<DerivedB> &Msub);

template <typename DerivedA, typename DerivedB>
void getCols(std::set<int> &cols, MatrixBase<DerivedA> const &M, MatrixBase<DerivedB> &Msub);

template <typename DerivedPhi1, typename DerivedPhi2, typename DerivedD>
void angleDiff(MatrixBase<DerivedPhi1> const &phi1, MatrixBase<DerivedPhi2> const &phi2, MatrixBase<DerivedD> &d);

template <int Rows, int Cols>
mxArray* eigenToMatlab(Matrix<double,Rows,Cols> &m);

mxArray* myGetProperty(const mxArray* pobj, const char* propname);
mxArray* myGetField(const mxArray* pobj, const char* propname);
bool inSupport(std::vector<SupportStateElement> supports, int body_idx);
void collisionDetect(void* map_ptr, Vector3d const & contact_pos, Vector3d &pos, Vector3d *normal, double terrain_height);
void surfaceTangents(const Vector3d & normal, Matrix<double,3,m_surface_tangents> & d);
int contactPhi(RigidBodyManipulator* r, SupportStateElement& supp, void *map_ptr, VectorXd &phi, double terrain_height);
int contactConstraints(RigidBodyManipulator *r, int nc, std::vector<SupportStateElement>& supp, void *map_ptr, MatrixXd &n, MatrixXd &D, MatrixXd &Jp, MatrixXd &Jpdot,double terrain_height);
int contactConstraintsBV(RigidBodyManipulator *r, int nc, double mu, std::vector<SupportStateElement>& supp, void *map_ptr, MatrixXd &B, MatrixXd &JB, MatrixXd &Jp, MatrixXd &Jpdot, MatrixXd &normals, double terrain_height);
MatrixXd individualSupportCOPs(RigidBodyManipulator* r, const std::vector<SupportStateElement>& active_supports, const MatrixXd& normals, const MatrixXd& B, const VectorXd& beta);


#endif
