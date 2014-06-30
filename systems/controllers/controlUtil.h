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

typedef struct _support_state_element
{
  int body_idx;
  std::set<int> contact_pt_inds;
  int contact_surface;
} SupportStateElement;

// helper function for shuffling debugging data back into matlab
template <int Rows, int Cols>
mxArray* eigenToMatlab(Matrix<double,Rows,Cols> &m)
{
  mxArray* pm = mxCreateDoubleMatrix(m.rows(),m.cols(),mxREAL);
  if (m.rows()*m.cols()>0)
    memcpy(mxGetPr(pm),m.data(),sizeof(double)*m.rows()*m.cols());
  return pm;
}

template <typename DerivedA, typename DerivedB>
void getRows(std::set<int> &rows, MatrixBase<DerivedA> const &M, MatrixBase<DerivedB> &Msub)
{
  if (rows.size()==M.rows()) {
    Msub = M; 
    return;
  }
  
  int i=0;
  for (std::set<int>::iterator iter=rows.begin(); iter!=rows.end(); iter++)
    Msub.row(i++) = M.row(*iter);
}

template <typename DerivedA, typename DerivedB>
void getCols(std::set<int> &cols, MatrixBase<DerivedA> const &M, MatrixBase<DerivedB> &Msub)
{
  if (cols.size()==M.cols()) {
    Msub = M;
    return;
  }
  int i=0;
  for (std::set<int>::iterator iter=cols.begin(); iter!=cols.end(); iter++)
    Msub.col(i++) = M.col(*iter);
}

mxArray* myGetProperty(const mxArray* pobj, const char* propname);
mxArray* myGetField(const mxArray* pobj, const char* propname);
bool inSupport(std::vector<SupportStateElement> supports, int body_idx);
void collisionDetect(void* map_ptr, Vector3d const & contact_pos, Vector3d &pos, Vector3d *normal, double terrain_height);
void surfaceTangents(const Vector3d & normal, Matrix<double,3,m_surface_tangents> & d);
int contactPhi(RigidBodyManipulator* r, SupportStateElement& supp, void *map_ptr, VectorXd &phi, double terrain_height);
int contactConstraints(RigidBodyManipulator *r, int nc, std::vector<SupportStateElement>& supp, void *map_ptr, MatrixXd &n, MatrixXd &D, MatrixXd &Jp, MatrixXd &Jpdot,double terrain_height);
int contactConstraintsBV(RigidBodyManipulator *r, int nc, double mu, std::vector<SupportStateElement>& supp, void *map_ptr, MatrixXd &B, MatrixXd &JB, MatrixXd &Jp, MatrixXd &Jpdot,double terrain_height);
#endif