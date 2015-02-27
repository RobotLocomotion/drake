#ifndef _CONTROL_UTIL_H_
#define _CONTROL_UTIL_H_

#include <mex.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <set>
#include <vector>
#include <Eigen/Dense>

#ifdef USE_MAPS
#include "terrain-map/TerrainMap.hpp"
#endif

#include "drake/RigidBodyManipulator.h"

#if defined(WIN32) || defined(WIN64)
  #if defined(drakeControlUtil_EXPORTS)
    #define drakeControlUtilEXPORT __declspec( dllexport )
  #else
    #define drakeControlUtilEXPORT __declspec( dllimport )
  #endif
#else
  #define drakeControlUtilEXPORT
#endif


const int m_surface_tangents = 2;  // number of faces in the friction cone approx

#define EPSILON 10e-8

typedef Matrix<double, 6,1> Vector6d;

typedef struct _support_state_element
{
  int body_idx;
  std::vector<Vector4d> contact_pts;
  int contact_surface;
  bool support_logic_map[4];
} SupportStateElement;

drakeControlUtilEXPORT std::vector<SupportStateElement> parseSupportData(const mxArray* supp_data);

drakeControlUtilEXPORT bool isSupportElementActive(SupportStateElement* se, bool contact_force_detected, bool kinematic_contact_detected);

drakeControlUtilEXPORT Matrix<bool, Dynamic, 1> getActiveSupportMask(RigidBodyManipulator* r, void* map_ptr, double* q, double* qd, std::vector<SupportStateElement> available_supports, Matrix<bool, Dynamic, 1> contact_force_detected, double contact_threshold, double terrain_height);

drakeControlUtilEXPORT std::vector<SupportStateElement> getActiveSupports(RigidBodyManipulator* r, void* map_ptr, double* q, double* qd, std::vector<SupportStateElement> available_supports, Matrix<bool, Dynamic, 1> contact_force_detected, double contact_threshold, double terrain_height);

template <typename DerivedA, typename DerivedB>
drakeControlUtilEXPORT void getRows(std::set<int> &rows, MatrixBase<DerivedA> const &M, MatrixBase<DerivedB> &Msub);

template <typename DerivedA, typename DerivedB>
drakeControlUtilEXPORT void getCols(std::set<int> &cols, MatrixBase<DerivedA> const &M, MatrixBase<DerivedB> &Msub);

template <typename DerivedPhi1, typename DerivedPhi2, typename DerivedD>
drakeControlUtilEXPORT void angleDiff(MatrixBase<DerivedPhi1> const &phi1, MatrixBase<DerivedPhi2> const &phi2, MatrixBase<DerivedD> &d);

drakeControlUtilEXPORT mxArray* myGetProperty(const mxArray* pobj, const char* propname);
drakeControlUtilEXPORT mxArray* myGetField(const mxArray* pobj, const char* propname);
drakeControlUtilEXPORT bool inSupport(std::vector<SupportStateElement> supports, int body_idx);
drakeControlUtilEXPORT void collisionDetect(void* map_ptr, Vector3d const & contact_pos, Vector3d &pos, Vector3d *normal, double terrain_height);
drakeControlUtilEXPORT void surfaceTangents(const Vector3d & normal, Matrix<double,3,m_surface_tangents> & d);
drakeControlUtilEXPORT int contactPhi(RigidBodyManipulator* r, SupportStateElement& supp, void *map_ptr, VectorXd &phi, double terrain_height);
drakeControlUtilEXPORT int contactConstraints(RigidBodyManipulator *r, int nc, std::vector<SupportStateElement>& supp, void *map_ptr, MatrixXd &n, MatrixXd &D, MatrixXd &Jp, MatrixXd &Jpdot,double terrain_height);
drakeControlUtilEXPORT int contactConstraintsBV(RigidBodyManipulator *r, int nc, double mu, std::vector<SupportStateElement>& supp, void *map_ptr, MatrixXd &B, MatrixXd &JB, MatrixXd &Jp, MatrixXd &Jpdot, MatrixXd &normals, double terrain_height);
drakeControlUtilEXPORT MatrixXd individualSupportCOPs(RigidBodyManipulator* r, const std::vector<SupportStateElement>& active_supports, const MatrixXd& normals, const MatrixXd& B, const VectorXd& beta);


#endif
