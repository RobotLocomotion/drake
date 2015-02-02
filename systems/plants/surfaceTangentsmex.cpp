#include "mex.h"
#include <iostream>
#include <Eigen/Dense>
#include "math.h"

using namespace Eigen;
using namespace std;

/*
 * Mex interface for computing friction cone surface tangent basis vectors given contact normals.
 * Currently uses 4 basis vectors as a #define so the compiler can optimize the resulting Eigen templates
 *
 * based on code from <DRAKE_ROOT>/systems/controllers/controlUtil.cpp
 *
 * MATLAB signature:
 * d = 
 *     surfaceTangents(normals)
 */

#define BASIS_VECTOR_HALF_COUNT 2  //number of basis vectors over 2 (i.e. 4 basis vectors in this case)
#define NORMAL_DIMENSION 3

typedef Matrix<double, NORMAL_DIMENSION, Dynamic> Matrix3xd;
typedef Matrix<double, NORMAL_DIMENSION, BASIS_VECTOR_HALF_COUNT> Matrix3kd;

inline void surfaceTangents(const Vector3d & normal, Matrix3kd & d);

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) 
{
  if (nrhs < 1) {
    mexErrMsgIdAndTxt("Drake:surfaceTangentsmex:NotEnoughInputs","Usage: [d] = surfaceTangentsmex(normals)");
  }
  
  const unsigned int numNormals = mxGetN(prhs[0]);  //number of normal vectors
  const unsigned int dimNormals = mxGetM(prhs[0]);  //dimension of each normal vector
  
  if(dimNormals != NORMAL_DIMENSION) {
  	mexErrMsgIdAndTxt("Drake:surfaceTangentsmex:InvalidNormalDimensions","Normals must be 3-Dimensional");	
  }

  //Mapping Eigen Matrix to existing memory to avoid copy overhead
  Map<Matrix3xd> normals(mxGetPr(prhs[0]), NORMAL_DIMENSION, numNormals); 

  if(nlhs > 0) {
  	int k = 0;
  	const size_t cellDims[] = {1, BASIS_VECTOR_HALF_COUNT};
  	double *cells[BASIS_VECTOR_HALF_COUNT];

  	plhs[0] = mxCreateCellArray(2, (const mwSize*)&cellDims[0]);

    //initialize output cell array
  	for( k = 0 ; k<BASIS_VECTOR_HALF_COUNT ; k++) {
  		mxArray* mxCell = mxCreateDoubleMatrix(NORMAL_DIMENSION, numNormals, mxREAL);
  		cells[k] = mxGetPr(mxCell);
  		mxSetCell(plhs[0], k, mxCell);
  	}

    //fill in cell array with surface tangents
    for(int curNormal = 0; curNormal<numNormals; curNormal++)
    {
    	Matrix3kd d;
    	surfaceTangents(normals.col(curNormal), d);
    	for( k = 0; k<BASIS_VECTOR_HALF_COUNT ; k++) {
    		Map<Matrix3xd> eigenCell(cells[k], NORMAL_DIMENSION, numNormals);
    		eigenCell.col(curNormal) = d.col(k); 
    	}
	}

  } else {
    mexErrMsgIdAndTxt("Drake:surfaceTangentsmex:NotEnoughOutputs","Please take at least one output");	
  }
}

inline void surfaceTangents(const Vector3d & normal, Matrix3kd & d)
{
  Vector3d t1,t2;
  double theta;

  if (1 - normal(2) < 10e-8) { // handle the unit-normal case (since it's unit length, just check z)
  	t1 << 1,0,0;
  } else if(1 + normal(2) < 10e-8) {
    t1 << -1,0,0;  //same for the reflected case
  } else {// now the general case
 	t1 << normal(1), -normal(0) , 0;
    t1 /= sqrt(normal(1)*normal(1) + normal(0)*normal(0));
  }
    
  t2 = t1.cross(normal);
  
  for (int k=0; k<BASIS_VECTOR_HALF_COUNT; k++) {
    theta = k*M_PI/BASIS_VECTOR_HALF_COUNT;
    d.col(k)=cos(theta)*t1 + sin(theta)*t2;
  }
}