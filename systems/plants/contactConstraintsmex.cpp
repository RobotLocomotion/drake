#include "mex.h"
#include <iostream>
#include "drakeUtil.h"
#include "RigidBodyManipulator.h"
#include "math.h"
#include <Eigen/Sparse>

using namespace Eigen;
using namespace std;

/*
 * MATLAB signature: [d, n, D, dn, dD] = contactConstraintsmex(model_ptr, normals, idxA, idxB, xA, xB)
 * 
 * Inputs:
 *  model_ptr = mex_model_ptr to the RBM
 *  normals = (3xm) contact normals (from B to A) for each of m possible contacts in world space
 *  idxA = 1-based body indexes of body A for m possible contacts
 *  idxB = 1-based body indexes of body B for m possible contacts
 *  xA = (3xm) matrix of contact positions in body A space
 *  xB = (3xm) matrix of contact positions in body B space 
 * Outputs:
 *  d = {k}(3xm) surface tangent basis vectors for the friction cone approximation in world space
 *     **NOTE** k represents the half-count for basis vectors. (e.g. k = 2 means 4 basis vectors)
 *   n = m x (num_dof) dphi/dq normal vectors in joint coordinates
 *   D = {2k} m x (num_dof) dD/dq surface tangent basis vectors in joint coordinates
 *   dn = m*num_dof x num_dof dn/dq second derivative of contact distances with respect to state
 *   dD = {2k} m*num_dof x numdof dD/dq second derivate of basis vectors with respect to state
 */

#define BASIS_VECTOR_HALF_COUNT 2  //number of basis vectors over 2 (i.e. 4 basis vectors in this case)
#define EPSILON 10e-8

typedef Matrix<double, 3, BASIS_VECTOR_HALF_COUNT> Matrix3kd;
typedef Matrix<double, 3, Dynamic> Matrix3xd;

inline void getUniqueBodiesSorted(VectorXi const & idxA, VectorXi const & idxB, vector<int> & bodyIndsSorted)
{
	size_t m = idxA.size();
	set<int> bodyInds;

	for(int i = 0 ; i < m ; i++)
	{  
		bodyInds.insert(idxA[i]);
		bodyInds.insert(idxB[i]);
	}

	bodyIndsSorted.clear();

	for(set<int>::const_iterator citer = bodyInds.begin() ; citer != bodyInds.end() ; citer++)
	{
		if( *citer > 1 )
		{
			bodyIndsSorted.push_back(*citer);
		}
	}

	sort(bodyIndsSorted.begin(), bodyIndsSorted.end());
}

inline void getBodyPoints(vector<int> const & cindA, vector<int> const & cindB, Matrix3xd const & xA, Matrix3xd const & xB, const int num_dof, MatrixXd & bodyPoints)
{
	int i = 0;
	int numPtsA = cindA.size();
	int numPtsB = cindB.size();

	bodyPoints.resize(4, numPtsA + numPtsB);

	for(i = 0 ; i < numPtsA ; i++ ) {
		bodyPoints.col(i) << xA.col(cindA[i]) , 1; //homogeneous coordinates
	}

	for(i = 0 ; i < numPtsB ; i++ ) {
		bodyPoints.col(numPtsA + i) << xB.col(cindB[i]), 1;
	}
}

inline void findContactIndexes(VectorXi const & idxList, const int bodyIdx, vector<int> & contactIdx)
{
	int m = idxList.size();
	contactIdx.clear();
	for(int i = 0 ; i < m ; i++) {
		if(idxList[i] == bodyIdx) {
			contactIdx.push_back(i); //zero-based index 
		}
	}
}

inline void accumulateJacobian(RigidBodyManipulator *model, const int bodyInd, MatrixXd const & bodyPoints, vector<int> const & cindA, vector<int> const & cindB, MatrixXd & J)
{
	const int nq = J.cols();
	const int numPts = bodyPoints.cols();
	const size_t numCA = cindA.size();
	const size_t numCB = cindB.size();
	const size_t offset = 3*numCA;

	MatrixXd J_tmp(3*numPts, nq);
	model->forwardJac(bodyInd - 1, bodyPoints, 0, J_tmp);

	//add contributions from points in xA
	for(int x = 0 ; x < numCA ; x++)
	{
		J.block(3*cindA[x], 0, 3, nq) += J_tmp.block(3*x, 0, 3, nq);
	}

	//subtract contributions from points in xB
	for(int x = 0 ; x < numCB ; x++)
	{
		J.block(3*cindB[x], 0, 3, nq) -= J_tmp.block(offset + 3*x, 0, 3, nq);
	}
}

inline void accumulateSecondOrderJacobian(RigidBodyManipulator *model, const int bodyInd, MatrixXd const & bodyPoints, vector<int> const & cindA, vector<int> const & cindB, MatrixXd & dJ)
{
	const int dJCols = dJ.cols(); //nq^2 instead of nq
	const int numPts = bodyPoints.cols();
	const size_t numCA = cindA.size();
	const size_t numCB = cindB.size();
	const size_t offset = 3*numCA;
	MatrixXd dJ_tmp(3*numPts, dJCols);
	model->forwarddJac(bodyInd - 1, bodyPoints, dJ_tmp); //dJac instead of Jac

	//add contributions from points in xA
	for(int x = 0 ; x < numCA ; x++)
	{
		dJ.block(3*cindA[x], 0, 3, dJCols) += dJ_tmp.block(3*x, 0, 3, dJCols);
	}

	//subtract contributions from points in xB
	for(int x = 0 ; x < numCB ; x++)
	{
		dJ.block(3*cindB[x], 0, 3, dJCols) -= dJ_tmp.block(offset + 3*x, 0, 3, dJCols);
	}
}

inline void buildSparseMatrix(Matrix3xd const & pts, SparseMatrix<double> & sparse)
{
	const int m = pts.cols();
	const int numNonZero = 3*m;

	sparse.resize(m, numNonZero);
	sparse.reserve(VectorXi::Constant(numNonZero, 1));

	int j = 0;
	for(int i = 0 ; i < m ; i++) {
		for(int k = 0 ; k < 3 ; k++) {
			sparse.insert(i, j) =  pts(j);
			j++;
		}
	}
}

void computeContactJacobians(RigidBodyManipulator * const model, Map<VectorXi> const & idxA, Map<VectorXi> const & idxB, Map<Matrix3xd> const & xA, Map<Matrix3xd> const & xB, const bool compute_second_derivatives, MatrixXd & J, MatrixXd & dJ)
{
	vector<int> bodyInds;
	const int nq = model->num_dof;
	const int numContactPairs = xA.cols();

	J = MatrixXd::Zero(3*numContactPairs, nq);
 	dJ = MatrixXd::Zero(3*numContactPairs, nq*nq);
	
	getUniqueBodiesSorted(idxA, idxB, bodyInds);
	
	const int numUniqueBodies = bodyInds.size();

	for(int i = 0; i < numUniqueBodies ; i++) {
		const int bodyInd = bodyInds[i];
		vector<int> cindA, cindB;
		MatrixXd bodyPoints;
		findContactIndexes(idxA, bodyInd, cindA);
		findContactIndexes(idxB, bodyInd, cindB);
		getBodyPoints(cindA, cindB, xA, xB, nq, bodyPoints);
		accumulateJacobian(model, bodyInd, bodyPoints, cindA, cindB, J);
		if(compute_second_derivatives)
		{
			accumulateSecondOrderJacobian(model, bodyInd, bodyPoints, cindA, cindB, dJ);
		}
	} 
}

//computes surface tangent vectors for a single normal
inline void surfaceTangents(Vector3d const & normal, Matrix3kd & d)
{
	Vector3d t1,t2;
	double theta;

	if (1 - normal(2) < EPSILON) { // handle the unit-normal case (since it's unit length, just check z)
		t1 << 1,0,0;
	} else if(1 + normal(2) < EPSILON) {
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


//computes surface tangent vectors for many normals at once
inline void surfaceTangents(Map<Matrix3xd> const & normals, vector< Map<Matrix3xd> > & tangents)
{
	const int numContactPairs = normals.cols();

	for(int curNormal = 0 ; curNormal < numContactPairs; curNormal++)
	{
		Matrix3kd d;
		surfaceTangents(normals.col(curNormal), d);
		for(int k = 0 ; k < BASIS_VECTOR_HALF_COUNT ; k++)
		{
			tangents[k].col(curNormal) = d.col(k);
		}
	}
}

//forms a mex cell array and computes the surface tangents in-place using maps
inline mxArray* getTangentsArray(Map<Matrix3xd> normals)
{
	const int numContactPairs = normals.cols();
	const mwSize cellDims[] = {1, BASIS_VECTOR_HALF_COUNT};
	mxArray* tangentCells = mxCreateCellArray(2, cellDims);
	
	vector< Map<Matrix3xd> > tangents;
	for(int k = 0 ; k < BASIS_VECTOR_HALF_COUNT ; k++)
	{
		mxArray *cell = mxCreateDoubleMatrix(3, numContactPairs, mxREAL );		
		tangents.push_back(Map<Matrix3xd>(mxGetPr(cell), 3, numContactPairs));
		mxSetCell(tangentCells, k, cell);
	}

	surfaceTangents(normals, tangents);

	return tangentCells;
}

void mexFunction( int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[] ) {
	//valid usage modes: 
  // 1 input + 1 output tangents only mode
  // 6 inputs + 3 outputs tangents and first derivatives
  // 6 inputs + 5 outputs tangents, first + second derivatives
  if (!((nrhs == 1 && nlhs == 1) || (nrhs == 6 && nlhs == 3) || (nrhs == 6 && nlhs == 5))) {
		mexErrMsgIdAndTxt("Drake:contactConstraintsmex:NotEnoughInputs","Usage: \nd = contactConstraintsmex(normal)\n[d, n, D] = contactConstraintsmex(normal, model_ptr, idxA, idxB, xA, xB)\n[d, n, D, dn, dD] = contactConstraintsmex(normal, model_ptr, idxA, idxB, xA, xB)");
	}

	if(nrhs >= 4 && mxGetN(prhs[2]) != mxGetN(prhs[3])) {
		mexErrMsgIdAndTxt("Drake:contactConstraintsmex:InvalidBodyIndexes", "idxA and idxB must be the same size");
	}

	if(nrhs >= 6 && mxGetN(prhs[4]) != mxGetN(prhs[5])) {
		mexErrMsgIdAndTxt("Drake:contactConstraintsmex:InvalidBodyPoints", "xA and xB must be the same size");
	} 

	if(nrhs >= 6 && (mxGetM(prhs[4]) != 3 || mxGetM(prhs[5]) != 3)) {
		mexErrMsgIdAndTxt("Drake:contactConstraintsmex:InvalidBodyPointsDimension", "body points xA and xB must be 3 dimensional");
	}

	if(nrhs >= 5 && mxGetN(prhs[1]) != mxGetN(prhs[4])) {
		mexErrMsgIdAndTxt("Drake:contactConstraintsmex:InvalidBodyPointsDimension", "normals must match the number of contact points");
	}

	const int numContactPairs = mxGetN(prhs[1]);
	const Map<Matrix3xd> normals(mxGetPr(prhs[1]), 3, numContactPairs); //contact normals in world space
	
  	const bool compute_second_derivatives = nlhs > 3;

	if(nlhs > 0) {
		plhs[0] = getTangentsArray(normals);
	}

	if(nrhs < 2) 
	{
		return; // just return tangents
	}

	const Map<VectorXi> idxA((int*)mxGetData(prhs[2]), numContactPairs); //collision pairs index of body A
	const Map<VectorXi> idxB((int*)mxGetData(prhs[3]), numContactPairs); //collision pairs index of body B
	const Map<Matrix3xd> xA(mxGetPr(prhs[4]), 3, numContactPairs); //contact point in body A space
	const Map<Matrix3xd> xB(mxGetPr(prhs[5]), 3, numContactPairs); //contact point in body B space

	RigidBodyManipulator *model= (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);

	if(model != NULL) {
		const int nq = model->num_dof;
		if(nlhs > 1) {

			MatrixXd J; 
			MatrixXd dJ;
			vector<int> bodyInds;
			SparseMatrix<double> sparseNormals;

			computeContactJacobians(model, idxA, idxB, xA, xB, compute_second_derivatives, J, dJ);			
			buildSparseMatrix(normals, sparseNormals);

			plhs[1] = mxCreateDoubleMatrix(numContactPairs, nq, mxREAL);
			Map<MatrixXd> n(mxGetPr(plhs[1]), numContactPairs, nq);
			n = sparseNormals * J; //dphi/dq


			if(nlhs > 2) {
				const mwSize cellDims[] = {1, 2*BASIS_VECTOR_HALF_COUNT};
				plhs[2] = mxCreateCellArray(2, cellDims);

				if(nlhs > 4) {
					plhs[4] = mxCreateCellArray(2, cellDims);
				}
				
				for(int k = 0 ; k < BASIS_VECTOR_HALF_COUNT ; k++) { //for each friction cone basis vector
					Map<Matrix3xd> dk(mxGetPr(mxGetCell(plhs[0], k)), 3, numContactPairs);
					SparseMatrix<double> sparseTangents;
					buildSparseMatrix(dk, sparseTangents);
					mxArray *D_cell = mxCreateDoubleMatrix(numContactPairs, nq, mxREAL);
					mxArray *D_cell_reflected = mxCreateDoubleMatrix(numContactPairs, nq, mxREAL);
					Map<MatrixXd> Dk(mxGetPr(D_cell), numContactPairs, nq);
					Map<MatrixXd> Dk_reflected(mxGetPr(D_cell_reflected), numContactPairs, nq);
					Dk = sparseTangents * J; //dd/dq
					Dk_reflected = -Dk;
					mxSetCell(plhs[2], k, D_cell);
					mxSetCell(plhs[2], k + BASIS_VECTOR_HALF_COUNT, D_cell_reflected);
					if(compute_second_derivatives) {
						if(nlhs > 4) {
							mxArray *dD_cell = mxCreateDoubleMatrix(numContactPairs * nq, nq, mxREAL);
							Map<MatrixXd> dD(mxGetPr(dD_cell), numContactPairs , nq * nq);
							dD = sparseTangents * dJ;
							mxSetCell(plhs[4], k, dD_cell);
							mxArray *dD_cell_reflected = mxCreateDoubleMatrix(numContactPairs * nq, nq, mxREAL);
							Map<MatrixXd> dD_reflected(mxGetPr(dD_cell_reflected), numContactPairs, nq*nq);
							dD_reflected = -dD;
							mxSetCell(plhs[4], k + BASIS_VECTOR_HALF_COUNT, dD_cell_reflected);
						}
					}
				}
			}

			if(nlhs > 3) {
				plhs[3] = mxCreateDoubleMatrix(numContactPairs * nq, nq, mxREAL);
				Map<MatrixXd> dn(mxGetPr(plhs[3]), numContactPairs, nq*nq);
				dn = sparseNormals * dJ;
			}
		}
	}
}


