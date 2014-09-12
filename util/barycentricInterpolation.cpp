#include <mex.h>
#include <stdlib.h>
#include <stdint.h>

/* barycentricInterpolation
 *  Looks up the indices and weighted coefficients for a barycentric
 * interpolation on an ndgrid.
 * @param bins is an m-element cell array with bins{i} describing the mesh
 *             discretization along dimention i.  Bin elements must be
 *             sorted in ascending order.
 * @param pts is an m-by-n matrix with m pts to be interpolated
 * @retval indices is a d-by-n list of indices into the ndarray, with
 *             element (i,j) being the ith interpolant of the point j.
 *             For triangulated interpolation (the only one implemented so
 *             far), d = m+1
 * @retval coefs is a d-by-n list of weights for each interpolant, such
 *             that, for instance, the original pts(:,i) could be
 *             reconstructed using
 *         pt(:,j) = sum_i coefs(i,j) * gridpoint(:,indices(i,j));
 *           aka pt(:,j) = gridpoint(:,indices(:,j))*coefs(:,j)
 * Note that points off the grid are cropped to the closest point (in an
 * l1 sense) on the grid
 *
 * For a simple description of barycentric interpolation, see
 *  Scott Davies, "Multidimensional Triangulation... ", NIPS, 1996
 */

/* helpers for qsort */
typedef struct {
  int dim;
  double fracway;
} bary;
int barycomp(const void* b1, const void* b2){
  bary *bb1 = (bary*)b1, *bb2 = (bary*)b2;
  int order;
  if (bb1->fracway > bb2->fracway) order = 1;
  else if (bb1->fracway < bb2->fracway) order = -1;
  else order = 0;
  return order;
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  if (nrhs < 2) {
    mexPrintf("Usage: [indices,coefs] = barycentricInterpolation(bins,pts)\n");
    return;
  }

  double* pts = mxGetPr(prhs[1]);

  int i,j;
  int m = mxGetM(prhs[1]), n=mxGetN(prhs[1]), d=m+1;

  if (mxGetNumberOfElements(prhs[0])!=m)
    mexErrMsgIdAndTxt("Drake:barycentricInterpolation:BadInput","the number of bins must equal the dimension of the points");

  int* binsize = new int[m];
  double** bins = new double*[m];
  mxArray* pbin;
  int* nskip = new int[m];  // offsets for inline sub2ind
  for (i=0; i<m; i++) {
    pbin = mxGetCell(prhs[0],i);
    binsize[i] = mxGetNumberOfElements(pbin);
    bins[i]=mxGetPr(pbin);
    if (i==0) nskip[i]=1;
    else nskip[i] = nskip[i-1]*binsize[i-1];
  }

  plhs[0] = mxCreateNumericMatrix(d,n,mxINT64_CLASS,mxREAL);

  int64_t* indices = (int64_t*) mxGetData(plhs[0]);
  double* coefs;
  if (nlhs>1) {
    plhs[1] = mxCreateDoubleMatrix(d,n,mxREAL);
    coefs = mxGetPr(plhs[1]);
  } else {
    coefs = new double[d*n];
  }

  bary* b = new bary[d];

  for (j=0; j<n; j++) {
    int64_t sidx = 1;  // index of the top right corner
    for (i=0; i<m; i++) {
      double pt = pts[i];
      double* current_bin = bins[i];
      int current_bin_size=binsize[i];
      b[i].dim = i;

      // truncate back onto the grid if it's off the edge
      if (pt>current_bin[current_bin_size-1]) pt=current_bin[current_bin_size-1];
      if (pt<current_bin[0]) pt=current_bin[0];

      // note: could do smarter search here
      int next_bin_index=1;
      while (pt>current_bin[next_bin_index]) next_bin_index++;

      if (current_bin_size==1) { // support singleton dimensions
        // sidx is unchanged
        b[i].fracway = 1.0;
      } else {
        sidx += nskip[i]*next_bin_index;  
        b[i].fracway = (pt - current_bin[next_bin_index-1])/(current_bin[next_bin_index]-current_bin[next_bin_index-1]);
      }
    }

    // sort dimensions based on fracway (lowest to highest)
    qsort(b,m,sizeof(bary),barycomp);
    b[m].fracway = 1.0;  // final element, to make the loop below cleaner
    b[m].dim = m-1;

    // top right corner
    indices[0] = sidx;
    coefs[0] = b[0].fracway;

    // move down the box.  sorted list of dimensions tells us which order to descend.
    for (i=0; i<m; i++) {
      if (binsize[b[i].dim]>1) // support singleton dimension
        sidx -= nskip[b[i].dim];
      indices[i+1] = sidx;
      coefs[i+1] = b[i+1].fracway - b[i].fracway;
    }

    // move pointers ahead to the next point:
    pts += m;
    indices += d;
    coefs += d;
  }

  // clean up
  delete[] binsize;
  delete[] bins;
  delete[] nskip;
  delete[] b;
  if (nlhs<2) delete[] coefs;
}
