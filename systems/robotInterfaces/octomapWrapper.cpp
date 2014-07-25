#include "mex.h"
#include <iostream>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include "drakeUtil.h"

using namespace std;
using namespace octomap;

void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] )
{
  // Usage:
  //   Constructors/Destructor:
  //    octree = octomapWrapper(resolution);  // constructor: new tree with specified resolution
  //    octree = octomapWrapper(filename);    // constructor: load from file
  //    octomapWrapper(octree);     // destructor
  //
  //   Queries:
  //    results = octomapWrapper(octree,1,pts) // search
  //
  //   Update tree:
  //    octomapWrapper(octree,11,pts,occupied)   // updateNote(pts,occupied).  pts is 3-by-n, occupied is 1-by-n logical
  //
  //   General operations:
  //    octomapWrapper(octree,21,lcmgl_name)  // publish to lcmgl
  //    octomapWrapper(octree,22,filename)    // save to file


  OcTree* tree=NULL;

  if (nrhs==1) {
    if (mxIsNumeric(prhs[0])) {  // constructor w/ resolution
      if (nlhs>0) {
        double resolution = mxGetScalar(prhs[0]);
//        mexPrintf("Creating octree w/ resolution %f\n", resolution);
        tree = new OcTree(resolution);
        plhs[0] = createDrakeMexPointer((void*)tree,"octomapWrapper","OcTree");
      }
    } else if (mxIsChar(prhs[0])) {
      if (nlhs>0) {
        char* filename = mxArrayToString(prhs[0]);
//        mexPrintf("Loading octree from %s\n", filename);
        tree = new OcTree(filename);
        plhs[0] = createDrakeMexPointer((void*)tree,"octomapWrapper","OcTree");
        mxFree(filename);
      }
    } else { // destructor.  note: assumes prhs[0] is a DrakeMexPointer (todo: could check)
//      mexPrintf("Deleting octree\n");
      tree = (OcTree*) getDrakeMexPointer(prhs[0]);
      if (tree) delete tree;
    }
    return;
  }

  tree = (OcTree*) getDrakeMexPointer(prhs[0]);
  int COMMAND = (int) mxGetScalar(prhs[1]);

  switch (COMMAND) {
    case 1:  // search
      {
        mexPrintf("octree search\n");
        if (mxGetM(prhs[2])!=3) mexErrMsgTxt("octomapWrapper: pts must be 3-by-n");
        int n = mxGetN(prhs[2]);
        double* pts = mxGetPr(prhs[2]);
        if (nlhs>0) {
          plhs[0] = mxCreateDoubleMatrix(1,n,mxREAL);
          double* presults = mxGetPr(plhs[0]);
          for (int i=0; i<n; i++) {
            OcTreeNode* result = tree->search(pts[3*i],pts[3*i+1],pts[3*i+2]);
            if (result==NULL) presults[i]=-1.0;
            else presults[i]=result->getOccupancy();
          }
        }
      }
      break;
    case 11:  // add occupied pts
      {
//        mexPrintf("octree updateNode\n");
        if (mxGetM(prhs[2])!=3) mexErrMsgTxt("octomapWrapper: pts must be 3-by-n");
        int n = mxGetN(prhs[2]);
        double* pts = mxGetPr(prhs[2]);
        mxLogical* occupied = mxGetLogicals(prhs[3]);
        for (int i=0; i<n; i++) {
          tree->updateNode(pts[3*i],pts[3*i+1],pts[3*i+2],occupied[i]);
        }
      }
      break;
    default:
      mexErrMsgTxt("octomapWrapper: Unknown command");
  }

}
