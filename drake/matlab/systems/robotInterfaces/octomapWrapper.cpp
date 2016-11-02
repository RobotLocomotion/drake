#include <mex.h>

#include <iostream>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include "drake/matlab/util/drakeMexUtil.h"

using namespace std;
using namespace octomap;

DLL_EXPORT_SYM
void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
  // Usage:
  //   Constructors/Destructor:
  //    octree = octomapWrapper(resolution);  // constructor: new tree with
  //    specified resolution
  //    octree = octomapWrapper(filename);    // constructor: load from file
  //    octomapWrapper(octree);     // destructor
  //
  //   Queries:
  //    results = octomapWrapper(octree, 1, pts) // search
  //    leaf_nodes = octomapWrapper(octree, 2)  // getLeafNodes
  //
  //   Update tree:
  //    octomapWrapper(octree, 11, pts, occupied)  // updateNote(pts, occupied).
  //    pts is 3-by-n, occupied is 1-by-n logical
  //
  //   General operations:
  //    octomapWrapper(octree, 21, filename)    // save to file

  OcTree* tree = NULL;

  if (nrhs == 1) {
    if (mxIsNumeric(prhs[0])) {  // constructor w/ resolution
      if (nlhs > 0) {
        double resolution = mxGetScalar(prhs[0]);
        //        mexPrintf("Creating octree w/ resolution %f\n", resolution);
        tree = new OcTree(resolution);
        plhs[0] = createDrakeMexPointer((void*)tree, "OcTree");
      }
    } else if (mxIsChar(prhs[0])) {
      if (nlhs > 0) {
        char* filename = mxArrayToString(prhs[0]);
        //        mexPrintf("Loading octree from %s\n", filename);
        tree = new OcTree(filename);
        plhs[0] = createDrakeMexPointer((void*)tree, "OcTree");
        mxFree(filename);
      }
    } else {  // destructor.  note: assumes prhs[0] is a DrakeMexPointer (todo:
              // could check)
              //      mexPrintf("Deleting octree\n");
      destroyDrakeMexPointer<OcTree*>(prhs[0]);
    }
    return;
  }

  tree = (OcTree*)getDrakeMexPointer(prhs[0]);
  int COMMAND = (int)mxGetScalar(prhs[1]);

  switch (COMMAND) {
    case 1:  // search
    {
      mexPrintf("octree search\n");
      if (mxGetM(prhs[2]) != 3)
        mexErrMsgTxt("octomapWrapper: pts must be 3-by-n");
      int n = mxGetN(prhs[2]);
      double* pts = mxGetPrSafe(prhs[2]);
      if (nlhs > 0) {
        plhs[0] = mxCreateDoubleMatrix(1, n, mxREAL);
        double* presults = mxGetPrSafe(plhs[0]);
        for (int i = 0; i < n; i++) {
          OcTreeNode* result =
              tree->search(pts[3 * i], pts[3 * i + 1], pts[3 * i + 2]);
          if (result == NULL)
            presults[i] = -1.0;
          else
            presults[i] = result->getOccupancy();
        }
      }
    } break;
    case 2:  // get leaf nodes
    {
      //      mexPrintf("octree get leaf nodes\n");
      int N = tree->getNumLeafNodes();
      plhs[0] = mxCreateDoubleMatrix(3, N, mxREAL);
      double* leaf_xyz = mxGetPrSafe(plhs[0]);

      double* leaf_value = NULL, * leaf_size = NULL;
      if (nlhs > 1) {  // return value
        plhs[1] = mxCreateDoubleMatrix(1, N, mxREAL);
        leaf_value = mxGetPrSafe(plhs[1]);
      }
      if (nlhs > 2) {  // return size
        plhs[2] = mxCreateDoubleMatrix(1, N, mxREAL);
        leaf_size = mxGetPrSafe(plhs[2]);
      }

      for (OcTree::leaf_iterator leaf = tree->begin_leafs(),
                                 end = tree->end_leafs();
           leaf != end; ++leaf) {
        leaf_xyz[0] = leaf.getX();
        leaf_xyz[1] = leaf.getY();
        leaf_xyz[2] = leaf.getZ();
        leaf_xyz += 3;
        if (leaf_value) *leaf_value++ = leaf->getValue();
        if (leaf_size) *leaf_size++ = leaf.getSize();
      }
    } break;
    case 11:  // add occupied pts
    {
      //        mexPrintf("octree updateNode\n");
      if (mxGetM(prhs[2]) != 3)
        mexErrMsgTxt("octomapWrapper: pts must be 3-by-n");
      int n = mxGetN(prhs[2]);
      double* pts = mxGetPrSafe(prhs[2]);
      mxLogical* occupied = mxGetLogicals(prhs[3]);
      for (int i = 0; i < n; i++) {
        tree->updateNode(pts[3 * i], pts[3 * i + 1], pts[3 * i + 2],
                         occupied[i]);
      }
    } break;
    case 12:  // insert a scan of endpoints and sensor origin
    {
      // pointsA should be 3xN, originA is 3x1
      double* points = mxGetPrSafe(prhs[2]);
      double* originA = mxGetPrSafe(prhs[3]);
      int n = mxGetN(prhs[2]);
      point3d origin((float)originA[0], (float)originA[1], (float)originA[2]);
      Pointcloud pointCloud;
      for (int i = 0; i < n; i++) {
        point3d point((float)points[3 * i], (float)points[3 * i + 1],
                      (float)points[3 * i + 2]);
        pointCloud.push_back(point);
      }
      tree->insertPointCloud(pointCloud, origin);
    } break;
    case 21:  // save to file
    {
      char* filename = mxArrayToString(prhs[2]);
      //        mexPrintf("writing octree to %s\n", filename);
      tree->writeBinary(filename);
      mxFree(filename);
    } break;
    default:
      mexErrMsgTxt("octomapWrapper: Unknown command");
  }
}
