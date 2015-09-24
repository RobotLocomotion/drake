#include "controlUtil.h"
#include "drakeMexUtil.h"

using namespace std;
using namespace Eigen;

struct SupportDetectData {
  RigidBodyManipulator* r;
  void* map_ptr;
};

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  if (nrhs<1) mexErrMsgTxt("usage: ptr = supportDetectmex(0,robot_obj,...); alpha=supportDetectmex(ptr,...,...)");
  if (nlhs<1) mexErrMsgTxt("take at least one output... please.");
  
  struct SupportDetectData* pdata;
//   mxArray* pm;
  double* pr;
  int i,j;

  if (mxGetScalar(prhs[0])==0) { // then construct the data object and return
    pdata = new struct SupportDetectData;
    
    // get robot mex model ptr
    if (!mxIsNumeric(prhs[1]) || mxGetNumberOfElements(prhs[1])!=1)
      mexErrMsgIdAndTxt("DRC:supportDetectmex:BadInputs","the second argument should be the robot mex ptr");
    memcpy(&(pdata->r),mxGetData(prhs[1]),sizeof(pdata->r));
        
     // get the map ptr back from matlab
     if (!mxIsNumeric(prhs[2]) || mxGetNumberOfElements(prhs[2])!=1)
     mexErrMsgIdAndTxt("DRC:supportDetectmex:BadInputs","the third argument should be the map ptr");
     memcpy(&(pdata->map_ptr),mxGetPr(prhs[2]),sizeof(pdata->map_ptr));
    
    if (!pdata->map_ptr)
      mexWarnMsgTxt("Map ptr is NULL.  Assuming flat terrain at z=0");
   
     
    mxClassID cid;
    if (sizeof(pdata)==4) cid = mxUINT32_CLASS;
    else if (sizeof(pdata)==8) cid = mxUINT64_CLASS;
    else mexErrMsgIdAndTxt("Drake:supportDetectmex:PointerSize","Are you on a 32-bit machine or 64-bit machine??");
     
    plhs[0] = mxCreateNumericMatrix(1,1,cid,mxREAL);
    memcpy(mxGetData(plhs[0]),&pdata,sizeof(pdata));
     
    return;
  }
  
  // first get the ptr back from matlab
  if (!mxIsNumeric(prhs[0]) || mxGetNumberOfElements(prhs[0])!=1)
    mexErrMsgIdAndTxt("DRC:supportDetectmex:BadInputs","the first argument should be the ptr");
  memcpy(&pdata,mxGetData(prhs[0]),sizeof(pdata));

  int nq = pdata->r->num_positions;
  int nv = pdata->r->num_velocities;

  int narg=1;  
  double *q = mxGetPr(prhs[narg++]);
  double *qd = &q[nq];
  
  Map<VectorXd> qvec(q,nq);
  Map<VectorXd> qdvec(qd, nv);

  int desired_support_argid = narg++;

  double* double_contact_sensor = mxGetPr(prhs[narg]); int len = static_cast<int>(mxGetNumberOfElements(prhs[narg++]));
  VectorXi contact_sensor(len);  
  for (i=0; i<len; i++)
    contact_sensor(i)=(int)double_contact_sensor[i];
  double contact_threshold = mxGetScalar(prhs[narg++]);
  
  int contact_logic_AND = (int) mxGetScalar(prhs[narg++]); // true if we should AND plan and sensor, false if we should OR them

  KinematicsCache<double> cache = pdata->r->doKinematics(qvec, qdvec); // FIXME: pass this into the function.

  //---------------------------------------------------------------------
  // Compute active support from desired supports -----------------------
  vector<SupportStateElement,Eigen::aligned_allocator<SupportStateElement>> active_supports;
  set<int> contact_bodies; // redundant, clean up later
  int num_active_contact_pts=0;
  if (!mxIsEmpty(prhs[desired_support_argid])) {
    VectorXd phi;
    mxArray* mxBodies;
    mxBodies = mxGetProperty(prhs[desired_support_argid],0,"bodies");
    if (!mxBodies) mxBodies = mxGetField(prhs[desired_support_argid],0,"bodies");
    if (!mxBodies) mexErrMsgTxt("couldn't get bodies");
    double* pBodies = mxGetPr(mxBodies);

    mxArray* mxContactPts;
    // We may have gotten a RigidBodySupportState object (in which case we need mxGetProperty...
    mxContactPts = mxGetProperty(prhs[desired_support_argid],0,"contact_pts");
    // ...or a struct array, in which case we need mxGetField
    if (!mxContactPts) mxContactPts = mxGetField(prhs[desired_support_argid],0,"contact_pts");
    if (!mxContactPts) mexErrMsgTxt("couldn't get contact points");
    
    for (i=0; i<mxGetNumberOfElements(mxBodies);i++) {
      mxArray* mxBodyContactPts = mxGetCell(mxContactPts,i);
      assert(mxGetM(mxBodyContactPts) == 3);
      int nc = static_cast<int>(mxGetN(mxBodyContactPts));
      if (nc<1) continue;
      
      Map<MatrixXd> all_body_contact_pts(mxGetPr(mxBodyContactPts), mxGetM(mxBodyContactPts), mxGetN(mxBodyContactPts));

      SupportStateElement se;
      se.body_idx = (int) pBodies[i]-1;
      for (j=0; j<nc; j++) {
        se.contact_pts.push_back(all_body_contact_pts.col(j));
      }
      
      if (contact_threshold == -1) { // ignore terrain
        if (contact_sensor(i)!=0) { // no sensor info, or sensor says yes contact
          active_supports.push_back(se);
          num_active_contact_pts += nc;
          contact_bodies.insert((int)se.body_idx); 
        }
      } 
      else {
        contactPhi(pdata->r, cache, se, phi);
        bool in_contact = true;
        if (contact_logic_AND) { // plan is true, now check contact sensor/kinematics
          in_contact =  (phi.minCoeff()<=contact_threshold || contact_sensor(i)==1); // any contact below threshold (kinematically) OR contact sensor says yes contact
        }
        // else: (plan) OR (sensor), so in_contact = true

        if (in_contact) { 
          active_supports.push_back(se);
          num_active_contact_pts += nc;
          contact_bodies.insert((int)se.body_idx);
        }
      }
    }
  }

  if (nlhs>0) {
    plhs[0] = mxCreateDoubleMatrix(1,static_cast<int>(active_supports.size()),mxREAL);
    pr = mxGetPr(plhs[0]);
    int i=0;
    for (vector<SupportStateElement,Eigen::aligned_allocator<SupportStateElement>>::iterator iter = active_supports.begin(); iter!=active_supports.end(); iter++) {
      pr[i++] = (double) (iter->body_idx + 1);
    }
  }
} 
