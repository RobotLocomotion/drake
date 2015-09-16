#include "mex.h"
#include <iostream>
#include "drakeMexUtil.h"
#include "RigidBodyManipulator.h"
#include <Eigen/Dense>

using namespace std;

#define BUF_SIZE 256
#define FRAME_PARSER_EPSILON 1e-6

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) 
{
  if (nrhs != 2) {
    mexErrMsgIdAndTxt("Drake:testFrameParsermex:NotEnoughInputs","Usage: testFrameParsermex(mex_model_ptr, urdf)");
  }
  
  char buf[BUF_SIZE];
  RigidBodyManipulator *model= (RigidBodyManipulator*) getDrakeMexPointer(prhs[0]);  
  mxGetString(prhs[1] ,buf, BUF_SIZE);
  RigidBodyManipulator cpp_model(buf);

  if (cpp_model.frames.size() != model->frames.size()) {
  	mexErrMsgIdAndTxt("Drake:testFrameParsermex:FrameCountMismatch", "The manipulator frame counts did not match");
  }

  for (size_t x = 0; x < model->frames.size() ; x++) {
  	double err = (model->frames[x]->transform_to_body - cpp_model.frames[x]->transform_to_body).norm();
  	
  	if (err > FRAME_PARSER_EPSILON) {
  		mexErrMsgIdAndTxt("Drake:testFrameParsermex:FrameTransformMismatch", "The homogeneous frame transformation matrix did not match");		
  	}

  	if (model->frames[x]->name.compare(cpp_model.frames[x]->name) != 0 ) {
  		mexErrMsgIdAndTxt("Drake:testFrameParsermex:FrameNameMismatch", "The frame name did not match");		
  	}
  	
  }
}