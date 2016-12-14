#include <mex.h>

#include <iostream>
#include <memory>

#include <Eigen/Dense>

#include "drake/examples/examples_package_map.h"
#include "drake/matlab/util/drakeMexUtil.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/parser_common.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"

using namespace std;

#define BUF_SIZE 256
#define FRAME_PARSER_EPSILON 1e-6

DLL_EXPORT_SYM
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  if (nrhs != 2) {
    mexErrMsgIdAndTxt("Drake:testFrameParsermex:NotEnoughInputs",
                      "Usage: testFrameParsermex(mex_model_ptr, urdf)");
  }

  char buf[BUF_SIZE];
  RigidBodyTree<double> *model =
      (RigidBodyTree<double> *)getDrakeMexPointer(prhs[0]);
  mxGetString(prhs[1], buf, BUF_SIZE);
  auto cpp_model = std::make_unique<RigidBodyTree<double>>();
  drake::parsers::PackageMap package_map;
  drake::examples::AddExamplePackages(&package_map);
  drake::parsers::urdf::AddModelInstanceFromUrdfFileSearchingInRosPackages(
      buf, package_map, drake::multibody::joints::kRollPitchYaw,
      nullptr /* weld to frame */, cpp_model.get());

  if (cpp_model->frames.size() != model->frames.size()) {
    mexErrMsgIdAndTxt("Drake:testFrameParsermex:FrameCountMismatch",
                      "The manipulator frame counts did not match");
  }

  for (size_t x = 0; x < model->frames.size(); x++) {
    double err = (model->frames[x]->get_transform_to_body().matrix() -
        cpp_model->frames[x]->get_transform_to_body().matrix()).norm();

    if (err > FRAME_PARSER_EPSILON) {
      mexErrMsgIdAndTxt(
          "Drake:testFrameParsermex:FrameTransformMismatch",
          "The homogeneous frame transformation matrix did not match");
    }

    if (model->frames[x]->get_name().compare(cpp_model->frames[x]->get_name())
        != 0) {
      mexErrMsgIdAndTxt("Drake:testFrameParsermex:FrameNameMismatch",
                        "The frame name did not match");
    }
  }
}
