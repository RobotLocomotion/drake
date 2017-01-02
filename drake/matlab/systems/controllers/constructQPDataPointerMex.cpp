#include <memory>

#include "drake/examples/Atlas/atlasUtil.h"
#include "drake/examples/examples_package_map.h"
#include "drake/matlab/util/drakeMexUtil.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/parser_common.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/systems/controllers/InstantaneousQPController.h"

using namespace Eigen;

DLL_EXPORT_SYM
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  if (nrhs == 1) {
    // By convention, calling the constructor with just one argument (the
    // pointer) should delete the pointer
    if (isa(prhs[0], "DrakeMexPointer")) {
      destroyDrakeMexPointer<InstantaneousQPController *>(prhs[0]);
      return;
    } else {
      mexErrMsgIdAndTxt("Drake:constructQPDataPointerMex:BadInputs",
                        "Expected a DrakeMexPointer (or a subclass)");
    }
  }

  if (nrhs < 2)
    mexErrMsgTxt(
        "usage: ptr = constructQPDataPointerMex(urdf_filename, "
        "control_config_filename, urdf_modifications_filename="
        ");");

  if (nlhs < 1) mexErrMsgTxt("take at least one output... please.");

  int narg = 0;

  std::string urdf_filename = mxGetStdString(prhs[narg++]);
  std::cout << "urdf_filename: " << urdf_filename << std::endl;
  std::string control_config_filename = mxGetStdString(prhs[narg++]);
  std::cout << "control config: " << control_config_filename << std::endl;
  auto robot = std::make_unique<RigidBodyTree<double>>();
  drake::parsers::PackageMap package_map;
  drake::examples::AddExamplePackages(&package_map);
  drake::parsers::urdf::AddModelInstanceFromUrdfFileSearchingInRosPackages(
      urdf_filename, package_map, drake::multibody::joints::kRollPitchYaw,
      nullptr /* weld to frame */, robot.get());

  if (nrhs >= 3) {
    std::string urdf_modifications_filename = mxGetStdString(prhs[narg++]);
    std::cout << "urdf_modifications_filename: " << urdf_modifications_filename
              << std::endl;
    if (urdf_modifications_filename.size() > 0) {
      applyURDFModifications(robot, urdf_modifications_filename);
    }
  }
  InstantaneousQPController *controller =
      new InstantaneousQPController(std::move(robot), control_config_filename);
  plhs[0] =
      createDrakeMexPointer((void *)controller, "InstantaneousQPController");

  return;
}
