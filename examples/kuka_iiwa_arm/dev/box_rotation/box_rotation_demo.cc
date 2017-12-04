/**
 * @file This file implements the box rotation demo.
 */

#include <fstream>
#include <iostream>
#include <list>
#include <memory>
#include <string>

#include <gflags/gflags.h>
#include <lcm/lcm-cpp.hpp>

#include "drake/common/find_resource.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/multibody/parsers/urdf_parser.h"

DEFINE_string(keyframes, "", "Name of keyframe file to load");
DEFINE_string(urdf, "", "Name of keyframe file to load");

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace box_rotation {
namespace {

const char* const kKeyFramePath = "drake/examples/kuka_iiwa_arm/dev/"
    "box_rotation/robot_keyframes.txt";
const char* const kIiwaUrdf =
    "drake/examples/kuka_iiwa_arm/dev/box_rotation/"
        "models/dual_iiwa14_primitive_sphere_visual_collision.urdf";

const int kNumKeyFrames = 26;

MatrixX<double> get_posture(const std::string& name) {
  std::fstream fs;
  fs.open(name, std::fstream::in);
  DRAKE_DEMAND(fs.is_open());

  MatrixX<double> ret(kNumKeyFrames, 21);
  for (int i = 0; i < ret.rows(); ++i) {
    for (int j = 0; j < ret.cols(); ++j) {
      fs >> ret(i, j);
    }
  }
  return ret;
}

void RunBoxRotationDemo() {
  lcm::LCM lcm;

  const std::string iiwa_path =
      (!FLAGS_urdf.empty() ? FLAGS_urdf : FindResourceOrThrow(kIiwaUrdf));

  // create the RBT
  auto tree = std::make_unique<RigidBodyTree<double>>();

  parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      iiwa_path, multibody::joints::kFixed, tree.get());

  // create a reference to the RBT
  const RigidBodyTree<double>& iiwa = *(tree.get());

  const std::string framesFile = (!FLAGS_keyframes.empty() ? FLAGS_keyframes :
                               FindResourceOrThrow(kKeyFramePath));

  MatrixX<double> allKeyFrames = get_posture(framesFile);

  // extract left and right arm keyframes
  MatrixX<double>  keyframes(kNumKeyFrames, 14);
  keyframes.block<kNumKeyFrames, 7>(0, 0) =
      allKeyFrames.block<kNumKeyFrames, 7>(0, 1);
  keyframes.block<kNumKeyFrames, 7>(0, 7) =
      allKeyFrames.block<kNumKeyFrames, 7>(0, 8);
  keyframes.transposeInPlace();

  const int N = static_cast<int>(allKeyFrames.rows());
  std::vector<double> times(static_cast<size_t>(N));
  for (int i = 0; i < N; ++i) {
    if (i == 0)
      times[i] = allKeyFrames(i, 0);
    else
      times[i] = times[i - 1] + allKeyFrames(i, 0);
  }

  std::vector<int> info(times.size(), 1);
  robotlocomotion::robot_plan_t plan{};

  plan = EncodeKeyFrames(iiwa, times, info, keyframes);
  lcm.publish("COMMITTED_ROBOT_PLAN", &plan);
}

}  // namespace
}  // namespace box_rotation
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::examples::kuka_iiwa_arm::box_rotation::RunBoxRotationDemo();
  return 0;
}
