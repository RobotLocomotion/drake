#include <map>
#include <string>
#include <gflags/gflags.h>

#include "drake/common/eigen_types.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/manipulation/scene_generation/random_clutter_generator.h"
#include "drake/manipulation/util/simple_tree_visualizer.h"
#include "drake/multibody/rigid_body_tree.h"

#include <ctime>

namespace drake {
namespace manipulation {
namespace scene_generation {
namespace {
using std::string;
using std::map;

const string kPath = "examples/kuka_iiwa_arm/models/objects/";
int DoMain() {
  // time_t tstart, tend;

  // map<string, int> target_names = {{kPath + "block_for_pick_and_place.urdf", 2},
  //                                  {kPath + "block_for_pick_and_place_large_size.urdf",2},
  //                                  {kPath + "big_robot_toy.urdf", 2},
  //                                  {kPath + "block_for_pick_and_place_mid_size.urdf", 2}};

  // lcm::DrakeLcm lcm;
  // Isometry3<double> clutter_base = Isometry3<double>::Identity();
  // clutter_base.translation() = (VectorX<double>(3)<< 0.0, 0.0, 3.0).finished();

  // RandomClutterGenerator bounded_clutter(
  //     target_names, (VectorX<double>(3) << 0.75, 0.5, 0.3).finished(),
  //     clutter_base, true /* with_ik */,
  //     0.01 /* min_inter_object_distance */, &lcm);

  // for (int i = 0; i < 15; ++i) {
  //   tstart = time(0);
  //   bounded_clutter.Generate();
  //   tend = time(0);

  //   drake::log()->info("---------------------------------------");
  //   drake::log()->info("\tIt took {} sec", std::difftime(tend, tstart));
  //   drake::log()->info("---------------------------------------");
  // }

  // drake::log()->info("Bounded Clutter generated\n");
//
//  RandomClutterGenerator bounded_clutter_no_ik(
//          target_names, (VectorX<double>(3) << 0.75, 0.5, 0.45).finished(),
//          Isometry3<double>::Identity(), false /* with_ik */,
//          0.001 /* min_inter_object_distance */, &lcm);
//
//  for (int i = 0; i < 5; ++i) {
//    tstart = time(0);
//    bounded_clutter_no_ik.Generate();
//    tend = time(0);
//
//    drake::log()->info("---------------------------------------");
//    drake::log()->info("\tIt took {} sec", std::difftime(tend, tstart));
//    drake::log()->info("---------------------------------------");
//  }

//  drake::log()->info("Bounded Clutter with no ik generated\n");
//
//  RandomClutterGenerator unbounded_clutter(
//      target_names, Isometry3<double>::Identity(), true /* with_ik */,
//      0.001 /* min_inter_object_distance */, &lcm);
//
//  for (int i = 0; i < 5; ++i) {
//    tstart = time(0);
//    unbounded_clutter.Generate();
//    tend = time(0);
//
//    drake::log()->info("---------------------------------------");
//    drake::log()->info("\tIt took {} sec", std::difftime(tend, tstart));
//    drake::log()->info("---------------------------------------");
//  }
//
//  drake::log()->info("UnBounded Clutter generated\n");

  return 0;
}

}  // namespace
}  // namespace scene_generation
}  // namespace manipulation
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();
  drake::manipulation::scene_generation::DoMain();
}