#include <ctime>
#include <map>
#include <random>
#include <sstream>
#include <string>

#include <gflags/gflags.h>

#include "drake/common/eigen_types.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/manipulation/scene_generation/random_clutter_generator.h"
#include "drake/manipulation/scene_generation/simulate_plant_to_rest.h"
#include "drake/manipulation/util/simple_tree_visualizer.h"
#include "drake/manipulation/util/world_sim_tree_builder.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace manipulation {
namespace scene_generation {
namespace {
using std::string;
using std::stringstream;
using std::map;

DEFINE_int32(repetitions, 1, "each of 4 models is repeated this many times.");
DEFINE_int32(max_iterations, 100, "Max iterations for this demo.");
DEFINE_bool(visualize_only_terminal_state, true,
            "turn on visualization of the "
            "demo only at terminal states : i.e. when ik is available and when "
            "the simulate "
            "to rest terminates.");
DEFINE_double(max_settling_time, 1.5,
              "maximum simulation time for settling the object.");
DEFINE_double(v_threshold, 0.1, "velocity threshold to terminate sim early.");
DEFINE_bool(fall_sim, false,
            "Compute a fall simulation to 'settle' the objects.");
DEFINE_double(z_height_cost, 0.0, "Add a cost on the z height of the objects");

const char kPath[] = "examples/kuka_iiwa_arm/models/objects/";

std::unique_ptr<RigidBodyTreed> GenerateSceneTree(
    std::set<int>* clutter_instances, int num_repetitions) {
  std::vector<string> target_names = {
      {string(kPath) + "block_for_pick_and_place.urdf"},
      {string(kPath) + "block_for_pick_and_place_large_size.urdf"},
      {string(kPath) + "big_robot_toy.urdf"},
      {string(kPath) + "block_for_pick_and_place_mid_size.urdf"}};

  std::unique_ptr<util::WorldSimTreeBuilder<double>> tree_builder =
      std::make_unique<util::WorldSimTreeBuilder<double>>();
  tree_builder->StoreModel("open_top_box", string(kPath) + "open_top_box.urdf");
  tree_builder->AddFixedModelInstance("open_top_box",
                                      Eigen::Vector3d(0, 0, 0.01));

  std::set<int> clutter_instance_list;
  int model_ctr = 0;
  for (auto& it : target_names) {
    stringstream model_name;
    model_name << "model_" << model_ctr++;
    tree_builder->StoreModel(model_name.str(), it);

    for (int i = 0; i < num_repetitions; ++i) {
      // All floating objects are added to origin. The floating base
      // coordinates define the object's pose relative to the world
      // frame.
      Isometry3<double> X_WM = Isometry3<double>::Identity();
      int tree_instances = tree_builder->AddFloatingModelInstance(
          model_name.str(), X_WM.translation());
      clutter_instance_list.insert(tree_instances);
    }
  }

  *clutter_instances = clutter_instance_list;

  tree_builder->AddGround();
  return tree_builder->Build();
}

int DoMain() {
  lcm::DrakeLcm lcm;
  std::set<int> clutter_instances;

  auto scene_tree = GenerateSceneTree(&clutter_instances, FLAGS_repetitions);

  VectorX<double> q_nominal =
      VectorX<double>::Random(scene_tree->get_num_positions());

  RandomClutterGenerator clutter = RandomClutterGenerator(
      scene_tree.get(), clutter_instances, Vector3<double>(0.0, 0.0, 0.3),
      Vector3<double>(0.2, 0.4, 0.3 * FLAGS_repetitions));

  auto scene_plant =
      std::make_unique<systems::RigidBodyPlant<double>>(std::move(scene_tree));

  std::unique_ptr<SimpleTreeVisualizer> simple_tree_visualizer;

  std::unique_ptr<SimulatePlantToRest> dropper;

  if (FLAGS_visualize_only_terminal_state) {
    simple_tree_visualizer = std::make_unique<SimpleTreeVisualizer>(
        scene_plant->get_rigid_body_tree(), &lcm);
  }

  if (FLAGS_fall_sim) {
    if (!FLAGS_visualize_only_terminal_state) {
      auto drake_visualizer = std::make_unique<systems::DrakeVisualizer>(
          scene_plant->get_rigid_body_tree(), &lcm);
      dropper = std::make_unique<SimulatePlantToRest>(
          std::move(scene_plant), std::move(drake_visualizer));
    } else {
      dropper = std::make_unique<SimulatePlantToRest>(std::move(scene_plant));
    }
  }

  // Uses a arbitrary seed.
  std::default_random_engine generator(123);

  VectorX<double> q_final, v_final;
  double sum_ik_time = 0, sum_total_time = 0;
  for (int i = 0; i < FLAGS_max_iterations; ++i) {
    time_t t_start = time(0);
    VectorX<double> q_ik = clutter.GenerateFloatingClutter(
        q_nominal, &generator, FLAGS_z_height_cost);
    if (FLAGS_visualize_only_terminal_state) {
      simple_tree_visualizer->visualize(q_ik);
    }

    time_t t_end = time(0);
    double ik_time = std::difftime(t_end, t_start);

    sum_ik_time += ik_time;
    double mean_ik_time = sum_ik_time / (i + 1.0);
    if (FLAGS_fall_sim) {
      q_final = dropper->Run(q_ik, &v_final, FLAGS_v_threshold,
                             FLAGS_max_settling_time);
      if (FLAGS_visualize_only_terminal_state) {
        simple_tree_visualizer->visualize(q_final);
      }
    }
    t_end = time(0);

    double process_time = std::difftime(t_end, t_start);
    sum_total_time += process_time;
    double mean_total_time = sum_total_time / (i + 1.0);

    drake::log()->info("IK time : {} sec", ik_time);
    drake::log()->info("Computation time : {} sec", process_time);
    drake::log()->info("Trials : {}, Mean computation time : {}", i,
                       mean_total_time);
    drake::log()->info("Trials : {}, Mean ik time : {}", i, mean_ik_time);
  }

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
