#pragma once

#include <map>
#include <string>

#include "drake/common/eigen_types.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/manipulation/util/world_sim_tree_builder.h"
#include "drake/multibody/rigid_body_constraint.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace manipulation {
namespace scene_generation {
/* A utility method for populating a WorldSimTreeBuilder object with 
 * multiple floating instances of specified models.
 * @param model_map A std::map between model names and quantities of
 * repetitions of the objects to be rendered in the clutter.
 */
void PopulateWithFloatingObjectRepetitions(
  util::WorldSimTreeBuilder<double>* world_sim_tree_builder, 
  std::map<std::string, int> model_map);

/**
 * Given a RigidBodyTree containing a given scene with atleast one floating
 * object, the RandomClutterGenerator generates poses such that the specified 
 * floating element(s) are organised into a random cluttered scene (random 
 * configuration) within a specified bounding region. 
 * each of these objects are seperated from each other by (settable) minimum distance and their object frames are
 * located within a (settable) bounding box volume. 

 * The ClutterGenerator
 * initializes a random configuration of the specified set of objects and
 * uses InverseKinematics to find a feasible configuration that obeys
 * MinDistance, FixedOrientation, and WorldPosition constraints, and finally
 * simulates falling in order to generate the scene and return the final
 * poses.
 */
class RandomClutterGenerator {
 public:
  /**
   * Constructs the RandomClutterGenerator with bounding volume enabled.
   
   * @param bounding_box Specifications of the volume bounds on the
   * generated clutter.
   * @param lcm Pointer to the DrakeLcmInterface object.
   * @param visualize_steps Flag enabling visualization of the clutter
   * generation process.
   * @param min_inter_object_distance Minimum distance between objects.
   * @param X_WC Transform to convert the cluttered scene object poses
   * to a desired reference frame.
   */
  RandomClutterGenerator(
      const RigidBodyTreed& scene_tree, 
      Eigen::Isometry3<double> X_WB,
      Vector3<double> bounding_box_size, 
      std::vector<int> clutter_model_instances,
      double min_inter_object_distance = 0.001);
  /**
   * Generates the cluttered scene.
   * @return a ModelPosePair of the object model names and their poses.
   */
 VectorX<double> Generate(VectorX<double> q_in);

 private:
  // Generates a random pose within the bounds of the problem.
  Isometry3<double> GenerateRandomBoundedPose();

  // Generates a random bounded tree configuration.
  VectorX<double> GetRandomBoundedConfiguration(
      q_initial);

  // Simulates a drop of the objects to the ground.
  VectorX<double> DropObjectsToGround(
      std::unique_ptr<systems::RigidBodyPlant<double>> plant_ptr,
      const VectorX<double>& q_ik);

  const RigidBodyTreed scene_tree_;
  Isometry3<double> X_WB_{
    Isometry3<double>::Identity();}
  Vector3<double> bounding_box_size_{
    Vector3<double>::Zero();}
  std::vector<int> clutter_model_instances_;

  double inter_object_distance_{0.1};
  bool visualize_steps_{false};  
  std::vector<int> clutter_model_instances_;

  // Will be used to obtain a seed for the random number engine
  std::random_device random_device_{};
  // Standard mersenne_twister_engine seeded with rd()
  std::mt19937 generator_;
};

}  // namespace scene_generation
}  // namespace manipulation
}  // namespace drake