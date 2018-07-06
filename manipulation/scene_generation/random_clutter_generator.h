#pragma once

#include <random>
#include <set>
#include <string>

#include "drake/common/drake_optional.h"
#include "drake/common/eigen_types.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace manipulation {
namespace scene_generation {

/**
 * Given a RigidBodyTree containing a given scene the RandomClutterGenerator
 * can repeatedly generate bounded random poses/configurations on selected
 * model instances within the tree. Each of these objects are seperated from
 * each other by (settable) minimum distance and their object frames are
 * located within a (settable) bounding box volume. This class solves the
 * IK problem to find feasible poses on the clutter bodies
 *
 * NOTES:
 * 1. Current version only ensures bounded clutter for the case of all model
 * instances on the tree containing the QuaternionFloatingJoint.
 * 2. The current version has only been tested with SNOPT.
 * 3. The solvability of the problem is strongly dependent on the dimensions
 * of the clutter bounding volume as specified in clutter_size and the number
 * and geometry of the clutter model instances that are to be dealt with. There
 * is no explicit time-out on the execution and the GenerateFloatingClutter
 * will keep attempting to find a solution, if any.
 * 4. There are no explicit guarantees on the solvability.
 * 5. The underlying IK computations utilise the bullet collision library and
 * as such only process the convex-hull of the geometry. The resulting IK
 * solution will be subject to this simplification.
 */

class RandomClutterGenerator {
 public:
  /**
   * Constructs the RandomClutterGenerator
   * @param scene_tree A pointer to the tree containing the scene.
   * @param clutter_model_instances A set of model instance indices
   * corresponding to the bodies on the tree that should comprise the clutter
   * @param clutter_center Centroid of the clutter bounding box in
   * world coordinates.
   * @param clutter_size The size of the clutter bounding box along the
   * length, breadth, and height.
   * @param min_inter_object_distance Minimum distance between objects.
   */
  RandomClutterGenerator(RigidBodyTree<double>* scene_tree,
                         const std::set<int>& clutter_model_instances,
                         const Vector3<double>& clutter_center,
                         const Vector3<double>& clutter_size,
                         double min_inter_object_distance = 0.001);

  /**
   * Generates the "Floating" clutter scene by solving an IK problem.
   * @return a VectorX<double> containing a feasible configuration.
   * @param q_nominal : nominal configuration for the scene_tree. Poses of
   * the model_instances not specified in `clutter_model_instances' are set
   * to this value.
   * @param generator : used to pass a seed.
   * @param z_height_cost : An optional cost added to the optimization problem 
   * on the height (z) of each of the model intances. Set to either 0 or {} 
   * in order to not utilise any z cost. @pre z_height_cost must be 
   * non-negative, if specified.
   */
  VectorX<double> GenerateFloatingClutter(const VectorX<double>& q_nominal,
                                          std::default_random_engine *generator,
                                          optional<double> z_height_cost = {});

 private:
  RigidBodyTree<double>* scene_tree_ptr_{};
  std::set<int> clutter_model_instances_;

  Vector3<double> clutter_center_;
  Vector3<double> clutter_lb_;
  Vector3<double> clutter_ub_;

  double inter_object_distance_{0.1};
};

}  // namespace scene_generation
}  // namespace manipulation
}  // namespace drake
