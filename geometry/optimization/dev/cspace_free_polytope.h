#pragma once

#include <map>
#include <memory>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <fmt/format.h>

#include "drake/geometry/optimization/dev/collision_geometry.h"
#include "drake/geometry/optimization/dev/separating_plane.h"
#include "drake/multibody/rational/rational_forward_kinematics.h"

namespace drake {
namespace geometry {
namespace optimization {
/**
 Contains the information to enforce a pair of geometries are separated by a
 plane.
 The conditions are that certain rational functions should be always positive,
 and a certain vector has length <= 1.
 */
struct PlaneSeparatesGeometries {
  PlaneSeparatesGeometries(
      std::vector<symbolic::RationalFunction> m_rationals,
      std::vector<VectorX<symbolic::Polynomial>> m_unit_length_vectors,
      int m_plane_index)
      : rationals{std::move(m_rationals)},
        unit_length_vectors{std::move(m_unit_length_vectors)},
        plane_index{m_plane_index} {}
  const std::vector<symbolic::RationalFunction> rationals;
  const std::vector<VectorX<symbolic::Polynomial>> unit_length_vectors;
  int plane_index;
};

/**
 This class tries to find large convex region in the configuration space, such
 that this whole convex set is collision free.
 For more details, refer to the paper
 "Finding and Optimizing Certified, Colision-Free Regions in Configuration Space
 for Robot Manipulators" by Alexandre Amice, Hongkai Dai, Peter Werner, Annan
 Zhang and Russ Tedrake.
 */
class CspaceFreePolytope {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CspaceFreePolytope)

  using FilteredCollsionPairs =
      std::unordered_set<SortedPair<geometry::GeometryId>>;

  ~CspaceFreePolytope() {}

  /**
   @param plant The plant for which we compute the C-space free polytopes. It
   must outlive this CspaceFreePolytope object.
   @param scene_graph The scene graph that has been connected with `plant`. It
   must outlive this CspaceFreePolytope object.
   @param plane_order The order of the polynomials in the plane to separate a
   pair of collision geometries.
   */
  CspaceFreePolytope(const multibody::MultibodyPlant<double>* plant,
                     const geometry::SceneGraph<double>* scene_graph,
                     SeparatingPlaneOrder plane_order);

  [[nodiscard]] const multibody::RationalForwardKinematics&
  rational_forward_kin() const {
    return rational_forward_kin_;
  }

  /**
   separating_planes()[map_geometries_to_separating_planes.at(geometry1_id,
   geometry2_id)] is the separating plane that separates geometry1 and
   geometry 2.
   */
  [[nodiscard]] const std::unordered_map<SortedPair<geometry::GeometryId>, int>&
  map_geometries_to_separating_planes() const {
    return map_geometries_to_separating_planes_;
  }

  [[nodiscard]] const std::vector<SeparatingPlane<symbolic::Variable>>&
  separating_planes() const {
    return separating_planes_;
  }

  /**
   Generate all the conditions (certain rationals being non-negative, and
   certain vectors with length <= 1) such that the robot configuration is
   collision free.
   @param q_star Refer to RationalForwardKinematics for its meaning.
   @param filtered_collision_pairs
   @param separating_margin The distance between the separating plane to each
   geometry is at least this margin. If set to std::nullopt, then we only
   require separation, but don't require a separating margin.
   */
  [[nodiscard]] std::vector<PlaneSeparatesGeometries> GenerateRationals(
      const Eigen::Ref<const Eigen::VectorXd>& q_star,
      const CspaceFreePolytope::FilteredCollsionPairs& filtered_collision_pairs,
      const std::optional<symbolic::Variable>& separating_margin) const;

 private:
  multibody::RationalForwardKinematics rational_forward_kin_;
  const geometry::SceneGraph<double>& scene_graph_;
  std::map<multibody::BodyIndex,
           std::vector<std::unique_ptr<CollisionGeometry>>>
      link_geometries_;

  SeparatingPlaneOrder plane_order_;
  std::vector<SeparatingPlane<symbolic::Variable>> separating_planes_;
  std::unordered_map<SortedPair<geometry::GeometryId>, int>
      map_geometries_to_separating_planes_;
};

/**
 * Given a diagram (which contains the plant and the scene_graph), returns all
 * the collision geometries.
 */
[[nodiscard]] std::map<multibody::BodyIndex,
                       std::vector<std::unique_ptr<CollisionGeometry>>>
GetCollisionGeometries(const multibody::MultibodyPlant<double>& plant,
                       const geometry::SceneGraph<double>& scene_graph);

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
