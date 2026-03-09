#pragma once

#include <memory>
#include <unordered_map>
#include <vector>

#include "drake/geometry/optimization/dev/cspace_free_path.h"

namespace drake {
namespace geometry {
namespace optimization {
// This is a friend class of CspaceFreePath, we use it to expose the private
// functions in CspaceFreePath for unit testing.
class CspaceFreePathTester {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CspaceFreePathTester);

  CspaceFreePathTester(const multibody::MultibodyPlant<double>* plant,
                       const geometry::SceneGraph<double>* scene_graph,
                       const Eigen::Ref<const Eigen::VectorXd>& q_star,
                       int maximum_path_degree, int plane_order)
      : cspace_free_path_{new CspaceFreePath(
            plant, scene_graph, q_star, maximum_path_degree, plane_order)} {}

  [[nodiscard]] const CspaceFreePath& cspace_free_path() const {
    return *cspace_free_path_;
  }

  [[nodiscard]] const symbolic::Variable& get_mu() const {
    return cspace_free_path_->mu();
  }

  [[nodiscard]] const std::unordered_map<symbolic::Variable,
                                         symbolic::Polynomial>&
  get_path() const {
    return cspace_free_path_->path_;
  }

  [[nodiscard]] const std::vector<PlaneSeparatesGeometriesOnPath>&
  get_path_plane_geometries() const {
    return cspace_free_path_->plane_geometries_on_path_;
  }

  [[nodiscard]] const std::vector<CSpaceSeparatingPlane<symbolic::Variable>>&
  get_separating_planes() const {
    return cspace_free_path_->separating_planes_;
  }

  [[nodiscard]] const geometry::SceneGraph<double>& get_scene_graph() const {
    return cspace_free_path_->scene_graph_;
  }

  [[nodiscard]] const multibody::RationalForwardKinematics*
  get_rational_forward_kin() const {
    return &(cspace_free_path_->rational_forward_kin_);
  }

 private:
  std::unique_ptr<CspaceFreePath> cspace_free_path_;
};

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
