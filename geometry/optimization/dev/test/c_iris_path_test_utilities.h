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
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CspaceFreePathTester)

  CspaceFreePathTester(const multibody::MultibodyPlant<double>* plant,
                       const geometry::SceneGraph<double>* scene_graph,
                       SeparatingPlaneOrder plane_order,
                       const Eigen::Ref<const Eigen::VectorXd>& q_star,
                       unsigned int maximum_path_degree,
                       const CspaceFreePolytope::Options& options =
                           CspaceFreePolytope::Options())
      : cspace_free_path_{new CspaceFreePath(plant, scene_graph, plane_order,
                                             q_star, maximum_path_degree,
                                             options)} {}

  [[nodiscard]] const CspaceFreePath& cspace_free_path() const {
    return *cspace_free_path_;
  }

  [[nodiscard]] const symbolic::Variable& get_mu() const {
    return cspace_free_path_->mu_;
  }

  [[nodiscard]] const std::unordered_map<
      symbolic::Variable, symbolic::Polynomial>&
      get_path() const {
    return cspace_free_path_->path_;
  }

  [[nodiscard]] const symbolic::Variables& get_s_set() const {
    return cspace_free_path_->get_s_set();
  }

  [[nodiscard]] std::vector<PlaneSeparatesGeometries>&
  get_mutable_plane_geometries() {
    return cspace_free_path_->get_mutable_plane_geometries();
  }

 private:
  std::unique_ptr<CspaceFreePath> cspace_free_path_;
};

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
