#include "drake/planning/iris/c_iris.h"

namespace drake {
namespace planning {
using geometry::optimization::CspaceFreePolytope;

C_Iris::C_Iris(
    const SceneGraphCollisionChecker& checker,
    geometry::optimization::SeparatingPlaneOrder plane_order,
    const Eigen::Ref<const Eigen::VectorXd>& q_star,
    const geometry::optimization::CspaceFreePolytopeBase::Options& options)
    : IrisInterface<C_IrisOptions>(checker),
      set_builder_{&(checker_->plant()), &(checker_->model().scene_graph()),
                   plane_order, q_star, options} {};

void C_Iris::DoSetup(const C_IrisOptions& options,
                     geometry::optimization::HPolyhedron* set) {
  *set = options.seed_region;
  std::optional<CspaceFreePolytope::SearchResult> search_result =
      set_builder_.BinarySearch(options.ignored_collision_pairs, set->A(),
                                set->b(), set->ChebyshevCenter(),
                                options.binary_search_options);
  throw std::logic_error("unimplemented");
}

void C_Iris::DoImproveRegionHyperplanes(
    const C_IrisOptions& options, geometry::optimization::HPolyhedron* set) {
  // This should implement the step which pushes the hyperplanes away from the
  // max volume inscribed ellipse.
  unused(options, set);
  throw std::logic_error("unimplemented");
}

void C_Iris::DoUpdateMetric(const C_IrisOptions& options,
                            const geometry::optimization::HPolyhedron& set) {
  // This should compute the new inscribed ellipse and the new certificate after
  // the hyperplanes have been pushed back.
  unused(options, set);
  throw std::logic_error("unimplemented");
}

}  // namespace planning
}  // namespace drake
