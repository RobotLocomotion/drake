#pragma once

#include "drake/geometry/optimization/cspace_free_polytope.h"
#include "drake/geometry/optimization/cspace_free_polytope_base.h"
#include "drake/geometry/optimization/cspace_free_structs.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/planning/iris/iris_interface.h"
#include "drake/planning/iris/iris_interface_options.h"
#include "drake/planning/scene_graph_collision_checker.h"

namespace drake {
namespace planning {

struct C_IrisOptions : IrisInterfaceOptions {
  // TODO add all the unique options.

  // Options for running binary search during DoSetup which expands the seed
  // polytope.
  geometry::optimization::CspaceFreePolytope::BinarySearchOptions
      binary_search_options;

  // Options for running binary search during DoSetup which expands the seed
  // polytope.
  geometry::optimization::CspaceFreePolytope::BilinearAlternationOptions
      bilinearSearchOptions;

  geometry::optimization::HPolyhedron seed_region;

  geometry::optimization::CspaceFreePolytopeBase::IgnoredCollisionPairs
      ignored_collision_pairs;
};

class C_Iris final : public IrisInterface<C_IrisOptions> {
 public:
  C_Iris(
      const SceneGraphCollisionChecker& checker,
      geometry::optimization::SeparatingPlaneOrder plane_order,
      const Eigen::Ref<const Eigen::VectorXd>& q_star,
      const geometry::optimization::CspaceFreePolytopeBase::Options& options =
          geometry::optimization::CspaceFreePolytopeBase::Options{});

 private:
  // Do not hide IrisInterface's pure virtual functions.
  using IrisInterface<C_IrisOptions>::DoImproveRegionHyperplanes;
  using IrisInterface<C_IrisOptions>::DoUpdateMetric;
  using IrisInterface<C_IrisOptions>::DoSetup;

  /** Runs Binary Search on the seed polytope. */
  void DoSetup(const C_IrisOptions& options,
               geometry::optimization::HPolyhedron* set);

  /** Runs the step which pushes the hyperplanes away from the max volume
   * inscribed ellipse. */
  void DoImproveRegionHyperplanes(const C_IrisOptions& options,
                                  geometry::optimization::HPolyhedron* set);

  /** Runs the step which computes the max volume inscribed ellipse and a new
   * certificate. */
  void DoUpdateMetric(const C_IrisOptions& options,
                      const geometry::optimization::HPolyhedron& set);

  // Currently we use the set_builder_ to implement the above methods, but a
  // better rewrite will condense this into one file.
  geometry::optimization::CspaceFreePolytope set_builder_;
};

}  // namespace planning
}  // namespace drake
