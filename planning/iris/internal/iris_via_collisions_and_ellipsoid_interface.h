#pragma once

#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/planning/collision_checker.h"
#include "drake/planning/iris/internal/iris_via_collisions_and_ellipsoid_interface_options.h"
#include "drake/planning/iris/iris_interface.h"
#include "drake/planning/iris/iris_interface_options.h"

namespace drake {
namespace planning {
namespace internal {

// SFINAE that the options subclass must derive from the IrisInterfaceOptions.
template <typename IrisViaCollisionsAndEllipsoidInterfaceOptionsSubclass,
          typename = std::enable_if_t<std::is_base_of<
              IrisViaCollisionsAndEllipsoidInterfaceOptions,
              IrisViaCollisionsAndEllipsoidInterfaceOptionsSubclass>::value>>
class IrisViaCollisionsAndEllipsoidInterface
    : public IrisInterface<
          IrisViaCollisionsAndEllipsoidInterfaceOptionsSubclass> {
 public:
  virtual ~IrisViaCollisionsAndEllipsoidInterface();

 protected:
  explicit IrisViaCollisionsAndEllipsoidInterface(
      const CollisionChecker& checker);

  /** Each column of the returned matrix is a point in set which contains a
   * collision */
  virtual Eigen::MatrixXd FindCollisionPoints(
      const IrisViaCollisionsAndEllipsoidInterfaceOptions& options,
      const geometry::optimization::HPolyhedron& set) = 0;

  /** Given a set of points in collision that are inside sets, add hyperplanes
   * to set to exclude those collisions. */
  virtual void AddHyperplanesAtPoints(
      const Eigen::Ref<const Eigen::MatrixXd>& points,
      const IrisViaCollisionsAndEllipsoidInterfaceOptions& options,
      geometry::optimization::HPolyhedron* set) const = 0;

 private:
  void DoAddPlanesToSet(
      const IrisViaCollisionsAndEllipsoidInterfaceOptions& options,
      geometry::optimization::HPolyhedron* set);

  void DoUpdateMetric(
      const IrisViaCollisionsAndEllipsoidInterfaceOptions& options,
      const geometry::optimization::HPolyhedron& set);

  void DoSetup(const IrisViaCollisionsAndEllipsoidInterfaceOptions& options,
               geometry::optimization::HPolyhedron* set);
  bool HPolyhedronIsCollisionFreeViaUnadaptiveTest(
      const geometry::optimization::HPolyhedron& set,
      const IrisViaCollisionsAndEllipsoidInterfaceOptions& options) const;

  geometry::optimization::Hyperellipsoid ellipsoid_metric;
};

}  // namespace internal
}  // namespace planning
}  // namespace drake