#pragma once

#include <memory>

#include "drake/common/drake_copyable.h"
#include "drake/geometry/optimization/hpolyhedron.h"
#include "drake/planning/collision_checker.h"
#include "drake/planning/iris/iris_interface_options.h"

namespace drake {
namespace planning {

// SFINAE that the options subclass must derive from the IrisInterfaceOptions.
template <typename IrisInterfaceOptionsSubclass,
          typename = std::enable_if_t<std::is_base_of<
              IrisInterfaceOptions, IrisInterfaceOptionsSubclass>::value>>
class IrisInterface {
  /**
   * A class for implementing various Iris-type algorithms.
   */
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IrisInterface);
  virtual ~IrisInterface();

  geometry::optimization::HPolyhedron BuildSet(
      const IrisInterfaceOptionsSubclass& options) {
    geometry::optimization::HPolyhedron set =
        options.domain.value_or(geometry::optimization::HPolyhedron::MakeBox(
            checker_->plant().GetPositionLowerLimits(),
            checker_->plant().GetPositionUpperLimits()));
    DRAKE_THROW_UNLESS(set.IsBounded());

    DoSetup(options, &set);

    while (!options.termination_function(set)) {
      DoUpdateMetric(options, set);
      DoAddPlanesToSet(options, &set);
    }
    return set;
  };

 protected:
  explicit IrisInterface(const CollisionChecker& checker)
      : checker_{std::move(checker.Clone())} {};

  /** Runs any additional set up code before the set building loop starts. */
  virtual void DoSetup(const IrisInterfaceOptionsSubclass& options,
                       geometry::optimization::HPolyhedron* set) = 0;

  /** Given a proposed region, modify it to a better region, e.g. by adding
   * hyperplanes so that less of the set is in collision. */
  virtual void DoAddPlanesToSet(const IrisInterfaceOptionsSubclass& options,
                                geometry::optimization::HPolyhedron* set) = 0;

  /** Updates the metric used to find an improvement of the set */
  virtual void DoUpdateMetric(
      const IrisInterfaceOptionsSubclass& options,
      const geometry::optimization::HPolyhedron& set) = 0;

  std::unique_ptr<CollisionChecker> checker_;
};

}  // namespace planning
}  // namespace drake