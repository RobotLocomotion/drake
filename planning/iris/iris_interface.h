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
   * A class for implementing various Iris-type algorithms. Note that this
   * interface is NOT thread-safe in the sense that one IrisInterface object
   * cannot be used to construct multiple regions in parallel.
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

    Setup(options, &set);

    int iteration = 0;
    while (iteration < options.iteration_limit && !CheckTermination(set)) {
      ImproveRegionHyperplanes(options, &set);
      UpdateMetric(options, set);
      ++iteration;
    }
    return set;
  };

  void Setup(const IrisInterfaceOptionsSubclass& options,
             geometry::optimization::HPolyhedron* set) {
    DRAKE_THROW_UNLESS(set != nullptr);
    DoSetup(options, set);
  }

  void ImproveRegionHyperplanes(const IrisInterfaceOptionsSubclass& options,
                                geometry::optimization::HPolyhedron* set) {
    DRAKE_THROW_UNLESS(set != nullptr);
    DoImproveRegionHyperplanes(options, set);
  }

  void UpdateMetric(const IrisInterfaceOptionsSubclass& options,
                    const geometry::optimization::HPolyhedron& set) {
    DoUpdateMetric(options, set);
  }

  bool CheckTermination(const IrisInterfaceOptionsSubclass& options,
                        const geometry::optimization::HPolyhedron& set) {
    bool ret{false};
    if (options.termination_func) {
      ret = options.termination_func(set);
    }
    return ret || DoCheckTermination(options, set);
  }

 protected:
  IrisInterface(const CollisionChecker& checker)
      : checker_{std::move(checker.Clone())} {};

  /** Runs any additional set up code before the set building loop starts. */
  virtual void DoSetup(const IrisInterfaceOptionsSubclass& options,
                       geometry::optimization::HPolyhedron* set) = 0;

  /** Given a proposed region, modify it to a better region, e.g. by adding
   * hyperplanes so that less of the set is in collision. */
  virtual void DoImproveRegionHyperplanes(
      const IrisInterfaceOptionsSubclass& options,
      geometry::optimization::HPolyhedron* set) = 0;

  /** Updates the metric used to find an improvement of the set */
  virtual void DoUpdateMetric(
      const IrisInterfaceOptionsSubclass& options,
      const geometry::optimization::HPolyhedron& set) = 0;

  /** Returns true if the set construct loop should terminate. */
  virtual bool DoCheckTermination(
      const IrisInterfaceOptionsSubclass& options,
      const geometry::optimization::HPolyhedron& set) = 0;

  std::unique_ptr<CollisionChecker> checker_;
};

}  // namespace planning
}  // namespace drake