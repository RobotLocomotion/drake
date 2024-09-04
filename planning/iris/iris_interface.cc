#include "drake/planning/iris/iris_interface.h"

#include "drake/planning/collision_checker.h"

namespace drake {
namespace planning {
// using geometry::optimization::HPolyhedron;
//
//// template <typename IrisInterfaceOptionsSubclass>
// IrisInterface::IrisInterface(const CollisionChecker& checker)
//    : checker_{std::move(checker.Clone())} {};
//
//// template <typename IrisInterfaceOptionsSubclass>
//// IrisInterface<IrisInterfaceOptionsSubclass>::IrisInterface(
////    const CollisionChecker& checker)
////    : checker_{std::move(checker.Clone())} {};
//
// template <typename IrisInterfaceOptionsSubclass>
// HPolyhedron IrisInterface::BuildSet(
//    const IrisInterfaceOptionsSubclass& options) const {
//  HPolyhedron set = options.domain.value_or(
//      HPolyhedron::MakeBox(checker_->plant().GetPositionLowerLimits(),
//                           checker_->plant().GetPositionUpperLimits()));
//  DRAKE_THROW_UNLESS(set.IsBounded());
//
//  DoSetup(options, &set);
//
//  while (!options.termination_function(set)) {
//    DoUpdateMetric(options, set);
//    DoAddPlanesToSet(options, &set);
//  }
//  return set;
//}

}  // namespace planning
}  // namespace drake
