#pragma once

#include <memory>

#include "drake/automotive/maliput/api/intersection.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/api/rules/regions.h"
#include "drake/automotive/maliput/api/rules/right_of_way_phase_book.h"
#include "drake/automotive/maliput/api/rules/right_of_way_phase_provider.h"
#include "drake/automotive/maliput/api/rules/right_of_way_rule.h"
#include "drake/automotive/maliput/api/rules/right_of_way_state_provider.h"
#include "drake/automotive/maliput/api/rules/road_rulebook.h"

namespace drake {
namespace maliput {
namespace api {

/// Provides various factory methods for creating mock Maliput objects.
class Mock {
 public:
  /// Returns a rules::LaneSRoute containing an arbitrary route.
  static rules::LaneSRoute LaneSRoute();

  /// Returns a rules::RightOfWayRule::State::YieldGroup of size two.
  static rules::RightOfWayRule::State::YieldGroup YieldGroup2();

  /// Returns a rules::RightOfWayRule::State containing no yield groups.
  static rules::RightOfWayRule::State NoYieldState();

  /// Returns a rules::RightOfWayRule::State containing yield groups.
  static rules::RightOfWayRule::State YieldState();

  /// Returns a rules::RightOfWayRule containing arbitrary state.
  static rules::RightOfWayRule Rule();

  /// Returns an arbitrary RoadGeometry.
  static std::unique_ptr<RoadGeometry> CreateRoadGeometry();

  /// Returns an arbitrary rules::RoadRulebook.
  static std::unique_ptr<rules::RoadRulebook> CreateRoadRulebook();

  /// Returns an arbitrary rules::RightOfWayPhaseBook.
  static std::unique_ptr<rules::RightOfWayPhaseBook>
  CreateRightOfWayPhaseBook();

  /// Returns an arbitrary rules::RightOfWayStateProvider.
  static std::unique_ptr<rules::RightOfWayStateProvider>
  CreateRightOfWayStateProvider();

  /// Returns an arbitrary rules::RightOfWayPhaseProvider.
  static std::unique_ptr<rules::RightOfWayPhaseProvider>
  CreateRightOfWayPhaseProvider();

  /// Returns an arbitrary Intersection.
  static std::unique_ptr<Intersection> CreateIntersection(
      const Intersection::Id& id);

  static rules::RightOfWayPhaseRing ring_;
};

}  // namespace api
}  // namespace maliput
}  // namespace drake
