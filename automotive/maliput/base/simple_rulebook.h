#pragma once

#include <memory>
#include <unordered_map>
#include <vector>

#include "drake/automotive/maliput/api/rules/regions.h"
#include "drake/automotive/maliput/api/rules/right_of_way_rule.h"
#include "drake/automotive/maliput/api/rules/road_rulebook.h"
#include "drake/automotive/maliput/api/rules/speed_limit_rule.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace maliput {

/// SimpleRulebook is a simple concrete implementation of the
/// api::rules::RoadRulebook abstract interface.
class SimpleRulebook : public api::rules::RoadRulebook {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SimpleRulebook);

  /// Constructs an empty SimpleRulebook (i.e., containing no rules).
  SimpleRulebook();

  ~SimpleRulebook() override;

  /// Removes all rules.
  void RemoveAll();

  /// Adds a new RightOfWayRule.
  ///
  /// @throws std::runtime_error if a rule with the same ID already exists
  /// in the SimpleRulebook.
  void AddRule(const api::rules::RightOfWayRule& rule);

  /// Removes the RightOfWayRule labelled by `id`.
  ///
  /// @throws std::runtime_error if no such rule exists.
  void RemoveRule(const api::rules::RightOfWayRule::Id& id);

  /// Adds a new SpeedLimitRule.
  ///
  /// @throws std::runtime_error if a rule with the same ID already exists
  /// in the SimpleRulebook.
  void AddRule(const api::rules::SpeedLimitRule& rule);

  /// Removes the SpeedLimitRule labelled by `id`.
  ///
  /// @throws std::runtime_error if no such rule exists.
  void RemoveRule(const api::rules::SpeedLimitRule::Id& id);

 private:
  api::rules::RoadRulebook::QueryResults DoFindRules(
      const std::vector<api::rules::LaneSRange>& ranges,
      double tolerance) const override;
  api::rules::RightOfWayRule DoGetRule(
      const api::rules::RightOfWayRule::Id& id) const override;
  api::rules::SpeedLimitRule DoGetRule(
      const api::rules::SpeedLimitRule::Id& id) const override;

  template <class T>
  void AddAnyRule(const T& rule,
                  std::unordered_map<typename T::Id, T>* map);
  template <class T>
  T GetAnyRule(const typename T::Id& id,
               const std::unordered_map<typename T::Id, T>& map) const;
  template <class T>
  void RemoveAnyRule(const typename T::Id& id,
                     std::unordered_map<typename T::Id, T>* map);

  // ID->Rule indices for each rule type
  template <class T>
  using IdIndex = std::unordered_map<typename T::Id, T>;
  IdIndex<api::rules::RightOfWayRule> right_of_ways_;
  IdIndex<api::rules::SpeedLimitRule> speed_limits_;

  // An index from LaneSRange to collections of rule ID's of all types.
  class RangeIndex;
  std::unique_ptr<RangeIndex> index_;
};

}  // namespace maliput
}  // namespace drake
