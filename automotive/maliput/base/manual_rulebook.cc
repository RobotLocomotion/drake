#include "drake/automotive/maliput/base/manual_rulebook.h"

#include <algorithm>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include "drake/common/drake_optional.h"
#include "drake/common/hash.h"

namespace drake {
namespace maliput {

using api::LaneId;
using api::rules::DirectionUsageRule;
using api::rules::LaneSRange;
using api::rules::RightOfWayRule;
using api::rules::SpeedLimitRule;
using api::rules::SRange;

using QueryResults = api::rules::RoadRulebook::QueryResults;

namespace {

// TODO(maddog@tri.global)  When std::variant becomes available, use it like so:
// using IdVariant = std::variant<RightOfWayRule::Id,
//                                SpeedLimitRule::Id>;
struct IdVariant {
  drake::optional<RightOfWayRule::Id> r;
  drake::optional<SpeedLimitRule::Id> s;
  drake::optional<DirectionUsageRule::Id> d;

  // NOLINTNEXTLINE(runtime/explicit)
  IdVariant(const RightOfWayRule::Id& r_in) : r(r_in) {}

  // NOLINTNEXTLINE(runtime/explicit)
  IdVariant(const SpeedLimitRule::Id& s_in) : s(s_in) {}

  // NOLINTNEXTLINE(runtime/explicit)
  IdVariant(const DirectionUsageRule::Id& d_in) : d(d_in) {}

  template <class HashAlgorithm>
  friend void hash_append(HashAlgorithm& hasher,
                          const IdVariant& item) noexcept {
    using drake::hash_append;
    hash_append(hasher, item.r);
    hash_append(hasher, item.s);
    hash_append(hasher, item.d);
  }
};

bool operator==(const IdVariant& lhs, const IdVariant& rhs) {
  return (lhs.r == rhs.r) && (lhs.s == rhs.s) && (lhs.d == rhs.d);
}

}  // namespace

class ManualRulebook::Impl {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Impl)

  Impl() : index_(std::make_unique<RangeIndex>()) {}
  ~Impl() {}

  void RemoveAll() {
    right_of_ways_.clear();
    speed_limits_.clear();
    direction_usage_rules_.clear();
    index_->RemoveAll();
  }

  void AddRule(const api::rules::RightOfWayRule& rule) {
    AddAnyRule(rule, &right_of_ways_);
  }

  void RemoveRule(const api::rules::RightOfWayRule::Id& id) {
    RemoveAnyRule(id, &right_of_ways_);
  }

  void AddRule(const api::rules::SpeedLimitRule& rule) {
    AddAnyRule(rule, &speed_limits_);
  }

  void RemoveRule(const api::rules::SpeedLimitRule::Id& id) {
    RemoveAnyRule(id, &speed_limits_);
  }

  void AddRule(const api::rules::DirectionUsageRule& rule) {
    AddAnyRule(rule, &direction_usage_rules_);
  }

  void RemoveRule(const api::rules::DirectionUsageRule::Id& id) {
    RemoveAnyRule(id, &direction_usage_rules_);
  }

  QueryResults DoFindRules(const std::vector<LaneSRange>& ranges,
                           double tolerance) const {
    QueryResults result;
    for (const LaneSRange& range : ranges) {
      for (const IdVariant& id : index_->FindRules(range, tolerance)) {
        if (id.r) {
          result.right_of_way.push_back(right_of_ways_.at(*id.r));
        } else if (id.s) {
          result.speed_limit.push_back(speed_limits_.at(*id.s));
        } else if (id.d) {
          result.direction_usage.push_back(direction_usage_rules_.at(*id.d));
        } else {
          std::stringstream s;
          s << "ManualRulebook: IdVariant is empty (LaneId: "
            << range.lane_id().string() << ", s0: " << range.s_range().s0()
            << ", s1: " << range.s_range().s1() << ")";
          throw std::domain_error(s.str());
        }
      }
    }
    return result;
  }

  RightOfWayRule DoGetRule(const RightOfWayRule::Id& id) const {
    return GetAnyRule(id, right_of_ways_);
  }

  SpeedLimitRule DoGetRule(const SpeedLimitRule::Id& id) const {
    return GetAnyRule(id, speed_limits_);
  }

  DirectionUsageRule DoGetRule(const DirectionUsageRule::Id& id) const {
    return GetAnyRule(id, direction_usage_rules_);
  }

 private:
  // An index from LaneSRange to collections of rule ID's of all types.
  // RangeIndex indexes rules (by ID) on the LaneSRanges which they affect,
  // facilitating the lookup of rules by LaneSRange.
  class RangeIndex {
   public:
    void RemoveAll() { map_.clear(); }

    void AddRule(const SpeedLimitRule& rule) {
      AddRange(rule.id(), rule.zone());
    }

    void RemoveRule(const SpeedLimitRule& rule) {
      RemoveRanges(rule.id(), rule.zone().lane_id());
    }

    void AddRule(const RightOfWayRule& rule) {
      for (const LaneSRange& range : rule.zone().ranges()) {
        AddRange(rule.id(), range);
      }
    }

    void RemoveRule(const RightOfWayRule& rule) {
      for (const LaneSRange& range : rule.zone().ranges()) {
        RemoveRanges(rule.id(), range.lane_id());
      }
    }

    void AddRule(const DirectionUsageRule& rule) {
      AddRange(rule.id(), rule.zone());
    }

    void RemoveRule(const DirectionUsageRule& rule) {
      RemoveRanges(rule.id(), rule.zone().lane_id());
    }

    std::vector<IdVariant> FindRules(const LaneSRange& range,
                                     double tolerance) {
      std::vector<IdVariant> result;
      auto it = map_.find(range.lane_id());
      if (it != map_.end()) {
        for (const auto& id_and_range : it->second) {
          if (Intersects(id_and_range.second, range.s_range(), tolerance)) {
            result.emplace_back(id_and_range.first);
          }
        }
      }
      return result;
    }

   private:
    // Add a single (ID, LaneSRange) association.
    template <typename T>
    void AddRange(const T& id, const LaneSRange& range) {
      map_[range.lane_id()].emplace(id, range.s_range());
    }

    // Removes all associations involving `id` and `lane_id`.
    template <typename T>
    void RemoveRanges(const T& id, const LaneId& lane_id) {
      map_.at(lane_id).erase(id);
      if (map_[lane_id].empty()) {
        map_.erase(lane_id);
      }
    }

    // Determines if two SRanges intersect.  Positive tolerance makes the
    // comparison more optimistic.
    static bool Intersects(const SRange& lhs, const SRange& rhs,
                           double tolerance) {
      const SRange wider_rhs(std::min(rhs.s0(), rhs.s1()) - tolerance,
                             std::max(rhs.s0(), rhs.s1()) + tolerance);
      return !((std::max(lhs.s0(), lhs.s1()) < wider_rhs.s0()) ||
               (std::min(lhs.s0(), lhs.s1()) > wider_rhs.s1()));
    }

    // TODO(maddog@tri.global)  Perhaps this class would benefit from something
    //                          like boost::interval_map, though if there are
    //                          not many rules attached to each individual
    //                          lane_id, this is probably good enough.
    std::unordered_map<
        LaneId, std::unordered_multimap<IdVariant, SRange, drake::DefaultHash>>
        map_;
  };

  // ID->Rule indices for each rule type.
  template <class T>
  using IdIndex = std::unordered_map<typename T::Id, T>;

  template <class T>
  void AddAnyRule(const T& rule, IdIndex<T>* map) {
    // Add to map.
    auto map_result = map->emplace(rule.id(), rule);
    // Throw if the id was already present.
    DRAKE_THROW_UNLESS(map_result.second);
    // Add to index.
    index_->AddRule(rule);
  }

  template <class T>
  T GetAnyRule(const typename T::Id& id, const IdIndex<T>& map) const {
    return map.at(id);
  }

  template <class T>
  void RemoveAnyRule(const typename T::Id& id, IdIndex<T>* map) {
    DRAKE_THROW_UNLESS(map->count(id) == 1);
    // Remove from index.
    index_->RemoveRule(map->at(id));
    // Remove from map.
    auto map_result = map->erase(id);
    DRAKE_THROW_UNLESS(map_result > 0);
  }

  std::unique_ptr<RangeIndex> index_;
  IdIndex<api::rules::RightOfWayRule> right_of_ways_;
  IdIndex<api::rules::SpeedLimitRule> speed_limits_;
  IdIndex<api::rules::DirectionUsageRule> direction_usage_rules_;
};

ManualRulebook::ManualRulebook() : impl_(std::make_unique<Impl>()) {}

ManualRulebook::~ManualRulebook() = default;

void ManualRulebook::RemoveAll() { impl_->RemoveAll(); }

void ManualRulebook::AddRule(const api::rules::RightOfWayRule& rule) {
  impl_->AddRule(rule);
}

void ManualRulebook::RemoveRule(const api::rules::RightOfWayRule::Id& id) {
  impl_->RemoveRule(id);
}

void ManualRulebook::AddRule(const api::rules::SpeedLimitRule& rule) {
  impl_->AddRule(rule);
}

void ManualRulebook::RemoveRule(const api::rules::SpeedLimitRule::Id& id) {
  impl_->RemoveRule(id);
}

void ManualRulebook::AddRule(const api::rules::DirectionUsageRule& rule) {
  impl_->AddRule(rule);
}

void ManualRulebook::RemoveRule(const api::rules::DirectionUsageRule::Id& id) {
  impl_->RemoveRule(id);
}

QueryResults ManualRulebook::DoFindRules(const std::vector<LaneSRange>& ranges,
                                         double tolerance) const {
  return impl_->DoFindRules(ranges, tolerance);
}

RightOfWayRule ManualRulebook::DoGetRule(const RightOfWayRule::Id& id) const {
  return impl_->DoGetRule(id);
}

SpeedLimitRule ManualRulebook::DoGetRule(const SpeedLimitRule::Id& id) const {
  return impl_->DoGetRule(id);
}

DirectionUsageRule ManualRulebook::DoGetRule(
    const DirectionUsageRule::Id& id) const {
  return impl_->DoGetRule(id);
}

}  // namespace maliput
}  // namespace drake
