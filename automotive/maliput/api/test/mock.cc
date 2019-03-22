#include "drake/automotive/maliput/api/test/mock.h"

#include <vector>

#include "drake/automotive/maliput/api/branch_point.h"
#include "drake/automotive/maliput/api/junction.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/rules/regions.h"
#include "drake/automotive/maliput/api/rules/right_of_way_phase_ring.h"
#include "drake/automotive/maliput/api/segment.h"
#include "drake/common/drake_optional.h"

namespace drake {
namespace maliput {
namespace api {
namespace test {
namespace {

using rules::LaneSRange;
using rules::RightOfWayRule;

class MockIdIndex final : public RoadGeometry::IdIndex {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MockIdIndex);
  MockIdIndex() {}

 private:
  const Lane* DoGetLane(const LaneId&) const override { return nullptr; }
  const Segment* DoGetSegment(const SegmentId&) const override {
    return nullptr;
  };
  const Junction* DoGetJunction(const JunctionId&) const override {
    return nullptr;
  };
  const BranchPoint* DoGetBranchPoint(const BranchPointId&) const override {
    return nullptr;
  }
};

class MockRoadGeometry final : public RoadGeometry {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MockRoadGeometry)
  MockRoadGeometry() {}

 private:
  const RoadGeometryId do_id() const override { return RoadGeometryId("mock"); }
  int do_num_junctions() const override { return 1; }
  const Junction* do_junction(int) const override { return nullptr; };
  int do_num_branch_points() const override { return 1; }
  const BranchPoint* do_branch_point(int) const override { return nullptr; }
  const IdIndex& DoById() const override { return mock_id_index_; }
  RoadPosition DoToRoadPosition(const GeoPosition&, const RoadPosition*,
                                GeoPosition*, double*) const override {
    return RoadPosition();
  }
  double do_linear_tolerance() const override { return 0; }
  double do_angular_tolerance() const override { return 0; }
  double do_scale_length() const override { return 0; }
  MockIdIndex mock_id_index_;
};

class MockRoadRulebook final : public rules::RoadRulebook {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MockRoadRulebook)
  MockRoadRulebook() {}

 private:
  QueryResults DoFindRules(const std::vector<rules::LaneSRange>&,
                           double) const override {
    return {{}, {}};
  }
  rules::RightOfWayRule DoGetRule(
      const rules::RightOfWayRule::Id&) const override {
    return Rule();
  }
  rules::SpeedLimitRule DoGetRule(
      const rules::SpeedLimitRule::Id&) const override {
    const rules::LaneSRange kZone(rules::LaneSRange(LaneId("a"), {0., 9.}));
    return rules::SpeedLimitRule(rules::SpeedLimitRule::Id("some_id"), kZone,
                                 rules::SpeedLimitRule::Severity::kStrict, 33.,
                                 77.);
  }
};

class MockRightOfWayPhaseBook final : public rules::RightOfWayPhaseBook {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MockRightOfWayPhaseBook)
  MockRightOfWayPhaseBook() {}

 private:
  optional<rules::RightOfWayPhaseRing> DoGetPhaseRing(
      const rules::RightOfWayPhaseRing::Id&) const override {
    return nullopt;
  }

  optional<rules::RightOfWayPhaseRing> DoFindPhaseRing(
      const rules::RightOfWayRule::Id&) const override {
    return nullopt;
  }
};

class MockRightOfWayStateProvider final
    : public rules::RightOfWayStateProvider {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MockRightOfWayStateProvider)
  MockRightOfWayStateProvider() {}

 private:
  drake::optional<Result> DoGetState(const RightOfWayRule::Id&) const override {
    return nullopt;
  }
};

class MockRightOfWayPhaseProvider final
    : public rules::RightOfWayPhaseProvider {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MockRightOfWayPhaseProvider)
  MockRightOfWayPhaseProvider() {}

 private:
  optional<Result> DoGetPhase(
      const rules::RightOfWayPhaseRing::Id&) const override {
    return nullopt;
  }
};

class MockIntersection final : public Intersection {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MockIntersection)
  explicit MockIntersection(const Intersection::Id& id,
                            const rules::RightOfWayPhaseRing::Id& ring_id)
      : Intersection(id, {}, ring_id) {}

 private:
  const optional<rules::RightOfWayPhaseProvider::Result> Phase()
      const override {
    return nullopt;
  }
};

}  // namespace

rules::LaneSRoute LaneSRoute() {
  return rules::LaneSRoute(
      {LaneSRange(LaneId("a"), {0., 9.}), LaneSRange(LaneId("b"), {17., 12.})});
}

RightOfWayRule::State::YieldGroup YieldGroup2() {
  return {RightOfWayRule::Id("other_rule_a"),
          RightOfWayRule::Id("other_rule_b")};
}

RightOfWayRule::State NoYieldState() {
  return RightOfWayRule::State(RightOfWayRule::State::Id("s1"),
                               RightOfWayRule::State::Type::kStop, {});
}

RightOfWayRule::State YieldState() {
  return RightOfWayRule::State(RightOfWayRule::State::Id("s2"),
                               RightOfWayRule::State::Type::kGo, YieldGroup2());
}

RightOfWayRule Rule() {
  return RightOfWayRule(RightOfWayRule::Id("mock_id"), LaneSRoute(),
                        RightOfWayRule::ZoneType::kStopExcluded,
                        {NoYieldState(), YieldState()});
}

std::unique_ptr<RoadGeometry> CreateRoadGeometry() {
  return std::make_unique<MockRoadGeometry>();
}

std::unique_ptr<rules::RoadRulebook> CreateRoadRulebook() {
  return std::make_unique<MockRoadRulebook>();
}

std::unique_ptr<rules::RightOfWayPhaseBook> CreateRightOfWayPhaseBook() {
  return std::make_unique<MockRightOfWayPhaseBook>();
}

std::unique_ptr<rules::RightOfWayStateProvider>
CreateRightOfWayStateProvider() {
  return std::make_unique<MockRightOfWayStateProvider>();
}

std::unique_ptr<rules::RightOfWayPhaseProvider>
CreateRightOfWayPhaseProvider() {
  return std::make_unique<MockRightOfWayPhaseProvider>();
}

std::unique_ptr<Intersection> CreateIntersection(
    const Intersection::Id& id, const rules::RightOfWayPhaseRing::Id& ring_id) {
  return std::make_unique<MockIntersection>(id, ring_id);
}

}  // namespace test
}  // namespace api
}  // namespace maliput
}  // namespace drake
