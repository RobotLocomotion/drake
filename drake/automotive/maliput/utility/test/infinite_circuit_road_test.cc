#include "drake/automotive/maliput/utility/infinite_circuit_road.h"

#include <cmath>
#include <iostream>

#include "drake/automotive/maliput/monolane/builder.h"

#include "gtest/gtest.h"

namespace drake {
namespace maliput {
namespace utility {
namespace {

namespace mono = monolane;

class InfiniteCircuitRoadTest : public ::testing::Test {
 protected:
  const double kLinearTolerance {0.01};
  const double kAngularTolerance {0.01 * M_PI};

  const api::Lane* FindLane(const api::RoadGeometry* road,
                            const std::string& id_string) {
    for (int ji = 0; ji < road->num_junctions(); ++ji) {
      const api::Junction* junction = road->junction(ji);
      for (int si = 0; si < junction->num_segments(); ++si) {
        const api::Segment* segment = junction->segment(si);
        DRAKE_DEMAND(segment->num_lanes() == 1);
        const api::Lane* lane = segment->lane(0);
        if (lane->id().id == id_string) {
          return lane;
        }
      }
    }
    DRAKE_ABORT();
  }

  void SetUp() override {
    const api::RBounds kLaneBounds(-2., 2.);
    const api::RBounds kDriveableBounds(-4., 4.);
    mono::Builder b(kLaneBounds, kDriveableBounds,
                    kLinearTolerance, kAngularTolerance);
    {
      // Build a weird 'triple-oval' road --- so that we can exercise a path
      // which travels over the same lanes in both directions.
      //
      //    *   /-----7-----\   *
      //    *   |/-4-\ /-1-\|   *
      //    *   *     *     *   *
      //    *   |     |     |   *
      //    *   5     0     2   *
      //    *   |     |     |   *
      //    *   *     x     *   *
      //    *   |\-6-/ \-3-/|   *
      //    *   \-----8-----/   *
      //
      const double kStraightLength = 100;
      const mono::ArcOffset kSmallCounterClockwiseArc(25, M_PI);
      const mono::ArcOffset kSmallClockwiseArc(25, -M_PI);
      const mono::ArcOffset kLargeClockwiseArc(50, -M_PI);
      const mono::EndpointZ kZeroZ(0., 0., 0., 0.);

      const mono::Endpoint start {{0., 0., M_PI / 2.}, kZeroZ};
      auto c0 = b.Connect("0", start, kStraightLength, kZeroZ);

      auto c1 = b.Connect("1", c0->end(), kSmallClockwiseArc, kZeroZ);
      auto c2 = b.Connect("2", c1->end(), kStraightLength, kZeroZ);
      b.Connect("3", c2->end(), kSmallClockwiseArc, kZeroZ);

      auto c4 = b.Connect("4", c0->end(), kSmallCounterClockwiseArc, kZeroZ);
      auto c5 = b.Connect("5", c4->end(), kStraightLength, kZeroZ);
      b.Connect("6", c5->end(), kSmallCounterClockwiseArc, kZeroZ);

      b.Connect("7", c4->end().reverse(), kLargeClockwiseArc, kZeroZ);
      b.Connect("8", c2->end(), kLargeClockwiseArc, kZeroZ);
    }
    source_ = b.Build({"source"});

    source_lanes_ = std::vector<const api::Lane*>({
        FindLane(source_.get(), "l:0"),
            FindLane(source_.get(), "l:1"),
            FindLane(source_.get(), "l:2"),
            FindLane(source_.get(), "l:3"),
            FindLane(source_.get(), "l:4"),
            FindLane(source_.get(), "l:5"),
            FindLane(source_.get(), "l:6"),
            FindLane(source_.get(), "l:7"),
            FindLane(source_.get(), "l:8")
            });

    start_ = api::LaneEnd(source_lanes_[0], api::LaneEnd::kStart);
    path_ = std::vector<const api::Lane*>({
      source_lanes_[1],
          source_lanes_[2],
          source_lanes_[8],
          source_lanes_[5],
          source_lanes_[7],
          source_lanes_[2],
          source_lanes_[3],
          source_lanes_[0],
          source_lanes_[4],
          source_lanes_[5],
          source_lanes_[6]
          });

    api::RoadGeometryId dut_id {"dut"};
    dut_ = std::make_unique<InfiniteCircuitRoad>(
        dut_id, source_.get(), start_, path_);
  }


  std::unique_ptr<const api::RoadGeometry> source_;
  std::vector<const api::Lane*> source_lanes_;
  api::LaneEnd start_;
  std::vector<const api::Lane*> path_;
  std::unique_ptr<const InfiniteCircuitRoad> dut_;
};


TEST_F(InfiniteCircuitRoadTest, CheckInvariants) {
  EXPECT_TRUE(dut_->CheckInvariants().empty());
}


TEST_F(InfiniteCircuitRoadTest, source) {
  EXPECT_EQ(source_.get(), dut_->source());
}


TEST_F(InfiniteCircuitRoadTest, CycleLength) {
  double cycle_length = start_.lane->length();
  for (const auto lane : path_) {
    cycle_length += lane->length();
  }
  EXPECT_EQ(cycle_length, dut_->lane()->cycle_length());

  const double kSomeOffset = 100.;
  ASSERT_TRUE(cycle_length > kSomeOffset);
  EXPECT_NEAR(
      kSomeOffset, dut_->lane()->circuit_s(kSomeOffset), kLinearTolerance);
  EXPECT_NEAR(
      kSomeOffset,
      dut_->lane()->circuit_s(kSomeOffset + cycle_length), kLinearTolerance);
  EXPECT_NEAR(
      kSomeOffset,
      dut_->lane()->circuit_s(kSomeOffset + (9000. * cycle_length)),
      kLinearTolerance);
  EXPECT_NEAR(
      kSomeOffset,
      dut_->lane()->circuit_s(kSomeOffset - (9000. * cycle_length)),
      kLinearTolerance);
}


TEST_F(InfiniteCircuitRoadTest, PathRecords) {
  EXPECT_EQ(1 + path_.size(), dut_->lane()->num_path_records());

  EXPECT_EQ(start_.lane, dut_->lane()->path_record(0).lane);
  EXPECT_EQ(0., dut_->lane()->path_record(0).start_circuit_s);
  EXPECT_EQ(start_.lane->length(), dut_->lane()->path_record(0).end_circuit_s);
  EXPECT_EQ(0, dut_->lane()->GetPathIndex(0.5 * start_.lane->length()));
  double cumulative_length = start_.lane->length();


  for (size_t i = 0; i < path_.size(); ++i) {
    const InfiniteCircuitRoad::Record& rec = dut_->lane()->path_record(i + 1);
    EXPECT_EQ(path_[i], rec.lane);
    EXPECT_EQ(cumulative_length, rec.start_circuit_s);
    EXPECT_EQ(cumulative_length + path_[i]->length(), rec.end_circuit_s);
    EXPECT_EQ(i + 1,
              dut_->lane()->GetPathIndex(cumulative_length +
                                         (0.5 * start_.lane->length())));
    cumulative_length += path_[i]->length();
  }
}

#if 0
TEST_F(InfiniteCircuitRoadTest, ) {

}


TEST_F(InfiniteCircuitRoadTest, ) {

}


TEST_F(InfiniteCircuitRoadTest, ) {

}


TEST_F(InfiniteCircuitRoadTest, ) {

}


TEST_F(InfiniteCircuitRoadTest, ) {

}


TEST_F(InfiniteCircuitRoadTest, ) {

}


TEST_F(InfiniteCircuitRoadTest, ) {

}


TEST_F(InfiniteCircuitRoadTest, ) {

}


TEST_F(InfiniteCircuitRoadTest, ) {

}


TEST_F(InfiniteCircuitRoadTest, ) {

}


TEST_F(InfiniteCircuitRoadTest, ) {

}


TEST_F(InfiniteCircuitRoadTest, ) {

}


TEST_F(InfiniteCircuitRoadTest, ) {

}


TEST_F(InfiniteCircuitRoadTest, ) {

}
#endif


}  // namespace
}  // namespace utility
}  // namespace maliput
}  // namespace drake
