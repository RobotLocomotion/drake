#include "drake/automotive/dev/infinite_circuit_road.h"

#include <cmath>

#include <gtest/gtest.h>

#include "drake/automotive/maliput/monolane/builder.h"

namespace drake {
namespace maliput {
namespace utility {
namespace {

namespace mono = monolane;

class InfiniteCircuitRoadTestBase : public ::testing::Test {
 protected:
  const double kLinearTolerance{1e-6};
  const double kAngularTolerance{1e-6 * M_PI};

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
};


class InfiniteCircuitRoadTest : public InfiniteCircuitRoadTestBase {
 protected:
  void SetUp() override {
    const api::RBounds kLaneBounds(-2., 2.);
    const api::RBounds kDriveableBounds(-4., 4.);
    mono::Builder b(kLaneBounds, kDriveableBounds,
                    kLinearTolerance, kAngularTolerance);
    {
      // Build a weird 'triple-oval' road --- so that we can exercise a path
      // which travels over the same lanes in both directions.
      //
      //    *   /-------9------>\   *
      //    *   |/<-4--\ /--1->\|   *
      //    *   *       *       *   *
      //    *   |       ^       |   *
      //    *   5       |       |   *
      //    *   |       |       |   *
      //    *   v       |       |   *
      //    *   |       |       |   *
      //    *   6       0       2   *
      //    *   |       |       |   *
      //    *   v       |       |   *
      //    *   |       |       |   *
      //    *   7       |       |   *
      //    *   |       |       |   *
      //    *   v       |       v   *
      //    *   *       x       *   *
      //    *   |\--8->/ \<-3--/|   *
      //    *   \<------10------/   *
      //
      const double kStraightLength = 100.;
      const double kShorterLength = 50.;
      const mono::ArcOffset kSmallCounterClockwiseArc(25., M_PI);
      const mono::ArcOffset kSmallClockwiseArc(25., -M_PI);
      const mono::ArcOffset kLargeClockwiseArc(50., -M_PI);
      const mono::EndpointZ kFlatLowZ(-2., 0., 0., 0.);
      const mono::EndpointZ kFlatHighZ(12., 0., 0., 0.);
      const double kTilt = M_PI / 10.;
      const mono::EndpointZ kTiltedSlopedUpperZ(10., -10. / kShorterLength,
                                                kTilt, 0.);
      const mono::EndpointZ kTiltedSlopedLowerZ(0., -10. / kShorterLength,
                                                kTilt, 0.);

      const mono::Endpoint start{{0., 0., M_PI / 2.}, kFlatLowZ};
      auto c0 = b.Connect("0", start, kStraightLength, kFlatHighZ);

      auto c1 = b.Connect("1", c0->end(), kSmallClockwiseArc, kFlatHighZ);
      auto c2 = b.Connect("2", c1->end(), kStraightLength, kFlatLowZ);
      b.Connect("3", c2->end(), kSmallClockwiseArc, kFlatLowZ);

      auto c4 = b.Connect("4", c0->end(), kSmallCounterClockwiseArc,
                          kFlatHighZ);
      auto c5 = b.Connect("5", c4->end(),
                          0.5 * (kStraightLength - kShorterLength),
                          kTiltedSlopedUpperZ);
      auto c6 = b.Connect("6", c5->end(),
                          kShorterLength, kTiltedSlopedLowerZ);
      auto c7 = b.Connect("7", c6->end(),
                          0.5 * (kStraightLength - kShorterLength),
                          kFlatLowZ);
      b.Connect("8", c7->end(), kSmallCounterClockwiseArc, kFlatLowZ);

      b.Connect("9", c4->end().reverse(), kLargeClockwiseArc, kFlatHighZ);
      b.Connect("10", c2->end(), kLargeClockwiseArc, kFlatLowZ);
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
            FindLane(source_.get(), "l:8"),
            FindLane(source_.get(), "l:9"),
            FindLane(source_.get(), "l:10")
            });

    start_ = api::LaneEnd(source_lanes_[0], api::LaneEnd::kStart);
    path_ = std::vector<const api::Lane*>({
        source_lanes_[0],
            source_lanes_[1],
            source_lanes_[2],
            source_lanes_[10],
            source_lanes_[7],
            source_lanes_[6],
            source_lanes_[5],
            source_lanes_[9],
            source_lanes_[2],
            source_lanes_[3],
            source_lanes_[0],
            source_lanes_[4],
            source_lanes_[5],
            source_lanes_[6],
            source_lanes_[7],
            source_lanes_[8]
            });
    // We're constructing the path such that the first traversal of
    // lanes "l:5", "l:6", "l:7" are in reverse direction.  Those are
    // elements #4, #5, #6 in the circuit.
    // And, element #10 is "l:6" traversed in forward direction.
    reversed_set_.insert({4, 5, 6});
    reversed6_index_ = 5;
    forward6_index_ = 13;

    dut_id_.id = "dut";
    dut_ = std::make_unique<InfiniteCircuitRoad>(
        dut_id_, source_.get(), start_,
        // Skip first lane, which is inferred from start_.
        std::vector<const api::Lane*>(path_.begin() + 1, path_.end()));
  }


  std::unique_ptr<const api::RoadGeometry> source_;
  std::vector<const api::Lane*> source_lanes_;
  api::LaneEnd start_;
  std::vector<const api::Lane*> path_;
  std::set<size_t> reversed_set_;
  size_t reversed6_index_;
  size_t forward6_index_;
  api::RoadGeometryId dut_id_;
  std::unique_ptr<const InfiniteCircuitRoad> dut_;
};


TEST_F(InfiniteCircuitRoadTest, CheckInvariants) {
  EXPECT_TRUE(dut_->CheckInvariants().empty());
}


TEST_F(InfiniteCircuitRoadTest, BasicTopology) {
  EXPECT_EQ(0, dut_->num_branch_points());
  EXPECT_EQ(1, dut_->num_junctions());
  EXPECT_EQ(1, dut_->junction(0)->num_segments());
  EXPECT_EQ(1, dut_->junction(0)->segment(0)->num_lanes());
  EXPECT_EQ(dut_->lane(), dut_->junction(0)->segment(0)->lane(0));
}


TEST_F(InfiniteCircuitRoadTest, Tolerances) {
  EXPECT_DOUBLE_EQ(kLinearTolerance, dut_->linear_tolerance());
  EXPECT_DOUBLE_EQ(kAngularTolerance, dut_->angular_tolerance());
}


TEST_F(InfiniteCircuitRoadTest, Id) {
  EXPECT_EQ(dut_id_.id, dut_->id().id);
}


TEST_F(InfiniteCircuitRoadTest, Source) {
  EXPECT_EQ(source_.get(), dut_->source());
}


TEST_F(InfiniteCircuitRoadTest, CycleLength) {
  double cycle_length = 0;
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
  EXPECT_EQ(static_cast<int>(path_.size()), dut_->lane()->num_path_records());

  double cumulative_length = 0;
  for (size_t i = 0; i < path_.size(); ++i) {
    const InfiniteCircuitRoad::Record& rec = dut_->lane()->path_record(i);
    EXPECT_EQ(path_[i], rec.lane);
    EXPECT_EQ(cumulative_length, rec.start_circuit_s);
    EXPECT_EQ((reversed_set_.count(i) > 0), rec.is_reversed);

    const double s = cumulative_length + (0.25 * path_[i]->length());
    EXPECT_EQ(static_cast<int>(i), dut_->lane()->GetPathIndex(s));

    cumulative_length += path_[i]->length();
  }
}


TEST_F(InfiniteCircuitRoadTest, ProjectToSourceRoad) {
  for (size_t i = 0; i < path_.size(); ++i) {
    const double local_s = 0.25 * path_[i]->length();
    const double circuit_s =
        dut_->lane()->path_record(i).start_circuit_s + local_s;

    api::RoadPosition road_pos;
    bool is_reversed;
    std::tie(road_pos, is_reversed) = dut_->lane()->ProjectToSourceRoad(
        {circuit_s, 17.0, 23.0});
    EXPECT_EQ(path_[i], road_pos.lane);
    if (reversed_set_.count(i) > 0) {
      EXPECT_NEAR((path_[i]->length() - local_s), road_pos.pos.s,
                  kLinearTolerance);
      EXPECT_DOUBLE_EQ(-17.0, road_pos.pos.r);
    } else {
      EXPECT_NEAR(local_s, road_pos.pos.s, kLinearTolerance);
      EXPECT_DOUBLE_EQ(17.0, road_pos.pos.r);
    }
    EXPECT_DOUBLE_EQ(23.0, road_pos.pos.h);
    EXPECT_EQ((reversed_set_.count(i) > 0), is_reversed);
  }
}


TEST_F(InfiniteCircuitRoadTest, ToGeoPosition) {
  // Path records are trustworthy via another test.
  for (size_t i = 0; i < path_.size(); ++i) {
    const double local_s = 0.25 * dut_->lane()->path_record(i).lane->length();
    const double circuit_s =
        dut_->lane()->path_record(i).start_circuit_s + local_s;

    const api::GeoPosition dut_pos =
        dut_->lane()->ToGeoPosition({circuit_s, 17.0, 23.0});

    const api::GeoPosition expected_pos = path_[i]->ToGeoPosition(
        (reversed_set_.count(i) > 0) ?
        api::LanePosition(path_[i]->length() - local_s, -17.0, 23.0) :
        api::LanePosition(local_s, 17.0, 23.0));

    EXPECT_NEAR(expected_pos.x, dut_pos.x, kLinearTolerance);
    EXPECT_NEAR(expected_pos.y, dut_pos.y, kLinearTolerance);
    EXPECT_NEAR(expected_pos.z, dut_pos.z, kLinearTolerance);
  }
}

// Apply a Rotation to a 3-vector (generically represented by a GeoPosition).
// TODO(maddog@tri.global)  This should probably be a method of Rotation, and or
//                          consolidated with something else somehow.
api::GeoPosition Rotate(const api::Rotation& rot, const api::GeoPosition& in) {
  const double sa = std::sin(rot.roll);
  const double ca = std::cos(rot.roll);
  const double sb = std::sin(rot.pitch);
  const double cb = std::cos(rot.pitch);
  const double sg = std::sin(rot.yaw);
  const double cg = std::cos(rot.yaw);

  return api::GeoPosition(
      ((cb * cg) * in.x) +
      ((-ca*sg + sa*sb*cg) * in.y) +
      ((sa*sg + ca*sb*cg) * in.z),

      ((cb*sg) * in.x) +
      ((ca*cg + sa*sb*sg) * in.y) +
      ((-sa*cg + ca*sb*sg) * in.z),

      ((-sb) * in.x) +
      ((sa*cb) * in.y) +
      ((ca*cb) * in.z));
}

::testing::AssertionResult IsRotationClose(
     const api::Rotation& expected, const api::Rotation& actual,
     double tolerance) {
  // Compute transformed unit vectors of expected frame.
  api::GeoPosition es = Rotate(expected, {1., 0., 0.});
  api::GeoPosition er = Rotate(expected, {0., 1., 0.});
  api::GeoPosition eh = Rotate(expected, {0., 0., 1.});
  // Compute transformed unit vectors of actual frame.
  api::GeoPosition as = Rotate(actual, {1., 0., 0.});
  api::GeoPosition ar = Rotate(actual, {0., 1., 0.});
  api::GeoPosition ah = Rotate(actual, {0., 0., 1.});
  // Compute angles between pairs of unit vectors.
  double ds = std::acos((es.x * as.x) + (es.y * as.y) + (es.z * as.z));
  double dr = std::acos((er.x * ar.x) + (er.y * ar.y) + (er.z * ar.z));
  double dh = std::acos((eh.x * ah.x) + (eh.y * ah.y) + (eh.z * ah.z));

  double d = std::sqrt((ds * ds) + (dr * dr) + (dh * dh));

  if (d > tolerance) {
    return ::testing::AssertionFailure()
        << "Distance between expected {"
        << expected.roll << ", " << expected.pitch << ", " << expected.yaw
        << "} and actual {"
        << actual.roll << ", " << actual.pitch << ", " << actual.yaw
        << "} exceeds tolerance " << tolerance << ".";
  }
  return ::testing::AssertionSuccess();
}


TEST_F(InfiniteCircuitRoadTest, GetOrientation) {
  const double kSOffset = 12.5;  // == 0.25 * kShorterLength;

  // Path records are trustworthy via another test.
  double circuit_s =
      dut_->lane()->path_record(forward6_index_).start_circuit_s + kSOffset;
  api::Rotation actual = dut_->lane()->GetOrientation({circuit_s, 1.0, 1.0});
  api::Rotation expected(
      // We know the roll from kTilt.
      M_PI / 10.,
      // We know the pitch from kTiltedSlopedUpper.z, kTiltedSlopedLower.z,
      // and kShorterLength.
      -std::atan2(-10., 50.),
      // We know the yaw from how we constructed it.
      -M_PI / 2.);

  EXPECT_TRUE(IsRotationClose(expected, actual, kAngularTolerance));

  // Repeat, traversing Lane 6 in the reverse direction.
  circuit_s =
      dut_->lane()->path_record(reversed6_index_).start_circuit_s + kSOffset;
  actual = dut_->lane()->GetOrientation({circuit_s, 1.0, 1.0});
  expected = api::Rotation(-M_PI / 10.,
                           -std::atan2(10., 50.),
                           M_PI / 2.);
  EXPECT_TRUE(IsRotationClose(expected, actual, kAngularTolerance));
}


TEST_F(InfiniteCircuitRoadTest, EvalMotionDerivatives) {
  const double kSOffset = 12.5;  // == 0.25 * kShorterLength;

  // Path records are trustworthy via another test.
  double circuit_s =
      dut_->lane()->path_record(forward6_index_).start_circuit_s + kSOffset;
  api::LanePosition actual =
      dut_->lane()->EvalMotionDerivatives({circuit_s, 17., 23.},
                                          {1.0, 2.0, 3.0});
  // Lane is not curved, so derivatives should equal input velocities.
  api::LanePosition expected(1.0, 2.0, 3.0);
  EXPECT_NEAR(expected.s, actual.s, kLinearTolerance);
  EXPECT_NEAR(expected.r, actual.r, kLinearTolerance);
  EXPECT_NEAR(expected.h, actual.h, kLinearTolerance);

  // Repeat, traversing Lane 6 in the reverse direction.
  circuit_s =
      dut_->lane()->path_record(reversed6_index_).start_circuit_s + kSOffset;
  actual = dut_->lane()->EvalMotionDerivatives({circuit_s, 17., 23.},
                                               {1.0, 2.0, 3.0});
  // Same expected value.
  EXPECT_NEAR(expected.s, actual.s, kLinearTolerance);
  EXPECT_NEAR(expected.r, actual.r, kLinearTolerance);
  EXPECT_NEAR(expected.h, actual.h, kLinearTolerance);
}



TEST_F(InfiniteCircuitRoadTestBase, Bounds) {
  // Construct single-lane ring road with asymmetric bounds.
  const api::RBounds kLaneBounds(-1., 7.);
  const api::RBounds kDriveableBounds(-2., 10.);
  mono::Builder b(kLaneBounds, kDriveableBounds,
                  kLinearTolerance, kAngularTolerance);

  const mono::EndpointZ kZeroZ{0., 0., 0., 0.};
  const mono::Endpoint start{{0., 0., 0.}, kZeroZ};
  b.Connect("0", start, mono::ArcOffset(100., 2. * M_PI), kZeroZ);
  auto source = b.Build({"source"});

  InfiniteCircuitRoad forward_dut(
      {"forward_dut"}, source.get(),
      api::LaneEnd(source->junction(0)->segment(0)->lane(0),
                   api::LaneEnd::kStart),
      std::vector<const api::Lane*>());

  api::RBounds forward_lane_actual = forward_dut.lane()->lane_bounds(0.);
  EXPECT_NEAR(-1., forward_lane_actual.r_min, kLinearTolerance);
  EXPECT_NEAR(7., forward_lane_actual.r_max, kLinearTolerance);

  api::RBounds forward_driveable_actual =
      forward_dut.lane()->driveable_bounds(0.);
  EXPECT_NEAR(-2., forward_driveable_actual.r_min, kLinearTolerance);
  EXPECT_NEAR(10., forward_driveable_actual.r_max, kLinearTolerance);

  InfiniteCircuitRoad reverse_dut(
      {"forward_dut"}, source.get(),
      api::LaneEnd(source->junction(0)->segment(0)->lane(0),
                   api::LaneEnd::kFinish),
      std::vector<const api::Lane*>());

  api::RBounds reverse_lane_actual = reverse_dut.lane()->lane_bounds(0.);
  EXPECT_NEAR(-7., reverse_lane_actual.r_min, kLinearTolerance);
  EXPECT_NEAR(1., reverse_lane_actual.r_max, kLinearTolerance);

  api::RBounds reverse_driveable_actual =
      reverse_dut.lane()->driveable_bounds(0.);
  EXPECT_NEAR(-10., reverse_driveable_actual.r_min, kLinearTolerance);
  EXPECT_NEAR(2., reverse_driveable_actual.r_max, kLinearTolerance);
}


TEST_F(InfiniteCircuitRoadTestBase, CircuitFinding) {
  // To exercise the "find a circuit if path is not specified" behavior,
  // craft a source road which has a spur which merges into a ring.
  // ('cause the trick is find the circuit around the ring, and discard
  // the spur.)
  const api::RBounds kLaneBounds(-1., 1.);
  const api::RBounds kDriveableBounds(-2., 2.);
  mono::Builder b(kLaneBounds, kDriveableBounds,
                  kLinearTolerance, kAngularTolerance);

  const double kSpurLength = 100.;
  const double kRingRadius = 50.;
  const mono::EndpointZ kZeroZ{0., 0., 0., 0.};
  const mono::Endpoint start{{0., 0., 0.}, kZeroZ};
  auto c0 = b.Connect("spur", start, kSpurLength, kZeroZ);
  b.Connect("ring", c0->end(), mono::ArcOffset(kRingRadius, 2. * M_PI), kZeroZ);
  auto source = b.Build({"source"});

  InfiniteCircuitRoad dut(
      {"dut"}, source.get(),
      api::LaneEnd(FindLane(source.get(), "l:spur"), api::LaneEnd::kStart),
      std::vector<const api::Lane*>());

  EXPECT_TRUE(dut.CheckInvariants().empty());
  EXPECT_NEAR(kRingRadius * 2. * M_PI, dut.lane()->cycle_length(),
              kLinearTolerance);
  EXPECT_EQ(1, dut.lane()->num_path_records());
  EXPECT_EQ(FindLane(source.get(), "l:ring"), dut.lane()->path_record(0).lane);
  EXPECT_NEAR(0., dut.lane()->path_record(0).start_circuit_s,
              kLinearTolerance);
}


}  // namespace
}  // namespace utility
}  // namespace maliput
}  // namespace drake
