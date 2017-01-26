#include "drake/automotive/endless_road_oracle.h"
#include "drake/automotive/endless_road_oracle-internal.h"

#include <memory>

#include "gtest/gtest.h"

#include "drake/automotive/maliput/monolane/builder.h"
#include "drake/automotive/maliput/utility/infinite_circuit_road.h"

namespace drake {
namespace automotive {
namespace {

namespace api = drake::maliput::api;
namespace mono = drake::maliput::monolane;
namespace utility = drake::maliput::utility;



class EndlessRoadOracleInternalTestBase : public ::testing::Test {
 protected:
  const double kLinearTolerance {1e-6};
  const double kAngularTolerance {1e-6 * M_PI};

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


class EndlessRoadOracleInternalTest : public EndlessRoadOracleInternalTestBase {
 protected:
  void SetUp() override {
    const api::RBounds kLaneBounds(-2., 2.);
    const api::RBounds kDriveableBounds(-4., 4.);
    mono::Builder b(kLaneBounds, kDriveableBounds,
                    kLinearTolerance, kAngularTolerance);
    {
      // Build a 'triple ring' road:  three rings of 1000m circumference,
      // tangent to each other.  Schematically:
      //
      //     4------->\ /--------0
      //     |         *         |
      //     |        / ^        |
      //     |       /   \       |
      //     |      5     1      |
      //     |     /       \     |
      //     |    v         \    |
      //     |   *-----3---->*   |
      //     +--/ \         / \<-+
      //           \<--2---/
      //
      const double kRadius = 1000. / (2. * M_PI);
      const mono::EndpointZ kFlatZ(0., 0., 0., 0.);
      const mono::Endpoint start {{0., 0., M_PI / 2.}, kFlatZ};
      const mono::ArcOffset kClockwise300Deg(kRadius, -M_PI * 5. / 3.);
      const mono::ArcOffset kClockwise60Deg(kRadius, -M_PI * 1. / 3.);

      auto c0 = b.Connect("0", start, kClockwise300Deg, kFlatZ);
      b.Connect("1", c0->end(), kClockwise60Deg, kFlatZ);

      auto c2 = b.Connect("2", c0->end().reverse(), kClockwise300Deg, kFlatZ);
      b.Connect("3", c2->end(), kClockwise60Deg, kFlatZ);

      auto c4 = b.Connect("4", c2->end().reverse(), kClockwise300Deg, kFlatZ);
      b.Connect("5", c4->end(), kClockwise60Deg, kFlatZ);
    }
    source_road_ = b.Build({"source"});

    source_lanes_ = std::vector<const api::Lane*>({
        FindLane(source_road_.get(), "l:0"),
            FindLane(source_road_.get(), "l:1"),
            FindLane(source_road_.get(), "l:2"),
            FindLane(source_road_.get(), "l:3"),
            FindLane(source_road_.get(), "l:4"),
            FindLane(source_road_.get(), "l:5"),
            });


    start_ = api::LaneEnd(source_lanes_[0], api::LaneEnd::kStart);
    path_ = std::vector<const api::Lane*>({
        source_lanes_[0],
            source_lanes_[1],
            source_lanes_[4],
            source_lanes_[3],
            source_lanes_[0],
            source_lanes_[1],
            source_lanes_[2],
            source_lanes_[5]});

    infinite_road_ = std::make_unique<utility::InfiniteCircuitRoad>(
        api::RoadGeometryId {"infinite"}, source_road_.get(), start_,
        std::vector<const api::Lane*>(path_.begin() + 1, path_.end()));
  }

  std::unique_ptr<const api::RoadGeometry> source_road_;
  std::vector<const api::Lane*> source_lanes_;
  api::LaneEnd start_;
  std::vector<const api::Lane*> path_;
  std::unique_ptr<const utility::InfiniteCircuitRoad> infinite_road_;
};


bool IsNear(double a, double b, double margin) {
  return std::abs(a - b) <= margin;
}


::testing::AssertionResult IsSourceStateClose(
     const api::Lane* expected_lane,
     double expected_s, double expected_r, double expected_h,
     double expected_circuit_s_speed,
     const internal::SourceState& actual_source_state,
     double linear_tolerance) {
  if (actual_source_state.rp.lane != expected_lane) {
    return ::testing::AssertionFailure()
        << "Expected lane " << expected_lane->id().id << " ("
        << expected_lane << ") does not match actual lane "
        << actual_source_state.rp.lane->id().id << " ("
        << actual_source_state.rp.lane << ").";
  }
  if (!(IsNear(actual_source_state.rp.pos.s, expected_s, linear_tolerance) &&
        IsNear(actual_source_state.rp.pos.r, expected_r, linear_tolerance) &&
        IsNear(actual_source_state.rp.pos.h, expected_h, linear_tolerance))) {
    return ::testing::AssertionFailure()
        << "Expected lane position (" << expected_s
        << ", " << expected_r << ", " << expected_h << ") differs from actual ("
        << actual_source_state.rp.pos.s << ", "
        << actual_source_state.rp.pos.r << ", "
        << actual_source_state.rp.pos.h << ") by more than "
        << linear_tolerance << " in at least one component.";
  }
  if (!IsNear(actual_source_state.circuit_s_speed,
              expected_circuit_s_speed, linear_tolerance)) {
    return ::testing::AssertionFailure()
        << "Expected circuit-s speed " << expected_circuit_s_speed
        << " differs from actual " << actual_source_state.circuit_s_speed
        << " by more than " << linear_tolerance << ".";
  }
  return ::testing::AssertionSuccess();
}


::testing::AssertionResult IsPathRecordEq(
     const api::Lane* expected_lane, bool expected_is_reversed,
     const internal::PathRecord& actual_path_record) {
  if ((actual_path_record.lane != expected_lane) ||
      (actual_path_record.is_reversed != expected_is_reversed)) {
    return ::testing::AssertionFailure()
        << "Expected lane " << expected_lane->id().id << " ("
        << expected_lane << ") and is_reversed " << expected_is_reversed
        << " does not match actual lane "
        << actual_path_record.lane->id().id << " ("
        << actual_path_record.lane << ") and is_reversed "
        << actual_path_record.is_reversed << ".";
  }
  return ::testing::AssertionSuccess();
}


TEST_F(EndlessRoadOracleInternalTest, UnwrapEndlessRoadCarState) {

  std::vector<EndlessRoadCarState<double>> inputs(4);
  const double kSpeed = 10.;
  inputs[0].set_s(10.);
  inputs[0].set_speed(kSpeed);

  inputs[1].set_s(1010.);
  inputs[1].set_speed(kSpeed);

  inputs[2].set_s(1990.);
  inputs[2].set_speed(kSpeed);

  inputs[3].set_s(2010.);
  inputs[3].set_speed(kSpeed);


  std::vector<const EndlessRoadCarState<double>*> input_ptrs;
  for (const EndlessRoadCarState<double>& input : inputs) {
    input_ptrs.push_back(&input);
  }

  std::vector<internal::SourceState> source_states(inputs.size());
  std::vector<std::vector<internal::PathRecord>> paths(inputs.size());

  const double kHorizonMeters = 1500.;
  const double kHorizonSeconds = kHorizonMeters / kSpeed;
  internal::UnwrapEndlessRoadCarState(input_ptrs, infinite_road_.get(),
                                      kHorizonSeconds,
                                      &source_states, &paths);
  EXPECT_TRUE(IsSourceStateClose(source_lanes_[0], 10., 0., 0., kSpeed,
                                 source_states[0], kLinearTolerance));
  EXPECT_TRUE(IsSourceStateClose(source_lanes_[4],
                                 (1000. * 5. / 6.) - 10., 0., 0., kSpeed,
                                 source_states[1], kLinearTolerance));
  EXPECT_TRUE(IsSourceStateClose(source_lanes_[3],
                                 (1000. / 6.) - 10., 0., 0., kSpeed,
                                 source_states[2], kLinearTolerance));
  EXPECT_TRUE(IsSourceStateClose(source_lanes_[0],
                                 (1000. * 5. / 6.) - 10., 0., 0., kSpeed,
                                 source_states[3], kLinearTolerance));

  ASSERT_EQ(3, paths[0].size());
  EXPECT_TRUE(IsPathRecordEq(source_lanes_[0], false, paths[0][0]));
  EXPECT_TRUE(IsPathRecordEq(source_lanes_[1], false, paths[0][1]));
  EXPECT_TRUE(IsPathRecordEq(source_lanes_[4], true,  paths[0][2]));
}

#if 0
TEST_F(EndlessRoadOracleInternalTest, AssessLongitudinal) {


}
TEST_F(EndlessRoadOracleInternalTest, DetermineLaneRelation) {


}
TEST_F(EndlessRoadOracleInternalTest, AssessInteractions) {


}
  // Create an InfiniteCircuitRoad wrapping a simple ring road.
  //
  // The ring is centered at origin with radius 50.
  // The start of the ring is at (50., 0.) with heading (pi/2) (i.e.,
  // +y direction).  The ring has a 0.22rad superelevation, tilted outward.
  const double kLinearTolerance = 0.01;
  const double kAngularTolerance = 0.01 * M_PI;
  const api::RBounds kLaneBounds {-1., 1.};
  const api::RBounds kDriveableBounds {-2., 2.};
  mono::Builder b(kLaneBounds, kDriveableBounds,
                  kLinearTolerance, kAngularTolerance);
  const double kSuperelevation = 0.22;  // radians
  const mono::EndpointZ kTiltedZ {0., 0., kSuperelevation, 0.};
  const double kRingRadius = 50.;
  const mono::Endpoint start {{kRingRadius, 0., 0.5 * M_PI}, kTiltedZ};
  b.Connect("0", start, mono::ArcOffset(kRingRadius, 2. * M_PI), kTiltedZ);
  const std::unique_ptr<const api::RoadGeometry> ring = b.Build({"ring"});
  const utility::InfiniteCircuitRoad road(
      {"road"}, ring.get(),
      api::LaneEnd(ring->junction(0)->segment(0)->lane(0),
                   api::LaneEnd::kStart),
      std::vector<const api::Lane*>());

  // The device under test.
  auto dut =
      std::make_unique<EndlessRoadCarToEulerFloatingJoint<double>>(&road);
  auto context = dut->CreateDefaultContext();
  auto output = dut->AllocateOutput(*context);

  // Set an input value.
  auto value = std::make_unique<EndlessRoadCarState<double>>();
  const double kS = kRingRadius * M_PI;  // half-way around the ring
  const double kR = 10.;
  value->set_s(kS);
  value->set_r(kR);
  value->set_heading(0.5 * M_PI);  // pointing due-left along road
  value->set_speed(0.);  // (Speed should not matter.)
  context->SetInputPort(
      0, std::make_unique<systems::FreestandingInputPort>(std::move(value)));

  // Grab a pointer to where the CalcOutput results end up.
  const EulerFloatingJointState<double>* const result =
      dynamic_cast<const EulerFloatingJointState<double>*>(
          output->get_vector_data(0));
  ASSERT_NE(nullptr, result);

  // Confirm output values.
  dut->CalcOutput(*context, output.get());
  EXPECT_NEAR((kRingRadius - (kR * std::cos(kSuperelevation))) *
              std::cos(M_PI),
              result->x(), kLinearTolerance);
  EXPECT_NEAR((kRingRadius - (kR * std::cos(kSuperelevation))) *
              kRingRadius * std::sin(M_PI),
              result->y(), kLinearTolerance);
  EXPECT_NEAR(kR * std::sin(kSuperelevation),
              result->z(), kLinearTolerance);
  EXPECT_NEAR(0., result->yaw(), kAngularTolerance);
  EXPECT_NEAR(-kSuperelevation, result->pitch(), kAngularTolerance);
  EXPECT_NEAR(0., result->roll(), kAngularTolerance);
}
#endif

}  // namespace
}  // namespace automotive
}  // namespace drake
