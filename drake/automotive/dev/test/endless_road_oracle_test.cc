/* clang-format off */
#include "drake/automotive/dev/endless_road_oracle.h"
#include "drake/automotive/dev/endless_road_oracle-internal.h"
/* clang-format on */

#include <memory>

#include <gtest/gtest.h>

#include "drake/automotive/dev/infinite_circuit_road.h"
#include "drake/automotive/maliput/monolane/builder.h"

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
  const double kRingLength{1000.};
  // NB:  Each ring is divided into a 300 deg and 60 deg segment by
  // the tangent points.  (See road network construction below.)
  const double k300DegLength{kRingLength * 300. / 360.};
  const double k60DegLength{kRingLength * 60. / 360.};

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
      const double kRadius = kRingLength / (2. * M_PI);
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


::testing::AssertionResult IsOracleOutputNear(
     double expected_net_delta_sigma,
     double expected_delta_sigma_dot,
     const EndlessRoadOracleOutput<double>& actual_output,
     double linear_tolerance) {
  if (!(IsNear(actual_output.net_delta_sigma(),
               expected_net_delta_sigma, linear_tolerance) &&
        IsNear(actual_output.delta_sigma_dot(),
               expected_delta_sigma_dot, linear_tolerance))) {
    return ::testing::AssertionFailure()
        << "Expected output (" << expected_net_delta_sigma
        << ", " << expected_delta_sigma_dot << ") differs from actual output ("
        << actual_output.net_delta_sigma() << ", "
        << actual_output.delta_sigma_dot() << ") by more than "
        << linear_tolerance << ".";
  }
  return ::testing::AssertionSuccess();
}


TEST_F(EndlessRoadOracleInternalTest, UnwrapEndlessRoadCarState) {
  std::vector<EndlessRoadCarState<double>> inputs(5);
  const double kSpeed = 10.;
  inputs[0].set_s(10.);
  inputs[0].set_speed(kSpeed);

  inputs[1].set_s(1010.);
  inputs[1].set_speed(kSpeed);

  inputs[2].set_s(2010.);
  inputs[2].set_speed(kSpeed);

  inputs[3].set_s(4500.);
  inputs[3].set_speed(kSpeed);

  inputs[4].set_s(3510.);
  inputs[4].set_speed(kSpeed);


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
                                 k300DegLength - 10., 0., 0., kSpeed,
                                 source_states[1], kLinearTolerance));
  EXPECT_TRUE(IsSourceStateClose(source_lanes_[0],
                                 k300DegLength - 10., 0., 0., kSpeed,
                                 source_states[2], kLinearTolerance));
  EXPECT_TRUE(IsSourceStateClose(source_lanes_[0], 500., 0., 0., kSpeed,
                                 source_states[3], kLinearTolerance));
  EXPECT_TRUE(IsSourceStateClose(source_lanes_[2], 510., 0., 0., kSpeed,
                                 source_states[4], kLinearTolerance));

  ASSERT_EQ(5, paths[4].size());
  EXPECT_TRUE(IsPathRecordEq(source_lanes_[2], false, paths[4][0]));
  EXPECT_TRUE(IsPathRecordEq(source_lanes_[5], true,  paths[4][1]));
  EXPECT_TRUE(IsPathRecordEq(source_lanes_[0], false, paths[4][2]));
  EXPECT_TRUE(IsPathRecordEq(source_lanes_[1], false, paths[4][3]));
  EXPECT_TRUE(IsPathRecordEq(source_lanes_[4], true,  paths[4][4]));
}


TEST_F(EndlessRoadOracleInternalTest, AssessForwardPath) {
  // Set up a 'world' with three cars.
  std::vector<internal::SourceState> source_states;
  std::vector<std::vector<internal::PathRecord>> paths;

  source_states.emplace_back(
      api::RoadPosition {source_lanes_[0], {400., 0., 0.}}, 1., 1.);
  paths.emplace_back(std::vector<internal::PathRecord> {
      {source_lanes_[0], false},
      {source_lanes_[1], false},
      {source_lanes_[4], true}});

  source_states.emplace_back(
      api::RoadPosition {source_lanes_[0], {600., 0., 0.}}, 1., 5.);
  paths.emplace_back(std::vector<internal::PathRecord> {
      {source_lanes_[0], false},
      {source_lanes_[1], false},
      {source_lanes_[4], true}});

  source_states.emplace_back(
      api::RoadPosition {source_lanes_[3], {0., 0., 0.}}, 1., 10.);
  paths.emplace_back(std::vector<internal::PathRecord> {
      {source_lanes_[3], false},
      {source_lanes_[0], true},
      {source_lanes_[5], false},
      {source_lanes_[2], true}});

  source_states.emplace_back(
      api::RoadPosition {source_lanes_[5], {50., 0., 0.}}, 1., 100.);
  paths.emplace_back(std::vector<internal::PathRecord> {
      {source_lanes_[5], true},
      {source_lanes_[0], false},
      {source_lanes_[3], true},
      {source_lanes_[4], false}});

  std::vector<EndlessRoadOracleOutput<double>> outputs(source_states.size());
  std::vector<EndlessRoadOracleOutput<double>*> output_ptrs;
  for (EndlessRoadOracleOutput<double>& output : outputs) {
    output_ptrs.push_back(&output);
  }

  internal::AssessForwardPath(source_states, paths, output_ptrs);

  ASSERT_EQ(4, outputs.size());
  EXPECT_TRUE(IsOracleOutputNear(
      (600. - 400. - internal::kCarLength), -4., outputs[0],
      kLinearTolerance));
  EXPECT_TRUE(IsOracleOutputNear(internal::kEnormousDistance, 0., outputs[1],
                                 kLinearTolerance));
  EXPECT_TRUE(IsOracleOutputNear(
      (k60DegLength + (k300DegLength - 600.) - internal::kCarLength), 15.,
      outputs[2],
      kLinearTolerance));
  EXPECT_TRUE(IsOracleOutputNear(
      (50. + 400. - internal::kCarLength), 99., outputs[3],
      kLinearTolerance));
}


// TODO(maddog@tri.global) Implement these tests!
TEST_F(EndlessRoadOracleInternalTest, DetermineLaneRelation) {}
TEST_F(EndlessRoadOracleInternalTest, AssessJunctions) {}
TEST_F(EndlessRoadOracleInternalTest, MeasureJunctions) {}

}  // namespace
}  // namespace automotive
}  // namespace drake
