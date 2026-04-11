#include "drake/planning/joint_limits.h"

#include <limits>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/benchmarks/acrobot/make_acrobot_plant.h"

namespace drake {
namespace planning {
namespace {

using Eigen::VectorXd;
using multibody::MultibodyPlant;

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kNan = std::numeric_limits<double>::quiet_NaN();

class JointLimitsTest : public ::testing::Test {
 protected:
  // Ordinary limit data.
  VectorXd lo_{VectorXd::Constant(3, -1)};
  VectorXd hi_{VectorXd::Constant(3, 2)};
  // Adversarial data for size checks.
  VectorXd five_{VectorXd::Zero(5)};
  // Adversarial data for finitude checks and limits checks.
  VectorXd infs_{VectorXd::Constant(3, kInf)};
  // Adversarial data for NaN checks.
  VectorXd nans_{VectorXd::Constant(3, kNan)};
};

TEST_F(JointLimitsTest, DefaultCtor) {
  JointLimits dut;
  EXPECT_EQ(dut.num_positions(), 0);
  EXPECT_EQ(dut.num_velocities(), 0);
  EXPECT_EQ(dut.num_accelerations(), 0);
}

TEST_F(JointLimitsTest, EmptyPlantCtor) {
  MultibodyPlant<double> plant{0.01};
  DRAKE_EXPECT_THROWS_MESSAGE(JointLimits(plant), ".*must call Finalize.*");
  plant.Finalize();  // Required for certain APIs used.
  JointLimits dut(plant, true, true, true);
  EXPECT_EQ(dut.num_positions(), 0);
  EXPECT_EQ(dut.num_velocities(), 0);
  EXPECT_EQ(dut.num_accelerations(), 0);
}

TEST_F(JointLimitsTest, AcrobotPlantCtor) {
  auto plant = multibody::benchmarks::acrobot::MakeAcrobotPlant(
      {}, /* finalized= */ true);
  // The acrobot plant built above has infinite limits for all derivatives.
  DRAKE_EXPECT_THROWS_MESSAGE(JointLimits(*plant, true),
                              "Position.*not finite[\\s\\S]*");
  DRAKE_EXPECT_THROWS_MESSAGE(JointLimits(*plant, false, true),
                              "Velocity.*not finite[\\s\\S]*");
  DRAKE_EXPECT_THROWS_MESSAGE(JointLimits(*plant, false, false, true),
                              "Acceleration.*not finite[\\s\\S]*");

  JointLimits dut(*plant);
  EXPECT_EQ(dut.num_positions(), 2);
  EXPECT_EQ(dut.num_velocities(), 2);
  EXPECT_EQ(dut.num_accelerations(), 2);
}

TEST_F(JointLimitsTest, EmptyPlantSelectingCtor) {
  DofMask none(0, true);
  DofMask too_many(2, true);
  MultibodyPlant<double> plant{0.01};
  DRAKE_EXPECT_THROWS_MESSAGE(JointLimits(plant, none),
                              ".*must call Finalize.*");
  plant.Finalize();  // Required for certain APIs used.
  DRAKE_EXPECT_THROWS_MESSAGE(JointLimits(plant, too_many, true, true, true),
                              ".*GetFromArray.*size.*==.*size.*");
  JointLimits dut(plant, none, true, true, true);
  EXPECT_EQ(dut.num_positions(), 0);
  EXPECT_EQ(dut.num_velocities(), 0);
  EXPECT_EQ(dut.num_accelerations(), 0);
}

TEST_F(JointLimitsTest, AcrobotPlantSelectingCtors) {
  DofMask too_few(0, true);
  DofMask some{false, true};
  DofMask too_many{true, false, true};
  auto plant = multibody::benchmarks::acrobot::MakeAcrobotPlant(
      {}, /* finalized= */ true);
  // The acrobot plant built above has infinite limits for all derivatives.
  DRAKE_EXPECT_THROWS_MESSAGE(JointLimits(*plant, some, true),
                              "Position.*not finite[\\s\\S]*");
  DRAKE_EXPECT_THROWS_MESSAGE(JointLimits(*plant, some, false, true),
                              "Velocity.*not finite[\\s\\S]*");
  DRAKE_EXPECT_THROWS_MESSAGE(JointLimits(*plant, some, false, false, true),
                              "Acceleration.*not finite[\\s\\S]*");
  // The active_dofs.size() must match the size of all limits vectors.
  DRAKE_EXPECT_THROWS_MESSAGE(JointLimits(*plant, too_few),
                              ".*GetFromArray.*size.*==.*size.*");
  DRAKE_EXPECT_THROWS_MESSAGE(JointLimits(*plant, too_many),
                              ".*GetFromArray.*size.*==.*size.*");

  JointLimits dut(*plant, some);
  EXPECT_EQ(dut.num_positions(), 1);
  EXPECT_EQ(dut.num_velocities(), 1);
  EXPECT_EQ(dut.num_accelerations(), 1);

  JointLimits from_plant(*plant);
  JointLimits selected(from_plant, some);
  EXPECT_EQ(selected.num_positions(), 1);
  EXPECT_EQ(selected.num_velocities(), 1);
  EXPECT_EQ(selected.num_accelerations(), 1);
}

TEST_F(JointLimitsTest, Position) {
  DRAKE_EXPECT_THROWS_MESSAGE(JointLimits(five_, hi_, lo_, hi_, lo_, hi_),
                              "Position.*must.*same size.*");
  DRAKE_EXPECT_THROWS_MESSAGE(JointLimits(nans_, hi_, lo_, hi_, lo_, hi_),
                              "Position.*invalid[\\s\\S]*");
  DRAKE_EXPECT_THROWS_MESSAGE(JointLimits(lo_, nans_, lo_, hi_, lo_, hi_),
                              "Position.*invalid[\\s\\S]*");
  DRAKE_EXPECT_THROWS_MESSAGE(JointLimits(hi_, lo_, lo_, hi_, lo_, hi_),
                              "Position.*invalid[\\s\\S]*");
  DRAKE_EXPECT_THROWS_MESSAGE(JointLimits(lo_, infs_, lo_, hi_, lo_, hi_, true),
                              "Position.*not finite[\\s\\S]*");
  // No exception; maybe a log message?
  JointLimits no_throw(lo_, infs_, lo_, hi_, lo_, hi_);
}

TEST_F(JointLimitsTest, Velocity) {
  DRAKE_EXPECT_THROWS_MESSAGE(JointLimits(lo_, hi_, five_, hi_, lo_, hi_),
                              "Velocity.*must.*same size.*");
  DRAKE_EXPECT_THROWS_MESSAGE(JointLimits(lo_, hi_, nans_, hi_, lo_, hi_),
                              "Velocity.*invalid[\\s\\S]*");
  DRAKE_EXPECT_THROWS_MESSAGE(JointLimits(lo_, hi_, lo_, nans_, lo_, hi_),
                              "Velocity.*invalid[\\s\\S]*");
  DRAKE_EXPECT_THROWS_MESSAGE(JointLimits(lo_, hi_, hi_, lo_, lo_, hi_),
                              "Velocity.*invalid[\\s\\S]*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      JointLimits(lo_, hi_, lo_, infs_, lo_, hi_, false, true),
      "Velocity.*not finite[\\s\\S]*");
  // No exception; maybe a log message?
  JointLimits no_throw(lo_, hi_, lo_, infs_, lo_, hi_);
}

TEST_F(JointLimitsTest, Acceleration) {
  DRAKE_EXPECT_THROWS_MESSAGE(JointLimits(lo_, hi_, lo_, hi_, five_, hi_),
                              "Acceleration.*must.*same size.*");
  DRAKE_EXPECT_THROWS_MESSAGE(JointLimits(lo_, hi_, lo_, hi_, nans_, hi_),
                              "Acceleration.*invalid[\\s\\S]*");
  DRAKE_EXPECT_THROWS_MESSAGE(JointLimits(lo_, hi_, lo_, hi_, lo_, nans_),
                              "Acceleration.*invalid[\\s\\S]*");
  DRAKE_EXPECT_THROWS_MESSAGE(JointLimits(lo_, hi_, lo_, hi_, hi_, lo_),
                              "Acceleration.*invalid[\\s\\S]*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      JointLimits(lo_, hi_, lo_, hi_, lo_, infs_, false, false, true),
      "Acceleration.*not finite[\\s\\S]*");
  // No exception; maybe a log message?
  JointLimits no_throw(lo_, hi_, lo_, hi_, lo_, infs_);
}

TEST_F(JointLimitsTest, InLimits) {
  // This offset is used to force the limit sets for the positions, velocities,
  // and accelerations to be disjoint, to help in detecting any confusion among
  // them in the implementation.
  const int size = lo_.size();
  VectorXd offset{VectorXd::Constant(size, 100)};
  JointLimits dut(lo_, hi_, lo_ + offset, hi_ + offset, lo_ + 2 * offset,
                  hi_ + 2 * offset);

  // Example tolerance and offsets for testing tolerance semantics, used below.
  constexpr double kTol = 0.1;
  const VectorXd tolerable = VectorXd::Constant(size, kTol - 1e-14);
  const VectorXd intolerable = VectorXd::Constant(size, kTol + 1e-14);

  std::array<std::function<bool(const VectorXd&, double)>, 3> methods{
      [&](const VectorXd& v, double t) {
        return dut.CheckInPositionLimits(v, t);
      },
      [&](const VectorXd& v, double t) {
        return dut.CheckInVelocityLimits(v, t);
      },
      [&](const VectorXd& v, double t) {
        return dut.CheckInAccelerationLimits(v, t);
      },
  };
  for (int k = 0; k < 3; ++k) {
    auto method = methods[k];
    const VectorXd lo_offset = lo_ + k * offset;
    const VectorXd hi_offset = hi_ + k * offset;

    DRAKE_EXPECT_THROWS_MESSAGE(method(nans_, 0.0), ".*isNaN.*");
    DRAKE_EXPECT_THROWS_MESSAGE(method(five_, 0.0),
                                ".*value.*size.*==.*lower.*size.*failed.*");
    DRAKE_EXPECT_THROWS_MESSAGE(method(lo_offset, kNan),
                                ".*tolerance.*>=.*0.0.*failed.*");
    DRAKE_EXPECT_THROWS_MESSAGE(method(lo_offset, -1),
                                ".*tolerance.*>=.*0.0.*failed.*");
    EXPECT_TRUE(method(lo_offset, 0.0));
    EXPECT_TRUE(method(hi_offset, 0.0));
    EXPECT_FALSE(method(infs_, 0.0));
    EXPECT_TRUE(method(infs_, kInf));

    // Test near the edges of tolerance semantics.
    EXPECT_FALSE(method(lo_offset - intolerable, kTol));
    EXPECT_TRUE(method(lo_offset - tolerable, kTol));
    EXPECT_FALSE(method(hi_offset + intolerable, kTol));
    EXPECT_TRUE(method(hi_offset + tolerable, kTol));
  }
}

TEST_F(JointLimitsTest, AccelerationVelocitySizeMismatch) {
  DRAKE_EXPECT_THROWS_MESSAGE(JointLimits(lo_, lo_, lo_, lo_, five_, five_),
                              ".*velocity.*size.*==.*acceleration.*size.*");
  DRAKE_EXPECT_THROWS_MESSAGE(JointLimits(lo_, lo_, five_, five_, lo_, lo_),
                              ".*velocity.*size.*==.*acceleration.*size.*");
  // Position being a different size is fine.
  JointLimits this_is_fine(five_, five_, lo_, lo_, lo_, lo_);
}

TEST_F(JointLimitsTest, CopySelectCtor) {
  JointLimits mismatched(five_, five_, lo_, lo_, lo_, lo_);
  // Copy-select constructor requires all vectors to be the same size.
  DRAKE_EXPECT_THROWS_MESSAGE(JointLimits(mismatched, DofMask(5, true)),
                              ".*size.*==.*size.*");

  JointLimits matched(lo_, hi_, lo_, hi_, lo_, hi_);
  JointLimits dut(matched, DofMask{true, false, true});
  EXPECT_EQ(dut.num_positions(), 2);
  EXPECT_EQ(dut.num_velocities(), 2);
  EXPECT_EQ(dut.num_accelerations(), 2);
}

}  // namespace
}  // namespace planning
}  // namespace drake
