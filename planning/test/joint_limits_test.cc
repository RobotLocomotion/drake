#include "planning/joint_limits.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"

namespace anzu {
namespace planning {
namespace {

using drake::multibody::MultibodyPlant;
using Eigen::VectorXd;

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
  plant.Finalize();  // Required for certain APIs used.
  JointLimits dut(plant, true, true, true);
  EXPECT_EQ(dut.num_positions(), 0);
  EXPECT_EQ(dut.num_velocities(), 0);
  EXPECT_EQ(dut.num_accelerations(), 0);
}

TEST_F(JointLimitsTest, Position) {
  DRAKE_EXPECT_THROWS_MESSAGE(JointLimits(five_, hi_, lo_, hi_, lo_, hi_),
                              "Position.*must.*same size.*");
  DRAKE_EXPECT_THROWS_MESSAGE(JointLimits(nans_, hi_, lo_, hi_, lo_, hi_),
                              "Position.*no lower.*exceed.*upper[\\s\\S]*");
  DRAKE_EXPECT_THROWS_MESSAGE(JointLimits(lo_, nans_, lo_, hi_, lo_, hi_),
                              "Position.*no lower.*exceed.*upper[\\s\\S]*");
  DRAKE_EXPECT_THROWS_MESSAGE(JointLimits(hi_, lo_, lo_, hi_, lo_, hi_),
                              "Position.*no lower.*exceed.*upper[\\s\\S]*");
  DRAKE_EXPECT_THROWS_MESSAGE(JointLimits(lo_, infs_, lo_, hi_, lo_, hi_, true),
                              "Position.*not finite[\\s\\S]*");
  // No exception; maybe a log message?
  JointLimits no_throw(lo_, infs_, lo_, hi_, lo_, hi_);
}

TEST_F(JointLimitsTest, Velocity) {
  DRAKE_EXPECT_THROWS_MESSAGE(JointLimits(lo_, hi_, five_, hi_, lo_, hi_),
                              "Velocity.*must.*same size.*");
  DRAKE_EXPECT_THROWS_MESSAGE(JointLimits(lo_, hi_, nans_, hi_, lo_, hi_),
                              "Velocity.*no lower.*exceed.*upper[\\s\\S]*");
  DRAKE_EXPECT_THROWS_MESSAGE(JointLimits(lo_, hi_, lo_, nans_, lo_, hi_),
                              "Velocity.*no lower.*exceed.*upper[\\s\\S]*");
  DRAKE_EXPECT_THROWS_MESSAGE(JointLimits(lo_, hi_, hi_, lo_, lo_, hi_),
                              "Velocity.*no lower.*exceed.*upper[\\s\\S]*");
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
                              "Acceleration.*no lower.*exceed.*upper[\\s\\S]*");
  DRAKE_EXPECT_THROWS_MESSAGE(JointLimits(lo_, hi_, lo_, hi_, lo_, nans_),
                              "Acceleration.*no lower.*exceed.*upper[\\s\\S]*");
  DRAKE_EXPECT_THROWS_MESSAGE(JointLimits(lo_, hi_, lo_, hi_, hi_, lo_),
                              "Acceleration.*no lower.*exceed.*upper[\\s\\S]*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      JointLimits(lo_, hi_, lo_, hi_, lo_, infs_, false, false, true),
      "Acceleration.*not finite[\\s\\S]*");
  // No exception; maybe a log message?
  JointLimits no_throw(lo_, hi_, lo_, hi_, lo_, infs_);
}

TEST_F(JointLimitsTest, InLimits) {
  // This offset is used to force the limit sets for the derivatives to be
  // disjoint, to help in detecting any confusion among derivatives in the
  // implementation.
  Eigen::VectorXd offset{VectorXd::Constant(3, 100)};
  JointLimits dut(lo_, hi_, lo_ + offset, hi_ + offset, lo_ + 2 * offset,
                  hi_ + 2 * offset);

  std::array<std::string, 3> derivative_names{"position", "velocity",
                                              "acceleration"};
  std::array<std::function<bool(const Eigen::VectorXd&, double)>, 3> methods{
      [&](const Eigen::VectorXd& v, double t) {
        return dut.CheckInPositionLimits(v, t);
      },
      [&](const Eigen::VectorXd& v, double t) {
        return dut.CheckInVelocityLimits(v, t);
      },
      [&](const Eigen::VectorXd& v, double t) {
        return dut.CheckInAccelerationLimits(v, t);
      },
  };
  for (int k = 0; k < 3; ++k) {
    auto method = methods[k];
    auto derivative_name = derivative_names[k];
    const Eigen::VectorXd lo_offset = lo_ + k * offset;
    const Eigen::VectorXd hi_offset = hi_ + k * offset;

    DRAKE_EXPECT_THROWS_MESSAGE(method(nans_, 0.0), ".*isNaN.*");
    DRAKE_EXPECT_THROWS_MESSAGE(
        method(five_, 0.0),
        fmt::format(".*{}.*size.*==.*{}.*lower.*size.*failed.*",
                    derivative_name, derivative_name));
    DRAKE_EXPECT_THROWS_MESSAGE(method(lo_offset, kNan),
                                ".*tolerance.*>=.*0.0.*failed.*");
    DRAKE_EXPECT_THROWS_MESSAGE(method(lo_offset, -1),
                                ".*tolerance.*>=.*0.0.*failed.*");
    EXPECT_TRUE(method(lo_offset, 0.0));
    EXPECT_TRUE(method(hi_offset, 0.0));
    EXPECT_FALSE(method(infs_, 0.0));
    EXPECT_TRUE(method(infs_, kInf));
  }
}

TEST_F(JointLimitsTest, AccelerationVelocitySizeMismatch) {
  DRAKE_EXPECT_THROWS_MESSAGE(JointLimits(lo_, lo_, lo_, lo_, five_, five_),
                              ".*velocity.*size.*==.*acceleration.*size.*");
}

}  // namespace
}  // namespace planning
}  // namespace anzu
