#include "drake/system1/Simulation.h"

#include <stdexcept>

#include "gtest/gtest.h"

#include "drake/systems/test/system_test_util.h"


using Eigen::Dynamic;

namespace drake {
namespace systems {
namespace test {
namespace {

using drake::EigenVector;

class SimulationTerminationTest : public ::testing::Test {
 public:
  void SetUp() override {
    sys_ptr_ = drake::system_test::CreateRandomAffineSystem<10, 10, 10>(
      10, 10, 10);

    xi_.setZero();
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // The system to simulate.
  std::shared_ptr<drake::AffineSystem<EigenVector<10>::template type,
                             EigenVector<10>::template type,
                             EigenVector<10>::template type>> sys_ptr_;

  // The initial state of the system.
  drake::AffineSystem<EigenVector<10>::template type,
                      EigenVector<10>::template type,
                      EigenVector<10>::template type>::StateVector<double> xi_;

  drake::SimulationOptions options_;

  double ti_{0};  // The initial simulation time.
  double tf_{1};  // The final simulation time.
};


// Tests whether drake::simulate() can be called using default simulation
// options. Since the default termination function returns false, the simulation
// should continue to run till the final simulation time (tf_) is reached. Thus,
// the final simulation time should be greater than or equal to tf_.
TEST_F(SimulationTerminationTest, TerminationConditionDefault) {
  EXPECT_GE(drake::simulate(*sys_ptr_, ti_, tf_, xi_, options_), tf_);
}

// Tests whether drake::simulate() can be called using simulation options
// with a custom termination function. This function returns true after the
// simulation has advanced 0.5 seconds in simulation time. Thus, the final
// simulation time should be greater than or equal to 0.5 seconds, and less than
// or equal to 0.5 seconds plus the step size.
TEST_F(SimulationTerminationTest, TerminationConditionAfterHalfSecond) {
  double termination_time = 0.5;
  options_.should_stop = [&termination_time](double sim_time) {
    return sim_time >= termination_time;
  };
  double final_sim_time = drake::simulate(*sys_ptr_, ti_, tf_, xi_, options_);
  EXPECT_GE(final_sim_time, termination_time);
  EXPECT_LE(final_sim_time, termination_time + options_.initial_step_size);
}

// Tests whether drake::simulate() can be called using simulation options
// with a custom termination function. This function immediately returns true
// terminating the simulation loop. Thus, the final simulation time should be
// equal to the initial simulation time (ti_).
TEST_F(SimulationTerminationTest, TerminationConditionTrue) {
  options_.should_stop = [](double sim_time) { return true; };
  EXPECT_EQ(drake::simulate(*sys_ptr_, ti_, tf_, xi_, options_), ti_);
}

}  // namespace
}  // namespace test
}  // namespace systems
}  // namespace drake
