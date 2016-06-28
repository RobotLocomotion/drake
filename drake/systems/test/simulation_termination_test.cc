#include "drake/systems/Simulation.h"

#include <stdexcept>

#include "gtest/gtest.h"

#include "drake/systems/test/system_test_util.h"


using Eigen::Dynamic;

namespace drake {
namespace systems {
namespace test {
namespace {

using Drake::EigenVector;

class SimulationTerminationTest : public ::testing::Test {
 public:
  void SetUp() override {
    sys_ptr_ = Drake::system_test::CreateRandomAffineSystem<10, 10, 10>(
      10, 10, 10);

    xi_.setZero();
  }

  // The system to simulate.
  std::shared_ptr<Drake::AffineSystem<EigenVector<10>::template type,
                             EigenVector<10>::template type,
                             EigenVector<10>::template type>> sys_ptr_;

  // The initial state of the system.
  Drake::AffineSystem<EigenVector<10>::template type,
                      EigenVector<10>::template type,
                      EigenVector<10>::template type>::StateVector<double> xi_;

  Drake::SimulationOptions options_;

  double ti_{0};  // The initial simulation time.
  double tf_{1};  // The final simulation time.
};


// Tests whether Drake::simulate() can be called using default simulation
// options. Since the default termination function returns false, the simulation
// should continue to run till the final simulation time (tf_) is reached. Thus,
// the final simulation time should be greater than or equal to tf_.
TEST_F(SimulationTerminationTest, TerminationConditionDefault) {
  EXPECT_GE(Drake::simulate(*sys_ptr_, ti_, tf_, xi_, options_), tf_);
}

// Tests whether Drake::simulate() can be called using simulation options
// with a custom termination function. This function returns true after the
// simulation has advanced 0.5 seconds in simulation time. Thus, the final
// simulation time should be greater than or equal to 0.5 seconds.
TEST_F(SimulationTerminationTest, TerminationConditionAfterHalfSecond) {
  double termination_time = 0.5;
  options_.should_stop = [&termination_time](double sim_time) {
    return sim_time >= termination_time;
  };
  EXPECT_GE(Drake::simulate(*sys_ptr_, ti_, tf_, xi_, options_),
    termination_time);
}

// Tests whether Drake::simulate() can be called using simulation options
// with a custom termination function. This function immediately returns true
// terminating the simulation loop. Thus, the final simulation time should be
// equal to the initial simulation time (ti_).
TEST_F(SimulationTerminationTest, TerminationConditionTrue) {
  options_.should_stop = [](double sim_time) { return true; };
  EXPECT_EQ(Drake::simulate(*sys_ptr_, ti_, tf_, xi_, options_), ti_);
}

// Tests whether Drake::simulate() can be called using simulation options
// customized with a different termination function. This function throws an
// exception, which should be caught.
TEST_F(SimulationTerminationTest, TerminationConditionThrow) {
  options_.should_stop = [](double sim_time) {
    throw std::runtime_error("Terminate now!");
    return true;
  };
  EXPECT_THROW(Drake::simulate(*sys_ptr_, ti_, tf_, xi_, options_),
    std::runtime_error);
}

}  // namespace
}  // namespace test
}  // namespace systems
}  // namespace drake
