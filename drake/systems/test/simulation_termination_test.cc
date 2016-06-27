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
    sys_ptr = Drake::system_test::CreateRandomAffineSystem<10, 10, 10>(
      10, 10, 10);

    xi.setZero();
    ti = 0;
    tf = 1;
  }

  // The system to simulate.
  std::shared_ptr<Drake::AffineSystem<EigenVector<10>::template type,
                             EigenVector<10>::template type,
                             EigenVector<10>::template type>> sys_ptr;

  // The initial state of the system.
  Drake::AffineSystem<EigenVector<10>::template type,
                      EigenVector<10>::template type,
                      EigenVector<10>::template type>::StateVector<double> xi;

  Drake::SimulationOptions options;

  double ti;  // The initial simulation time.
  double tf;  // The final simulation time.
};


// Tests whether Drake::simulate() can be called using default simulation
// options without an exception being thrown.
TEST_F(SimulationTerminationTest, TerminationConditionDefault) {
  EXPECT_NO_THROW(Drake::simulate(*sys_ptr, ti, tf, xi, options));\
}

// Tests whether Drake::simulate() can be called using simulation options
// with a custom termination function. This function returns true terminating
// the simulation loop.
TEST_F(SimulationTerminationTest, TerminationConditionTrue) {
  options.should_stop = []() { return true; };
  EXPECT_NO_THROW(Drake::simulate(*sys_ptr, ti, tf, xi, options));
}

// Tests whether Drake::simulate() can be called using simulation options
// customized with a different termination function. This function throws an
// exception, which should be caught.
TEST_F(SimulationTerminationTest, TerminationConditionThrow) {
  options.should_stop = []() {
    throw std::runtime_error("Terminate now!");
    return true;
  };
  EXPECT_THROW(Drake::simulate(*sys_ptr, ti, tf, xi, options),
    std::runtime_error);
}

}  // namespace
}  // namespace test
}  // namespace systems
}  // namespace drake
