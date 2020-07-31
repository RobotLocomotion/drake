#include "drake/examples/mass_spring_cloth/cloth_spring_model.h"

#include <gtest/gtest.h>

#include "drake/systems/analysis/simulator.h"

namespace drake {
namespace examples {
namespace mass_spring_cloth {
namespace {

class ContinuousClothSpringModelTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const int nx = 5;
    const int ny = 5;
    const double h = 0.05;
    const double dt = -1;  // Continuous system is run when dt <= 0.
    dut_ = std::make_unique<ClothSpringModel<double>>(nx, ny, h, dt);
    context_ = dut_->CreateDefaultContext();
  }

  std::unique_ptr<ClothSpringModel<double>> dut_;  //< The device under test.
  std::unique_ptr<systems::Context<double>> context_;
};

TEST_F(ContinuousClothSpringModelTest, Simulate) {
  // Simulate to t_final = 10.0 and make sure sim does not crash.
  const double t_final = 10.0;
  drake::systems::Simulator<double> simulator(*dut_, std::move(context_));
  simulator.Initialize();
  simulator.AdvanceTo(t_final);
  EXPECT_EQ(simulator.get_context().get_time(), t_final);
}

class DiscreteClothSpringModelTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const int nx = 5;
    const int ny = 5;
    const double h = 0.05;
    const double dt = 0.01;
    dut_ = std::make_unique<ClothSpringModel<double>>(nx, ny, h, dt);
    context_ = dut_->CreateDefaultContext();
  }

  std::unique_ptr<ClothSpringModel<double>> dut_;  // The device under test.
  std::unique_ptr<systems::Context<double>> context_;
};

TEST_F(DiscreteClothSpringModelTest, Simulate) {
  // Simulate to t_final = 10.0 and make sure sim does not crash.
  const double t_final = 10.0;
  drake::systems::Simulator<double> simulator(*dut_, std::move(context_));
  simulator.Initialize();
  simulator.AdvanceTo(t_final);
  EXPECT_EQ(simulator.get_context().get_time(), t_final);
}
}  // namespace
}  // namespace mass_spring_cloth
}  // namespace examples
}  // namespace drake
