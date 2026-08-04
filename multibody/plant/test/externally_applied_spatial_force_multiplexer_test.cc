#include "drake/multibody/plant/externally_applied_spatial_force_multiplexer.h"

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "drake/systems/framework/fixed_input_port_value.h"
#include "drake/systems/framework/test_utilities/scalar_conversion.h"

namespace drake {
namespace multibody {
namespace {

using ListType = std::vector<ExternallyAppliedSpatialForce<double>>;

ExternallyAppliedSpatialForce<double> MakeDummyForce(BodyIndex body_index) {
  return {.body_index = body_index,
          .p_BoBq_B = Eigen::Vector3d::Zero(),
          .F_Bq_W = SpatialForce<double>::Zero()};
}

class ExternallyAppliedSpatialForceMultiplexerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    system_ =
        std::make_unique<ExternallyAppliedSpatialForceMultiplexer<double>>(2);
    context_ = system_->CreateDefaultContext();

    input0_ = {MakeDummyForce(BodyIndex{0}), MakeDummyForce(BodyIndex{1})};
    input1_ = {MakeDummyForce(BodyIndex{1}), MakeDummyForce(BodyIndex{2})};
  }

  std::unique_ptr<ExternallyAppliedSpatialForceMultiplexer<double>> system_;
  std::unique_ptr<systems::Context<double>> context_;
  ListType input0_;
  ListType input1_;
};

// Tests that the system exports the correct topology.
TEST_F(ExternallyAppliedSpatialForceMultiplexerTest, Topology) {
  const int num_inputs = 2;
  ASSERT_EQ(num_inputs, system_->num_input_ports());
  for (int i = 0; i < num_inputs; ++i) {
    const systems::InputPort<double>& input_port = system_->get_input_port(i);
    EXPECT_EQ(systems::kAbstractValued, input_port.get_data_type());
  }

  ASSERT_EQ(1, system_->num_output_ports());
  const systems::OutputPort<double>& output_port = system_->get_output_port(0);
  EXPECT_EQ(&output_port, &system_->get_output_port());
  EXPECT_EQ(systems::kAbstractValued, output_port.get_data_type());
}

void AssertEqual(const ExternallyAppliedSpatialForce<double>& lhs,
                 const ExternallyAppliedSpatialForce<double>& rhs) {
  ASSERT_EQ(lhs.body_index, rhs.body_index);
  ASSERT_EQ(lhs.p_BoBq_B, rhs.p_BoBq_B);
  ASSERT_EQ(lhs.F_Bq_W.get_coeffs(), rhs.F_Bq_W.get_coeffs());
}

void AssertEqual(const ListType& lhs, const ListType& rhs) {
  size_t count = lhs.size();
  ASSERT_EQ(count, rhs.size());
  for (size_t i = 0; i < count; ++i) {
    SCOPED_TRACE(fmt::format("Element {}", i));
    AssertEqual(lhs[i], rhs[i]);
  }
}

// Tests that the system computes the correct output.
TEST_F(ExternallyAppliedSpatialForceMultiplexerTest, ConcatenateTwoVectors) {
  system_->get_input_port(0).FixValue(context_.get(), input0_);
  system_->get_input_port(1).FixValue(context_.get(), input1_);
  const ListType expected = {
      MakeDummyForce(BodyIndex{0}), MakeDummyForce(BodyIndex{1}),
      MakeDummyForce(BodyIndex{1}), MakeDummyForce(BodyIndex{2})};
  const ListType actual = system_->get_output_port().Eval<ListType>(*context_);
  AssertEqual(actual, expected);
}

// Tests that system allocates no state variables in the context_.
TEST_F(ExternallyAppliedSpatialForceMultiplexerTest, Stateless) {
  EXPECT_EQ(0, context_->num_continuous_states());
  EXPECT_EQ(0, context_->num_discrete_state_groups());
}

// Asserts that system has direct-feedthrough from all inputs to the output.
TEST_F(ExternallyAppliedSpatialForceMultiplexerTest, IsDirectFeedthrough) {
  EXPECT_TRUE(system_->HasAnyDirectFeedthrough());
  const int output_index = 0;
  for (int i = 0; i < system_->num_input_ports(); ++i) {
    EXPECT_TRUE(system_->HasDirectFeedthrough(i, output_index));
  }
}

TEST_F(ExternallyAppliedSpatialForceMultiplexerTest, ToAutoDiff) {
  EXPECT_TRUE(systems::is_autodiffxd_convertible(*system_));
}

TEST_F(ExternallyAppliedSpatialForceMultiplexerTest, ToSymbolic) {
  EXPECT_TRUE(systems::is_symbolic_convertible(*system_));
}

}  // namespace
}  // namespace multibody
}  // namespace drake
