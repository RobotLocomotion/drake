#include "drake/systems/framework/diagram_context.h"

#include <stdexcept>
#include <vector>

#include <Eigen/Dense>
#include "gtest/gtest.h"

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/primitives/adder.h"
#include "drake/systems/framework/system_input.h"
#include "drake/util/eigen_matrix_compare.h"

namespace drake {

using util::CompareMatrices;
using util::MatrixCompareType;

namespace systems {
namespace {

constexpr double kTime = 12.0;

class DiagramContextTest : public ::testing::Test {
 protected:
  void SetUp() override {
    sys0_.reset(new Adder<double>(2 /* inputs */, 1 /* length */));
    sys0_->set_name("adder0");
    sys1_.reset(new Adder<double>(2 /* inputs */, 1 /* length */));
    sys1_->set_name("adder1");
    sys2_.reset(new Adder<double>(2 /* inputs */, 1 /* length */));
    sys2_->set_name("adder2");

    context_.reset(new DiagramContext<double>());
    context_->set_time(kTime);

    AddSystem(*sys0_, 0);
    AddSystem(*sys1_, 1);

    context_->ExportInput({0 /* sys0_ */, 1 /* port 1 */});
    context_->ExportInput({1 /* sys1_ */, 0 /* port 0 */});
  }

  void AddSystem(const Adder<double>& sys, int index) {
    auto subcontext = sys.CreateDefaultContext();
    auto suboutput = sys.AllocateOutput(*subcontext);
    context_->AddSystem(index, std::move(subcontext), std::move(suboutput));
  }

  void AttachInputPorts() {
    auto vec0 = std::make_unique<BasicVector<double>>(1 /* length */);
    vec0->get_mutable_value() << 128;
    auto vec1 = std::make_unique<BasicVector<double>>(1 /* length */);
    vec1->get_mutable_value() << 256;

    context_->SetInputPort(
        0, std::make_unique<FreestandingInputPort<double>>(std::move(vec0)));
    context_->SetInputPort(
        1, std::make_unique<FreestandingInputPort<double>>(std::move(vec1)));
  }

  std::unique_ptr<DiagramContext<double>> context_;
  std::unique_ptr<Adder<double>> sys0_;
  std::unique_ptr<Adder<double>> sys1_;
  std::unique_ptr<Adder<double>> sys2_;
};

// Tests that systems do not have outputs or contexts in the DiagramContext
// until they are added as constituent systems.
TEST_F(DiagramContextTest, AddAndRetrieveConstituents) {
  EXPECT_EQ(nullptr, context_->GetSubsystemOutput(2));
  EXPECT_EQ(nullptr, context_->GetSubsystemContext(2));

  AddSystem(*sys2_, 2);

  EXPECT_NE(nullptr, context_->GetSubsystemOutput(2));
  EXPECT_NE(nullptr, context_->GetSubsystemContext(2));
}

// Tests that no exception is thrown when connecting a valid source
// and destination port.
TEST_F(DiagramContextTest, ConnectValid) {
  EXPECT_NO_THROW(context_->Connect({0 /* sys0_ */, 0 /* port 0 */},
                                    {1 /* sys1_ */, 1 /* port 1 */}));
}

// Tests that an exception is thrown when connecting from a source port that
// does not exist.
TEST_F(DiagramContextTest, ConnectInvalidSrcPort) {
  EXPECT_THROW(context_->Connect({0 /* sys0_ */, 1 /* port 1 */},
                                 {1 /* sys1_ */, 1 /* port 1 */}),
               std::out_of_range);
}

// Tests that an exception is thrown when connecting to a destination port that
// does not exist.
TEST_F(DiagramContextTest, ConnectInvalidDestPort) {
  EXPECT_THROW(context_->Connect({0 /* sys0_ */, 0 /* port 0 */},
                                 {1 /* sys1_ */, 2 /* port 2 */}),
               std::out_of_range);
}

// Tests that input ports can be assigned to the DiagramContext and then
// retrieved.
TEST_F(DiagramContextTest, SetAndGetInputPorts) {
  ASSERT_EQ(2, context_->get_num_input_ports());
  AttachInputPorts();
  EXPECT_EQ(128, context_->get_vector_input(0)->get_value()[0]);
  EXPECT_EQ(256, context_->get_vector_input(1)->get_value()[0]);
}

// Tests that an exception is thrown when setting or getting input ports that
// don't exist.
TEST_F(DiagramContextTest, InvalidInputPorts) {
  EXPECT_THROW(context_->SetInputPort(2, nullptr), std::out_of_range);
  EXPECT_THROW(context_->get_vector_input(2), std::out_of_range);
}

TEST_F(DiagramContextTest, Clone) {
  context_->Connect({0 /* sys0_ */, 0 /* port 0 */},
                    {1 /* sys1_ */, 1 /* port 1 */});
  AttachInputPorts();

  std::unique_ptr<DiagramContext<double>> clone(
      dynamic_cast<DiagramContext<double>*>(context_->Clone().release()));
  ASSERT_NE(nullptr, clone);

  // Verify that the time was copied.
  EXPECT_EQ(kTime, clone->get_time());

  // Verify that the cloned input ports contain the same data,
  // but are different pointers.
  EXPECT_EQ(2, clone->get_num_input_ports());
  for (int i = 0; i < 2; ++i) {
    EXPECT_NE(context_->get_vector_input(i), clone->get_vector_input(i));
    EXPECT_TRUE(CompareMatrices(context_->get_vector_input(i)->get_value(),
                                clone->get_vector_input(i)->get_value(), 1e-8,
                                MatrixCompareType::absolute));
  }

  // Verify that the graph structure was preserved: the VectorInterface in
  // sys0 output port 0 should be pointer-equal to the VectorInterface in
  // sys1 input port 1.
  EXPECT_EQ(clone->GetSubsystemContext(1)->get_vector_input(1),
            clone->GetSubsystemOutput(0)->get_port(0).get_vector_data());
}

}  // namespace
}  // namespace systems
}  // namespace drake
