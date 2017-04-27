#include "drake/systems/framework/sparsity_matrix.h"

#include <memory>

#include <gtest/gtest.h>

#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {
namespace {

const int kSize = 2;

class SparseSystem : public LeafSystem<symbolic::Expression> {
 public:
  SparseSystem() {
    this->DeclareInputPort(kVectorValued, kSize);
    this->DeclareInputPort(kVectorValued, kSize);
    this->DeclareOutputPort(kVectorValued, kSize);
    this->DeclareOutputPort(kVectorValued, kSize);
    this->DeclareAbstractOutputPort();

    this->DeclareContinuousState(kSize);
    this->DeclareDiscreteState(kSize);
  }

  void AddAbstractInputPort() {
    this->DeclareAbstractInputPort();
  }

  ~SparseSystem() override {}

 protected:
  void DoCalcOutput(const Context<symbolic::Expression>& context,
                    SystemOutput<symbolic::Expression>* output) const override {
    const auto& u0 = *(this->EvalVectorInput(context, 0));
    const auto& u1 = *(this->EvalVectorInput(context, 1));
    auto& y0 = *(output->GetMutableVectorData(0));
    auto& y1 = *(output->GetMutableVectorData(1));

    const auto& xc = context.get_continuous_state_vector();
    const auto& xd = *(context.get_discrete_state(0));

    // Output 0 depends on input 0 and the continuous state.  Input 1 appears in
    // an intermediate computation, but is ultimately cancelled out.
    y0.set_value(u1.get_value());
    y0.PlusEqScaled(1, u0);
    y0.PlusEqScaled(-1, u1);
    y0.PlusEqScaled(12, xc);

    // Output 1 depends on both inputs and the discrete state.
    y1.set_value(u0.get_value() + u1.get_value() + xd.get_value());
  }

  std::unique_ptr<AbstractValue> AllocateOutputAbstract(
      const OutputPortDescriptor<symbolic::Expression>& descriptor)
  const override {
    return AbstractValue::Make<int>(42);
  }
};

class SparsityMatrixTest : public ::testing::Test {
 public:
  SparsityMatrixTest()
      : system_() {}

 protected:
  void SetUp() override {
    matrix_ = std::make_unique<SparsityMatrix>(system_);
  }

  SparseSystem system_;
  std::unique_ptr<SparsityMatrix> matrix_;
};

// Tests that the SparsityMatrix infers, from the symbolic equations of the
// System, that input 1 does not affect output 0.
TEST_F(SparsityMatrixTest, InputToOutput) {
  // Only input 0 affects output 0.
  EXPECT_TRUE(matrix_->IsConnectedInputToOutput(0, 0));
  EXPECT_FALSE(matrix_->IsConnectedInputToOutput(1, 0));
  // Both inputs affect output 1.
  EXPECT_TRUE(matrix_->IsConnectedInputToOutput(0, 1));
  EXPECT_TRUE(matrix_->IsConnectedInputToOutput(1, 1));
  // All inputs are presumed to affect output 2, since it is abstract.
  EXPECT_TRUE(matrix_->IsConnectedInputToOutput(0, 2));
  EXPECT_TRUE(matrix_->IsConnectedInputToOutput(1, 2));
}

// Tests that, if the System has an abstract input, the SparsityMatrix
// conservatively reports that every output might depend on every input.
TEST_F(SparsityMatrixTest, AbstractContextThrwartsSparsity) {
  system_.AddAbstractInputPort();
  matrix_ = std::make_unique<SparsityMatrix>(system_);
  for (int i = 0; i < system_.get_num_input_ports(); ++i) {
    for (int j = 0; j < system_.get_num_output_ports(); ++j) {
      EXPECT_TRUE(matrix_->IsConnectedInputToOutput(i, j));
    }
  }
}

}  // namespace
}  // namespace systems
}  // namespace drake
