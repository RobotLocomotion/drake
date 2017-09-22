#include "drake/systems/primitives/piecewise_affine_system.h"

#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/eigen_types.h"

using std::make_unique;
using std::unique_ptr;

namespace drake {
namespace systems {
namespace {

class TestPWASystem : public PolytopicListPiecewiseAffineSystem<double> {
 public:
  explicit TestPWASystem(double time_period = 0.0)
      : PolytopicListPiecewiseAffineSystem<double>(1, 2, 1, time_period) {
    // clang-format off
    A << 1, 2,
        3, 4;
    B << 5,
        6;
    f0 << 7,
        8;
    C << 9, 10;
    D << 11;
    y0 << 12;
    Px << 1, 0;
    Pu << 0;
    prhs << 0;
    // clang-format on

    // For x(0) <= 0
    this->AddPiece(A, B, f0, C, D, y0, Px, Pu, prhs);
    // For x(0) > 0
    this->AddPiece(
        std::make_unique<AffineSystem<double>>(2 * A, 3 * B, 4 * f0, 5 * C,
                                               6 * D, 7 * y0, time_period),
        std::make_unique<PolytopicPiecewiseAffineSystem<double>::Domain>(
            -Px, Pu, prhs));
  }

  Eigen::MatrixXd A{2, 2}, B{2, 1}, f0{2, 1}, C{1, 2}, D{1, 1}, y0{1, 1};
  Eigen::MatrixXd Px{1, 2}, Pu{1, 1}, prhs{1, 1};
};

GTEST_TEST(PiecewiseAffineSystemTest, CheckPiecesTest) {
  // TestPWASystem adds pieces using both AddPiece(eigen...) and
  // AddPiece(system...).  Checks that the correct
  // coefficients are extracted for both pieces.
  TestPWASystem dut;

  EXPECT_EQ(dut.get_num_pieces(), 2);
  const auto& piece0 = dut.GetSystemAtIndex(0);
  EXPECT_EQ(piece0.A(), dut.A);
  EXPECT_EQ(piece0.B(), dut.B);
  EXPECT_EQ(piece0.f0(), dut.f0);
  EXPECT_EQ(piece0.C(), dut.C);
  EXPECT_EQ(piece0.D(), dut.D);
  EXPECT_EQ(piece0.y0(), dut.y0);
  const auto& domain0 = dut.GetDomainAtIndex(0);
  EXPECT_EQ(domain0.Px(), dut.Px);
  EXPECT_EQ(domain0.Pu(), dut.Pu);
  EXPECT_EQ(domain0.prhs(), dut.prhs);

  const auto& piece1 = dut.GetSystemAtIndex(1);
  EXPECT_EQ(piece1.A(), 2 * dut.A);
  EXPECT_EQ(piece1.B(), 3 * dut.B);
  EXPECT_EQ(piece1.f0(), 4 * dut.f0);
  EXPECT_EQ(piece1.C(), 5 * dut.C);
  EXPECT_EQ(piece1.D(), 6 * dut.D);
  EXPECT_EQ(piece1.y0(), 7 * dut.y0);
  const auto& domain1 = dut.GetDomainAtIndex(1);
  EXPECT_EQ(domain1.Px(), -dut.Px);
  EXPECT_EQ(domain1.Pu(), dut.Pu);
  EXPECT_EQ(domain1.prhs(), dut.prhs);

  auto context = dut.CreateDefaultContext();
  context->FixInputPort(0, Vector1d(5.0));
  context->get_mutable_continuous_state_vector()->SetFromVector(
      Eigen::Vector2d(1.0, 0.0));
  EXPECT_EQ(dut.GetSystemIndexAtContext(*context), 1);
  const auto& piece2 = dut.GetSystemAtContext(*context);
  EXPECT_EQ(piece2.A(), 2 * dut.A);
  const auto& domain2 = dut.GetDomainAtContext(*context);
  EXPECT_EQ(domain2.Px(), -dut.Px);

  context->get_mutable_continuous_state_vector()->SetFromVector(
      Eigen::Vector2d(-1.0, 0.0));
  EXPECT_EQ(dut.GetSystemIndexAtContext(*context), 0);
  const auto& piece3 = dut.GetSystemAtContext(*context);
  EXPECT_EQ(piece3.A(), dut.A);
  const auto& domain3 = dut.GetDomainAtContext(*context);
  EXPECT_EQ(domain3.Px(), dut.Px);
}

GTEST_TEST(PiecewiseAffineSystemTest, CheckOutputTest) {
  TestPWASystem dut;

  auto context = dut.CreateDefaultContext();
  Vector1d input(1.0);
  context->FixInputPort(0, input);

  Eigen::Vector2d state(-2.0, 3.0);
  context->get_mutable_continuous_state_vector()->SetFromVector(state);

  const OutputPort<double>& output_port = dut.get_output_port();
  std::unique_ptr<AbstractValue> output = output_port.Allocate(*context);
  output_port.Calc(*context, output.get());
  auto y = output->GetValueOrThrow<BasicVector<double>>().CopyToVector();
  EXPECT_TRUE(CompareMatrices(dut.C * state + dut.D * input + dut.y0, y));

  state(0) = 2.0;
  context->get_mutable_continuous_state_vector()->SetFromVector(state);
  output_port.Calc(*context, output.get());
  y = output->GetValueOrThrow<BasicVector<double>>().CopyToVector();

  EXPECT_TRUE(
      CompareMatrices(5 * dut.C * state + 6 * dut.D * input + 7 * dut.y0, y));
}

GTEST_TEST(PiecewiseAffineSystemTest, CheckTimeDerivativesTest) {
  TestPWASystem dut;

  auto context = dut.CreateDefaultContext();
  Vector1d input(1.0);
  context->FixInputPort(0, input);

  EXPECT_TRUE(context->has_only_continuous_state());

  Eigen::Vector2d state(-2.0, 3.0);
  context->get_mutable_continuous_state_vector()->SetFromVector(state);

  auto derivatives = dut.AllocateTimeDerivatives();
  dut.CalcTimeDerivatives(*context, derivatives.get());
  EXPECT_TRUE(CompareMatrices(dut.A * state + dut.B * input + dut.f0,
                              derivatives->CopyToVector()));

  state(0) = 2.0;
  context->get_mutable_continuous_state_vector()->SetFromVector(state);
  dut.CalcTimeDerivatives(*context, derivatives.get());
  EXPECT_TRUE(
      CompareMatrices(2 * dut.A * state + 3 * dut.B * input + 4 * dut.f0,
                      derivatives->CopyToVector()));
}

GTEST_TEST(PiecewiseAffineSystemTest, CheckDiscreteUpdateTest) {
  TestPWASystem dut(1.0);

  auto context = dut.CreateDefaultContext();
  Vector1d input(1.0);
  context->FixInputPort(0, input);

  EXPECT_TRUE(context->has_only_discrete_state());

  Eigen::Vector2d state(-2.0, 3.0);
  context->get_mutable_discrete_state(0)->SetFromVector(state);

  auto updates = dut.AllocateDiscreteVariables();
  dut.CalcDiscreteVariableUpdates(*context, updates.get());
  EXPECT_TRUE(CompareMatrices(dut.A * state + dut.B * input + dut.f0,
                              updates->get_vector(0)->get_value()));

  state(0) = 2.0;
  context->get_mutable_discrete_state(0)->SetFromVector(state);
  dut.CalcDiscreteVariableUpdates(*context, updates.get());
  EXPECT_TRUE(
      CompareMatrices(2 * dut.A * state + 3 * dut.B * input + 4 * dut.f0,
                      updates->get_vector(0)->get_value()));
}

}  // namespace
}  // namespace systems
}  // namespace drake
