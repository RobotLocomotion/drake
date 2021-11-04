// This test covers DiscreteValues (used directly by LeafContexts) and its
// derived class DiagramDiscreteValues (for DiagramContexts).

#include "drake/systems/framework/discrete_values.h"

#include <memory>
#include <stdexcept>

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/test_utilities/is_dynamic_castable.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/diagram_discrete_values.h"
#include "drake/systems/framework/test_utilities/my_vector.h"

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;

namespace drake {
namespace systems {
namespace {

class DiscreteValuesTest : public ::testing::Test {
 public:
  DiscreteValuesTest() {
    data_.push_back(std::make_unique<BasicVector<double>>(v00_));
    data_.push_back(std::make_unique<MyVector2d>(v01_));
    data1_.push_back(std::make_unique<BasicVector<double>>(v10_));
    data1_.push_back(std::make_unique<MyVector3d>(v11_));
    data1_.push_back(std::make_unique<MyVector2d>(v12_));
  }

 protected:
  const VectorXd v00_ = Vector2d{1., 1.},
                 v01_ = Vector2d{2., 3.},
                 v10_ = Vector3d{9., 10., 11.},
                 v11_ = Vector3d{-1.25, -2.5, -3.75},
                 v12_ = Vector2d{5., 6.};
  std::vector<std::unique_ptr<BasicVector<double>>> data_;
  std::vector<std::unique_ptr<BasicVector<double>>> data1_;
};

TEST_F(DiscreteValuesTest, OwnedState) {
  DiscreteValues<double> xd(std::move(data_));
  EXPECT_EQ(xd.get_vector(0).value(), v00_);
  EXPECT_EQ(xd.get_vector(1).value(), v01_);

  // To be deprecated: get_value() wraps a VectorBlock around the value.
  EXPECT_EQ(xd.get_vector(0).get_value(), v00_);
  EXPECT_EQ(xd.get_vector(1).get_value(), v01_);
}

TEST_F(DiscreteValuesTest, UnownedState) {
  DiscreteValues<double> xd(
      std::vector<BasicVector<double>*>{data_[0].get(), data_[1].get()});
  EXPECT_EQ(xd.get_vector(0).value(), v00_);
  EXPECT_EQ(xd.get_vector(1).value(), v01_);

  // To be deprecated: get_value() wraps a VectorBlock around the value.
  EXPECT_EQ(xd.get_vector(0).get_value(), v00_);
  EXPECT_EQ(xd.get_vector(1).get_value(), v01_);
}

TEST_F(DiscreteValuesTest, AppendGroup) {
  DiscreteValues<double> xd(std::move(data_));
  ASSERT_EQ(xd.num_groups(), 2);
  EXPECT_EQ(xd.get_vector(0).value(), v00_);
  EXPECT_EQ(xd.get_vector(1).value(), v01_);
  const int group_num = xd.AppendGroup(std::make_unique<MyVector3d>(v11_));
  EXPECT_EQ(group_num, 2);
  ASSERT_EQ(xd.num_groups(), 3);

  // Check that we didn't break previous values.
  EXPECT_EQ(xd.get_vector(0).value(), v00_);
  EXPECT_EQ(xd.get_vector(1).value(), v01_);

  // Check that new value and type are preserved.
  EXPECT_EQ(xd.get_vector(2).value(), v11_);
  EXPECT_TRUE(is_dynamic_castable<MyVector3d>(&xd.get_vector(2)));
}

TEST_F(DiscreteValuesTest, NoNullsAllowed) {
  // Unowned.
  EXPECT_THROW(DiscreteValues<double>(
      std::vector<BasicVector<double>*>{nullptr, data_[1].get()}),
               std::logic_error);
  // Owned.
  data_.push_back(nullptr);
  EXPECT_THROW(DiscreteValues<double>(std::move(data_)), std::logic_error);
}

// Clone should be deep, even for unowned data.
TEST_F(DiscreteValuesTest, Clone) {
  // Create a DiscreteValues object with unowned contents.
  DiscreteValues<double> xd(
      std::vector<BasicVector<double>*>{data_[0].get(), data_[1].get()});
  xd.set_system_id(internal::SystemId::get_new_id());
  std::unique_ptr<DiscreteValues<double>> clone = xd.Clone();

  // First check that the clone has the original values.
  EXPECT_EQ(clone->get_system_id(), xd.get_system_id());
  EXPECT_EQ(clone->get_vector(0).value(), v00_);
  EXPECT_EQ(clone->get_vector(1).value(), v01_);

  // Set the clone's new values.
  Eigen::Vector2d new0(9., 10.), new1(1.25, 2.5);
  clone->get_mutable_vector(0).set_value(new0);
  clone->get_mutable_vector(1).set_value(new1);

  // Check that the clone changed correctly.
  EXPECT_EQ(clone->get_vector(0).value(), new0);
  EXPECT_EQ(clone->get_vector(1).value(), new1);

  // Check that the original remains unchanged.
  EXPECT_EQ(xd.get_vector(0).value(), v00_);
  EXPECT_EQ(xd.get_vector(1).value(), v01_);
}

TEST_F(DiscreteValuesTest, SetFrom) {
  DiscreteValues<double> dut_double(
      BasicVector<double>::Make({1.0, 2.0}));
  DiscreteValues<AutoDiffXd> dut_autodiff(
      BasicVector<AutoDiffXd>::Make({3.0, 4.0}));
  DiscreteValues<symbolic::Expression> dut_symbolic(
      BasicVector<symbolic::Expression>::Make({5.0, 6.0}));

  // Check DiscreteValues<AutoDiff>::SetFrom<double>.
  dut_autodiff.SetFrom(dut_double);
  EXPECT_EQ(dut_autodiff.get_vector(0)[0].value(), 1.0);
  EXPECT_EQ(dut_autodiff.get_vector(0)[0].derivatives().size(), 0);
  EXPECT_EQ(dut_autodiff.get_vector(0)[1].value(), 2.0);
  EXPECT_EQ(dut_autodiff.get_vector(0)[1].derivatives().size(), 0);

  // Check DiscreteValues<Expression>::SetFrom<double>.
  dut_symbolic.SetFrom(dut_double);
  EXPECT_EQ(dut_symbolic.get_vector(0)[0], 1.0);
  EXPECT_EQ(dut_symbolic.get_vector(0)[1], 2.0);

  // Check DiscreteValues<double>::SetFrom<AutoDiff>.
  dut_autodiff.get_mutable_vector(0)[0] = 7.0;
  dut_autodiff.get_mutable_vector(0)[1] = 8.0;
  dut_double.SetFrom(dut_autodiff);
  EXPECT_EQ(dut_double.get_vector(0)[0], 7.0);
  EXPECT_EQ(dut_double.get_vector(0)[1], 8.0);

  // Check DiscreteValues<double>::SetFrom<Expression>.
  dut_symbolic.get_mutable_vector(0)[0] = 9.0;
  dut_symbolic.get_mutable_vector(0)[1] = 10.0;
  dut_double.SetFrom(dut_symbolic);
  EXPECT_EQ(dut_double.get_vector(0)[0], 9.0);
  EXPECT_EQ(dut_double.get_vector(0)[1], 10.0);

  // If there was an unbound variable, we get an exception.
  dut_symbolic.get_mutable_vector(0)[0] = symbolic::Variable("x");
  DRAKE_EXPECT_THROWS_MESSAGE(
      dut_double.SetFrom(dut_symbolic),
      ".*variable x.*\n*");
}

// Tests that the convenience accessors for a DiscreteValues that contains
// just one group work as documented.
GTEST_TEST(DiscreteValuesSingleGroupTest, ConvenienceSugar) {
  DiscreteValues<double> xd(BasicVector<double>::Make({42.0, 43.0}));
  EXPECT_EQ(2, xd.size());
  EXPECT_EQ(42.0, xd[0]);
  EXPECT_EQ(43.0, xd[1]);
  xd[0] = 100.0;
  EXPECT_EQ(100.0, xd.get_vector()[0]);
  xd.get_mutable_vector()[1] = 1000.0;
  EXPECT_EQ(1000.0, xd[1]);

  Eigen::Vector2d vec{44., 45.}, vec2{46., 47.};
  xd.set_value(vec);
  EXPECT_TRUE(CompareMatrices(xd.value(), vec));
  xd.get_mutable_value() = vec2;
  EXPECT_TRUE(CompareMatrices(xd.value(), vec2));

  DRAKE_EXPECT_THROWS_MESSAGE(xd.set_value(Vector3d::Ones()), ".*size.*");
}

// Tests that the convenience accessors for a DiscreteValues that contains
// no groups work as documented.
GTEST_TEST(DiscreteValuesSingleGroupTest, ConvenienceSugarEmpty) {
  static constexpr char expected_pattern[] = ".*exactly one group.*";
  DiscreteValues<double> xd;
  DRAKE_EXPECT_THROWS_MESSAGE(xd.size(), std::logic_error, expected_pattern);
  DRAKE_EXPECT_THROWS_MESSAGE(xd[0], std::logic_error, expected_pattern);
  DRAKE_EXPECT_THROWS_MESSAGE(xd.get_vector(), std::logic_error,
                              expected_pattern);
  DRAKE_EXPECT_THROWS_MESSAGE(xd.get_mutable_vector(), std::logic_error,
                              expected_pattern);
  DRAKE_EXPECT_THROWS_MESSAGE(xd.set_value(VectorXd()), expected_pattern);
  DRAKE_EXPECT_THROWS_MESSAGE(xd.value(), expected_pattern);
  DRAKE_EXPECT_THROWS_MESSAGE(xd.get_mutable_value(), expected_pattern);
}

// Tests that the convenience accessors for a DiscreteValues that contains
// multiple groups work as documented.
TEST_F(DiscreteValuesTest, ConvenienceSugarMultiple) {
  static constexpr char expected_pattern[] = ".*exactly one group.*";
  DiscreteValues<double> xd(std::move(data_));
  DRAKE_EXPECT_THROWS_MESSAGE(xd.size(), std::logic_error, expected_pattern);
  DRAKE_EXPECT_THROWS_MESSAGE(xd[0], std::logic_error, expected_pattern);
  DRAKE_EXPECT_THROWS_MESSAGE(xd.get_vector(), std::logic_error,
                              expected_pattern);
  DRAKE_EXPECT_THROWS_MESSAGE(xd.get_mutable_vector(), std::logic_error,
                              expected_pattern);

  Eigen::Vector2d vec{44., 45.}, vec2{46., 47.};
  xd.set_value(1, vec);
  EXPECT_TRUE(CompareMatrices(xd.value(1), vec));
  xd.get_mutable_value(1) = vec2;
  EXPECT_TRUE(CompareMatrices(xd.value(1), vec2));

  DRAKE_EXPECT_THROWS_MESSAGE(xd.set_value(1, Vector3d::Ones()), ".*size.*");
}

// For DiagramDiscreteValues we want to check that we can build a tree of
// unowned DiscreteValues but then Clone() produces an identical tree of
// owned DiscreteValues, with underlying BasicVector types preserved.
// Below b=basic vector, m=myvector.
TEST_F(DiscreteValuesTest, DiagramDiscreteValues) {
  DiscreteValues<double> dv0(std::move(data_));   // 2 groups size (b2,m2)
  DiscreteValues<double> dv1(std::move(data1_));  // 3 groups size (b3,m3,m2)
  auto dv2ptr = dv0.Clone(), dv3ptr = dv1.Clone();
  DiscreteValues<double>& dv2 = *dv2ptr;  // 2 groups size (b2,m2)
  DiscreteValues<double>& dv3 = *dv3ptr;  // 3 groups size (b3,m3,m2)
  EXPECT_EQ(dv0.num_groups(), 2);
  EXPECT_EQ(dv1.num_groups(), 3);
  EXPECT_EQ(dv2.num_groups(), 2);
  EXPECT_EQ(dv3.num_groups(), 3);

  //                     root
  //                    ╱    ╲       Flattened: root=10 groups
  //                  dv3   ddv1                ddv1=7 groups
  //                       ╱  |  ╲
  //                     dv0 dv1 dv2
  std::vector<DiscreteValues<double>*> unowned1{&dv0, &dv1, &dv2};
  DiagramDiscreteValues<double> ddv1(unowned1);
  std::vector<DiscreteValues<double>*> unowned2{&dv3, &ddv1};
  DiagramDiscreteValues<double> root(unowned2);

  EXPECT_EQ(ddv1.num_subdiscretes(), 3);
  EXPECT_EQ(ddv1.num_groups(), 7);
  EXPECT_EQ(ddv1.get_vector(0).size(), 2);  // dv0[0]
  EXPECT_EQ(ddv1.get_vector(3).size(), 3);  // dv1[1]
  EXPECT_EQ(ddv1.get_vector(6).size(), 2);  // dv2[1]
  EXPECT_EQ(root.num_subdiscretes(), 2);
  EXPECT_EQ(root.num_groups(), 10);
  EXPECT_EQ(root.get_subdiscrete(SubsystemIndex(0)).num_groups(), 3);
  EXPECT_EQ(root.get_subdiscrete(SubsystemIndex(1)).num_groups(), 7);
  EXPECT_EQ(root.get_vector(1).size(), 3);  // dv3[1]
  EXPECT_EQ(root.get_vector(3).size(), 2);  // dv0[0]
  EXPECT_EQ(root.get_vector(6).size(), 3);  // dv1[1]
  EXPECT_EQ(root.get_vector(9).size(), 2);  // dv2[1]

  // Check some of the vector types.
  EXPECT_FALSE(is_dynamic_castable<MyVector3d>(&root.get_vector(0)));  // dv3[0]
  EXPECT_TRUE(is_dynamic_castable<MyVector3d>(&root.get_vector(1)));   // dv3[1]
  EXPECT_TRUE(is_dynamic_castable<MyVector2d>(&root.get_vector(2)));   // dv3[2]
  EXPECT_TRUE(is_dynamic_castable<MyVector3d>(&root.get_vector(6)));   // dv1[1]

  // This is the shadowed Clone() method that preserves the concrete type.
  auto root2_ptr = root.Clone();
  DiagramDiscreteValues<double>& root2 = *root2_ptr;
  // Should have same structure.
  EXPECT_EQ(root2.num_subdiscretes(), root.num_subdiscretes());
  EXPECT_EQ(root2.num_groups(), root.num_groups());
  EXPECT_EQ(root2.get_subdiscrete(SubsystemIndex(0)).num_groups(),
            root.get_subdiscrete(SubsystemIndex(0)).num_groups());
  EXPECT_EQ(root2.get_subdiscrete(SubsystemIndex(1)).num_groups(),
            root.get_subdiscrete(SubsystemIndex(1)).num_groups());
  // But different owned BasicVectors, with the same values.
  for (int i = 0; i < root.num_groups(); ++i) {
    EXPECT_NE(&root.get_vector(i), &root2.get_vector(i));
    EXPECT_EQ(root2.get_vector(i).value(), root.get_vector(i).value());
  }

  // Check that the underlying vector types were preserved by Clone().
  EXPECT_FALSE(
      is_dynamic_castable<MyVector3d>(&root2.get_vector(0)));          // dv3[0]
  EXPECT_TRUE(is_dynamic_castable<MyVector3d>(&root2.get_vector(1)));  // dv3[1]
  EXPECT_TRUE(is_dynamic_castable<MyVector2d>(&root2.get_vector(2)));  // dv3[2]
  EXPECT_TRUE(is_dynamic_castable<MyVector3d>(&root2.get_vector(6)));  // dv1[1]

  // To make sure we didn't go too far with ownership, check that writing
  // through root2 changes what is seen by its subdiscretes.
  EXPECT_EQ(root2.get_vector(6).value()[1], -2.5);
  const DiscreteValues<double>& root2_ddv1 =
      root2.get_subdiscrete(SubsystemIndex(1));
  EXPECT_EQ(root2_ddv1.get_vector(3).value()[1], -2.5);
  root2.get_mutable_vector(6).get_mutable_value()[1] = 19.;
  EXPECT_EQ(root2.get_vector(6).value()[1], 19.);
  EXPECT_EQ(root2_ddv1.get_vector(3).value()[1], 19.);
  EXPECT_EQ(ddv1.get_vector(3).value()[1], -2.5);  // sanity check
}

}  // namespace
}  // namespace systems
}  // namespace drake
