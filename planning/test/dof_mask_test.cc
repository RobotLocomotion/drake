#include "drake/planning/dof_mask.h"

#include <algorithm>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <fmt/format.h>
#include <fmt/ranges.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/planning/test/make_dummy_plant.h"

namespace drake {
namespace planning {
namespace {

using Eigen::Vector2d;
using Eigen::Vector3d;
using multibody::BodyIndex;
using multibody::default_model_instance;
using multibody::JointIndex;
using multibody::ModelInstanceIndex;

/* Exercise all of the constructors and evaluate the size-like APIs. */
GTEST_TEST(DofMaskTest, ConstructorsAndSize) {
  const DofMask dut1;
  EXPECT_EQ(dut1.count(), 0);
  EXPECT_EQ(dut1.size(), 0);

  const DofMask dut2(13, true);
  EXPECT_EQ(dut2.count(), 13);
  EXPECT_EQ(dut2.size(), 13);

  const DofMask dut3(14, false);
  EXPECT_EQ(dut3.count(), 0);
  EXPECT_EQ(dut3.size(), 14);

  const DofMask dut4({true, false, true, true, false});
  EXPECT_EQ(dut4.count(), 3);
  EXPECT_EQ(dut4.size(), 5);

  std::vector<bool> bits{true, true, false, true, true, false};
  const DofMask dut5(bits);
  EXPECT_EQ(dut5.count(), 4);
  EXPECT_EQ(dut5.size(), 6);
}

// In addition to testing move/copy semantics, this also provides testing for
// operator==.
GTEST_TEST(DofMaskTest, CopyMoveSemantics) {
  const DofMask dofs({true, false, true, false});

  // Copy constructor.
  DofMask copied(dofs);
  EXPECT_EQ(copied, dofs);

  // Move constructor.
  DofMask moved(std::move(copied));
  EXPECT_EQ(moved, dofs);
  EXPECT_NE(copied, dofs);

  // A moved-from object must be zero-sized (it's the only reasonable way for
  // it to maintain its invariants after donating is storage to the new object.)
  EXPECT_EQ(copied.size(), 0);
  EXPECT_EQ(copied.count(), 0);

  // Move assignment.
  copied = std::move(moved);
  EXPECT_EQ(copied, dofs);
  EXPECT_NE(moved, dofs);

  // A moved-from object must be zero-sized (it's the only reasonable way for
  // it to maintain its invariants after donating is storage to the new object.)
  EXPECT_EQ(moved.size(), 0);
  EXPECT_EQ(moved.count(), 0);

  // Copy assignment.
  moved = copied;
  EXPECT_EQ(copied, dofs);
  EXPECT_EQ(moved, dofs);
}

// Confirms the factories work.
GTEST_TEST(DofMaskTest, Factories) {
  auto [plant, _] = MakeDummyPlant();

  // Names defined by MakeDummyPlant.
  const std::string one_dof_model = "my_model";
  const std::string five_dof_model = "other_model";
  const ModelInstanceIndex one_dof_index =
      plant->GetModelInstanceByName(one_dof_model);
  const ModelInstanceIndex five_dof_index =
      plant->GetModelInstanceByName(five_dof_model);

  // Construct 1-dof mask from index.
  {
    const DofMask dut = DofMask::MakeFromModel(*plant, one_dof_index);
    EXPECT_EQ(dut.size(), plant->num_positions());
    EXPECT_EQ(dut.count(), 1);
  }

  // Construct 5-dof mask from index.
  {
    const DofMask dut = DofMask::MakeFromModel(*plant, five_dof_index);
    EXPECT_EQ(dut.size(), plant->num_positions());
    EXPECT_EQ(dut.count(), 5);
  }

  // Construct 1-dof mask from name.
  {
    const DofMask dut = DofMask::MakeFromModel(*plant, one_dof_model);
    EXPECT_EQ(dut.size(), plant->num_positions());
    EXPECT_EQ(dut.count(), 1);
  }

  // Construct 5-dof mask from name.
  {
    const DofMask dut = DofMask::MakeFromModel(*plant, five_dof_model);
    EXPECT_EQ(dut.size(), plant->num_positions());
    EXPECT_EQ(dut.count(), 5);
  }

  // Incompatible plant causes factories to throw.
  {
    auto bad_plant = MakeDummyFloatingBodyPlant();
    const ModelInstanceIndex model = default_model_instance();
    DRAKE_EXPECT_THROWS_MESSAGE(
        DofMask::MakeFromModel(*bad_plant, model),
        ".*ith velocity must be the time derivative of the ith position.*");
    DRAKE_EXPECT_THROWS_MESSAGE(
        DofMask::MakeFromModel(*bad_plant,
                               bad_plant->GetModelInstanceName(model)),
        ".*ith velocity must be the time derivative of the ith position.*");
  }
}

GTEST_TEST(DofMaskTest, GetJoints) {
  auto [plant, _] = MakeDummyPlant();

  // Model instance with single joint.
  {
    const ModelInstanceIndex model = plant->GetModelInstanceByName("my_model");
    const DofMask dofs = DofMask::MakeFromModel(*plant, model);
    std::vector<JointIndex> joints = dofs.GetJoints(*plant);
    const std::vector<JointIndex> expected_joints{
        plant->GetJointByName("joint_b").index()};
    EXPECT_EQ(joints, expected_joints);
  }

  // Model instance with multiple joints.
  {
    const ModelInstanceIndex model =
        plant->GetModelInstanceByName("other_model");
    const DofMask dofs = DofMask::MakeFromModel(*plant, model);
    std::vector<JointIndex> joints = dofs.GetJoints(*plant);
    const std::vector<JointIndex> expected_joints{
        plant->GetJointByName("joint_c").index(),
        plant->GetJointByName("joint_d").index(),
        plant->GetJointByName("joint_e").index(),
        plant->GetJointByName("joint_f").index(),
        plant->GetJointByName("joint_g").index()};
    EXPECT_EQ(joints, expected_joints);
  }
}

GTEST_TEST(DofMaskTest, ElementAccess) {
  const std::vector<bool> bits{true, false, false, true, true};
  const DofMask dut(bits);

  for (int i = 0; i < dut.size(); ++i) {
    EXPECT_EQ(dut[i], bits[i]);
  }
}

GTEST_TEST(DofMaskTest, ToString) {
  const std::vector<bool> bits{true, false, false, true, true};
  const std::vector<int> int_bits(bits.begin(), bits.end());
  const DofMask dut(bits);
  // The string representation of DofMask should match that of vector<bool>.
  EXPECT_EQ(dut.to_string(), fmt::to_string(int_bits));
}

GTEST_TEST(DofMaskTest, Complement) {
  const DofMask d1({true, false, false});
  const DofMask expected({false, true, true});

  EXPECT_EQ(d1.Complement(), expected);
}

GTEST_TEST(DofMaskTest, Union) {
  const DofMask d1({true, false, false});
  const DofMask d2({false, false, true});
  const DofMask expected({true, false, true});

  EXPECT_EQ(d1.Union(d2), expected);
}

GTEST_TEST(DofMaskTest, Intersect) {
  const DofMask d1({true, true, false});
  const DofMask d2({false, true, true});
  const DofMask expected({false, true, false});

  EXPECT_EQ(d1.Intersect(d2), expected);
}

GTEST_TEST(DofMaskTest, Subtract) {
  const DofMask d1({true, true, false});
  const DofMask d2({false, true, true});
  const DofMask expected({true, false, false});

  EXPECT_EQ(d1.Subtract(d2), expected);
}

GTEST_TEST(DofMaskTest, GetFromArrayWithReturn) {
  const DofMask dofs{true, false, true, false, true};
  const auto full = (Eigen::VectorXd(5) << 1, 2, 3, 4, 5).finished();

  Eigen::VectorXd result = dofs.GetFromArray(full);
  ASSERT_EQ(result.size(), 3);
  EXPECT_EQ(result(0), 1);
  EXPECT_EQ(result(1), 3);
  EXPECT_EQ(result(2), 5);

  // To satisfy the [[nodiscard]] on GetFromArray(), we'll evaluate it in a
  // lambda function; we don't care about the return value because we expect
  // the invocation to throw.
  auto eval_get_from_array = [](const DofMask& mask, const auto& vector) {
    return mask.GetFromArray(vector);
  };
  // Using too few dofs throws.
  const DofMask too_few_dofs{true, true};
  EXPECT_THROW(eval_get_from_array(too_few_dofs, full), std::exception);

  // Using too many dofs throws.
  const DofMask too_many_dofs{true, true, false, false, true, true};
  EXPECT_THROW(eval_get_from_array(too_many_dofs, full), std::exception);

  // Matrix column selection.
  Eigen::Matrix3d mat;
  // clang-format off
  mat <<
    1, 2, 3,
    4, 5, 6,
    7, 8, 9;
  const DofMask mat_dofs{true, false, true};
  Eigen::Matrix<double, 3, 2> mat_selected_expected;
  mat_selected_expected <<
    1, 3,
    4, 6,
    7, 9;
  // clang-format on
  const Eigen::MatrixXd mat_selected = mat_dofs.GetColumnsFromMatrix(mat);
  EXPECT_EQ(mat_selected, mat_selected_expected);
}

GTEST_TEST(DofMaskTest, GetFromArrayWithOutParameter) {
  const DofMask dofs{true, false, true};
  const Vector3d full(1.0, 2.0, 3.0);
  const Vector2d selected(1.0, 3.0);

  // Places the answer in an output parameter.
  Vector2d selected_out(0, 0);
  dofs.GetFromArray(full, &selected_out);
  EXPECT_EQ(selected_out, selected);

  // Matrix column selection.
  Eigen::Matrix3d mat;
  // clang-format off
  mat <<
    1, 2, 3,
    4, 5, 6,
    7, 8, 9;
  Eigen::Matrix<double, 3, 2> mat_selected_expected;
  mat_selected_expected <<
    1, 3,
    4, 6,
    7, 9;
  // clang-format on
  Eigen::MatrixXd mat_selected(3, 2);
  dofs.GetColumnsFromMatrix(mat, &mat_selected);
  EXPECT_EQ(mat_selected, mat_selected_expected);
}

GTEST_TEST(DofMaskTest, SetInArray) {
  // Writing into full vector.
  const DofMask dofs{true, false, true};
  const Vector2d dof_values(1.0, 3.0);
  Vector3d full_new;
  full_new.setZero();
  dofs.SetInArray(dof_values, &full_new);
  const Vector3d full_new_expected(1.0, 0.0, 3.0);
  EXPECT_EQ(full_new, full_new_expected);
}

GTEST_TEST(DofMaskTest, GetSelectedToFullIndex) {
  const DofMask dofs1{true, false, false, true, false};
  EXPECT_EQ(dofs1.GetSelectedToFullIndex(), std::vector<int>({0, 3}));

  const DofMask dofs2{false, false, false, false};
  EXPECT_TRUE(dofs2.GetSelectedToFullIndex().empty());
}

GTEST_TEST(DofMaskTest, GetFullToSelectedIndex) {
  const DofMask dofs1{true, false, false, true, false, true, false};
  EXPECT_EQ(dofs1.GetFullToSelectedIndex(),
            (std::vector<std::optional<int>>{0, std::nullopt, std::nullopt, 1,
                                             std::nullopt, 2, std::nullopt}));

  const DofMask dofs2{false, false, false, false};
  EXPECT_EQ(dofs2.GetFullToSelectedIndex(),
            std::vector<std::optional<int>>(4, std::nullopt));
}

}  // namespace
}  // namespace planning
}  // namespace drake
