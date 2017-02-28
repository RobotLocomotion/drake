#include "gtest/gtest.h"

#include "drake/common/eigen_types.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"
#include "drake/multibody/multibody_tree/rigid_body.h"

#include <memory>
#include <string>
#include <sstream>

namespace drake {
namespace multibody {
namespace {

using std::make_unique;
using std::unique_ptr;

template <class IndexType>
void RunMultibodyIndexTests() {
  // Construction from an int.
  {
    IndexType index(0);
    EXPECT_EQ(index, 0);  // This also tests operator==(int).
    EXPECT_TRUE(index.is_valid());  // Tests is_valid().
    index.invalidate();  // Tests invalidate().
    EXPECT_FALSE(index.is_valid());
    EXPECT_TRUE(index.is_invalid());  // Tests is_invalid().
  }

  // Default Construction and validity.
  {
    IndexType invalid_index;
    EXPECT_TRUE(invalid_index.is_invalid());
    EXPECT_FALSE(invalid_index.is_valid());
    EXPECT_EQ(invalid_index, IndexType::Invalid());  // Tests Invalid().
    IndexType valid_index(1);
    EXPECT_TRUE(valid_index.is_valid());
    valid_index = IndexType::Invalid();  // Tests IndexType::Invalid().
    EXPECT_TRUE(valid_index.is_invalid());
  }

  // Conversion operator.
  {
    IndexType index(4);
    int four = index;
    EXPECT_EQ(four, index);
  }

  // Comparison operators.
  {
    IndexType index1(5);
    IndexType index2(5);
    IndexType index3(7);
    EXPECT_EQ(index1, index2);  // operator==
    EXPECT_NE(index1, index3);  // operator!=
    EXPECT_LT(index1, index3);  // operator<
    EXPECT_LE(index1, index2);  // operator<=
    EXPECT_GT(index3, index1);  // operator>
    EXPECT_GE(index2, index1);  // operator>=
  }

  // Prefix increment.
  {
    IndexType index(8);
    EXPECT_EQ(index, IndexType(8));
    IndexType index_plus = ++index;
    EXPECT_EQ(index, IndexType(9));
    EXPECT_EQ(index_plus, IndexType(9));
  }

  // Postfix increment.
  {
    IndexType index(8);
    EXPECT_EQ(index, IndexType(8));
    IndexType index_plus = index++;
    EXPECT_EQ(index, IndexType(9));
    EXPECT_EQ(index_plus, IndexType(8));
  }

  // Prefix decrement.
  {
    IndexType index(8);
    EXPECT_EQ(index, IndexType(8));
    IndexType index_minus = --index;
    EXPECT_EQ(index, IndexType(7));
    EXPECT_EQ(index_minus, IndexType(7));
  }

  // Postfix decrement.
  {
    IndexType index(8);
    EXPECT_EQ(index, IndexType(8));
    IndexType index_minus = index--;
    EXPECT_EQ(index, IndexType(7));
    EXPECT_EQ(index_minus, IndexType(8));
  }

  // Addition assignment.
  {
    IndexType index(8);
    IndexType index_plus_seven = index += 7;
    EXPECT_EQ(index, IndexType(15));
    EXPECT_EQ(index_plus_seven, IndexType(15));
    EXPECT_EQ(index, index_plus_seven);
  }

  // Subtraction assignment.
  {
    IndexType index(8);
    IndexType index_plus_seven = index -= 7;
    EXPECT_EQ(index, IndexType(1));
    EXPECT_EQ(index_plus_seven, IndexType(1));
    EXPECT_EQ(index, index_plus_seven);
  }

  // Stream insertion operator.
  {
    IndexType index(8);
    std::stringstream stream;
    stream << index;
    EXPECT_EQ(stream.str(), "8");
    index.invalidate();
    stream.str(std::string());  // Clear contents.
    stream << index;
    EXPECT_EQ(stream.str(), "Invalid");
  }
}

GTEST_TEST(MultibodyTreeIndexes, FrameIndex) {
  RunMultibodyIndexTests<FrameIndex>();
}

GTEST_TEST(MultibodyTreeIndexes, BodyIndex) {
  RunMultibodyIndexTests<BodyIndex>();
}

GTEST_TEST(MultibodyTreeIndexes, MobilizerIndex) {
  RunMultibodyIndexTests<MobilizerIndex>();
}

// Test the basic MultibodyTree API to create and add bodies.
GTEST_TEST(MultibodyTree, AddBodies) {
  auto owned_model = std::make_unique<MultibodyTree<double>>();
  MultibodyTree<double>* model = owned_model.get();

  // Initially there is only one body, the world.
  EXPECT_EQ(model->get_num_bodies(), 1);

  // Retrieves the world body.
  const Body<double>& world_body = model->get_world_body();

  // Adds a new body to the world.
  const RigidBody<double>& pendulum = RigidBody<double>::Create(model);

  // Body identifiers are unique and are assigned by MultibodyTree in increasing
  // order starting with id = 0 (MultibodyTree::kWorldBodyId) for the "world"
  // body.
  EXPECT_EQ(world_body.get_id(), kWorldBodyId);
  EXPECT_EQ(pendulum.get_id(), BodyIndex(1));

  // Tests API to access bodies.
  EXPECT_EQ(model->get_body(BodyIndex(1)).get_id(), pendulum.get_id());
  EXPECT_EQ(model->get_mutable_body(BodyIndex(1))->get_id(), pendulum.get_id());

  // Rigid bodies have no generalized coordinates.
  EXPECT_EQ(pendulum.get_num_positions(), 0);
  EXPECT_EQ(pendulum.get_num_velocities(), 0);

  // Verify that no more bodies can be added to a MultibodyTree if it was
  // compiled already.
  model->Compile();
  EXPECT_THROW(RigidBody<double>::Create(model), std::runtime_error);
}

// Tests the correctness of MultibodyTreeElement checks to verify one or more
// elements belong to a given MultibodyTree.
GTEST_TEST(MultibodyTree, MultibodyTreeElementChecks) {
  auto model1 = std::make_unique<MultibodyTree<double>>();
  auto model2 = std::make_unique<MultibodyTree<double>>();

  const RigidBody<double>& body1 = RigidBody<double>::Create(model1.get());
  const RigidBody<double>& body2 = RigidBody<double>::Create(model2.get());

  // Tests that the created bodies indeed do have a parent MultibodyTree.
  EXPECT_NO_THROW(body1.HasParentTreeOrThrows());
  EXPECT_NO_THROW(body2.HasParentTreeOrThrows());

  // Tests the check to verify that two bodies belong to the same MultibodyTree.
  EXPECT_THROW(body1.HasSameParentTreeOrThrows(body2), std::runtime_error);
  EXPECT_NO_THROW(model1->get_world_body().HasSameParentTreeOrThrows(body1));
  EXPECT_NO_THROW(model2->get_world_body().HasSameParentTreeOrThrows(body2));
}

}  // namespace
}  // namespace multibody
}  // namespace drake
