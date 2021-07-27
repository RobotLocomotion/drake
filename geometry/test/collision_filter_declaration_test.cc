#include "drake/geometry/collision_filter_declaration.h"

#include <gtest/gtest.h>

#include "drake/geometry/geometry_ids.h"

namespace drake {
namespace geometry {

using std::vector;

class CollisionFilterDeclTester {
 public:
  using Statement = CollisionFilterDeclaration::Statement;
  using StatementOp = CollisionFilterDeclaration::StatementOp;

  static const vector<Statement>& statements(
      const CollisionFilterDeclaration& decl) {
    return decl.statements();
  }
};

class GeometrySetTester {
 public:
  static bool AreEqual(const GeometrySet& set_A, const GeometrySet& set_B) {
    return set_A.geometry_ids_ == set_B.geometry_ids_ &&
           set_A.frame_ids_ == set_B.frame_ids_;
  }
};

namespace {

using Tester = CollisionFilterDeclTester;

GTEST_TEST(CollisionFilterDeclTest, DefaultConstructor) {
  CollisionFilterDeclaration decl;
  EXPECT_EQ(Tester::statements(decl).size(), 0);
}

// Confirms that the parameters of the AllowBetween method are properly
// encoded in the last statement.
GTEST_TEST(CollisionFilterDeclTest, AllowBetween) {
  const GeometrySet set_A({GeometryId::get_new_id(), GeometryId::get_new_id()});
  const GeometrySet set_B({FrameId::get_new_id(), FrameId::get_new_id()});

  CollisionFilterDeclaration decl;
  decl.AllowBetween(set_A, set_B);

  const auto& statements = Tester::statements(decl);
  EXPECT_EQ(statements.size(), 1);
  EXPECT_EQ(statements[0].operation, Tester::StatementOp::kAllowBetween);
  EXPECT_TRUE(GeometrySetTester::AreEqual(statements[0].set_A, set_A));
  EXPECT_TRUE(GeometrySetTester::AreEqual(statements[0].set_B, set_B));
}

// Confirms that the parameters of the AllowWithin method are properly
// encoded in the last statement.
GTEST_TEST(CollisionFilterDeclTest, AllowWithin) {
  const GeometrySet set_A({GeometryId::get_new_id(), GeometryId::get_new_id()});
  const GeometrySet set_B({FrameId::get_new_id(), FrameId::get_new_id()});

  for (const auto& geo_set : {set_A, set_B}) {
    CollisionFilterDeclaration decl;
    decl.AllowWithin(geo_set);

    const auto& statements = Tester::statements(decl);
    EXPECT_EQ(statements.size(), 1);
    EXPECT_EQ(statements[0].operation, Tester::StatementOp::kAllowWithin);
    EXPECT_TRUE(GeometrySetTester::AreEqual(statements[0].set_A, geo_set));
    EXPECT_TRUE(
        GeometrySetTester::AreEqual(statements[0].set_B, GeometrySet()));
  }
}

// Confirms that the parameters of the ExcludeBetween method are properly
// encoded in the last statement.
GTEST_TEST(CollisionFilterDeclTest, ExcludeBetween) {
  const GeometrySet set_A({GeometryId::get_new_id(), GeometryId::get_new_id()});
  const GeometrySet set_B({FrameId::get_new_id(), FrameId::get_new_id()});

  CollisionFilterDeclaration decl;
  decl.ExcludeBetween(set_A, set_B);

  const auto& statements = Tester::statements(decl);
  EXPECT_EQ(statements.size(), 1);
  EXPECT_EQ(statements[0].operation, Tester::StatementOp::kExcludeBetween);
  EXPECT_TRUE(GeometrySetTester::AreEqual(statements[0].set_A, set_A));
  EXPECT_TRUE(GeometrySetTester::AreEqual(statements[0].set_B, set_B));
}

// Confirms that the parameters of the ExcludeWithin method are properly encoded
// in the last statement.
GTEST_TEST(CollisionFilterDeclTest, ExcludeWithin) {
  const GeometrySet set_1({GeometryId::get_new_id(), GeometryId::get_new_id()});
  const GeometrySet set_2({FrameId::get_new_id(), FrameId::get_new_id()});

  for (const auto& geo_set : {set_1, set_2}) {
    CollisionFilterDeclaration decl;
    decl.ExcludeWithin(geo_set);

    const auto& statements = Tester::statements(decl);
    EXPECT_EQ(statements.size(), 1);
    EXPECT_EQ(statements[0].operation, Tester::StatementOp::kExcludeWithin);
    EXPECT_TRUE(GeometrySetTester::AreEqual(statements[0].set_A, geo_set));
    EXPECT_TRUE(
        GeometrySetTester::AreEqual(statements[0].set_B, GeometrySet()));
  }
}

// A simple test that confirms the *mechanism* for chaining; each statement
// method returns a pointer to itself. This method should exercise *every*
// statement method; as they get added to the class, they should be added here.
GTEST_TEST(CollisionFilterDeclTest, ChainingStatements) {
  const GeometrySet set_A({GeometryId::get_new_id(), GeometryId::get_new_id()});
  CollisionFilterDeclaration decl;
  EXPECT_EQ(&decl.AllowBetween(set_A, set_A), &decl);
  EXPECT_EQ(&decl.AllowWithin(set_A), &decl);
  EXPECT_EQ(&decl.ExcludeBetween(set_A, set_A), &decl);
  EXPECT_EQ(&decl.ExcludeWithin(set_A), &decl);
}

// Confirms that the order of the statements given is preserved. We'll confirm
// that each statement method appends the expected data to the sequence of
// statements, leaving the previous statements untouched.
GTEST_TEST(CollisionFilterDeclTest, StatementOrder) {
  const GeometrySet set_1({GeometryId::get_new_id(), GeometryId::get_new_id()});
  const GeometrySet set_2({GeometryId::get_new_id(), GeometryId::get_new_id()});

  CollisionFilterDeclaration decl_original;
  decl_original.ExcludeWithin(set_1);
  // Validates the configuration of the *first* statement. It has the prereq
  // that the declaration contains at least one statement.
  auto validate_base = [&set_1](const CollisionFilterDeclaration& declaration) {
    const auto& s = Tester::statements(declaration)[0];
    EXPECT_EQ(s.operation, Tester::StatementOp::kExcludeWithin);
    EXPECT_TRUE(GeometrySetTester::AreEqual(s.set_A, set_1));
    EXPECT_TRUE(GeometrySetTester::AreEqual(s.set_B, GeometrySet{}));
  };

  // Confirm that the baseline declaration has the expected statement.
  EXPECT_GE(Tester::statements(decl_original).size(), 1);
  validate_base(decl_original);

  // Confirm that there's a second statement, and its parameters are the given
  // set of expectations (and the base statement is unchanged).
  auto validate = [&validate_base](
                      const Tester::Statement expected,
                      const CollisionFilterDeclaration& declaration) {
    validate_base(declaration);
    const auto& statements = Tester::statements(declaration);
    EXPECT_EQ(statements.size(), 2);
    validate_base(declaration);
    const auto& s = statements[1];
    EXPECT_EQ(s.operation, expected.operation);
    EXPECT_TRUE(GeometrySetTester::AreEqual(s.set_A, expected.set_A));
    EXPECT_TRUE(GeometrySetTester::AreEqual(s.set_B, expected.set_B));
  };

  // Exercise each of the statement methods and confirm that it is appended to
  // the end.
  {
    CollisionFilterDeclaration decl(decl_original);
    decl.AllowBetween(set_2, set_1);
    SCOPED_TRACE("AllowBetween");
    validate({Tester::StatementOp::kAllowBetween, set_2, set_1}, decl);
  }

  {
    CollisionFilterDeclaration decl(decl_original);
    decl.AllowWithin(set_2);
    SCOPED_TRACE("AllowWithin");
    validate({Tester::StatementOp::kAllowWithin, set_2, GeometrySet()}, decl);
  }

  {
    CollisionFilterDeclaration decl(decl_original);
    decl.ExcludeBetween(set_2, set_1);
    SCOPED_TRACE("ExcludeBetween");
    validate({Tester::StatementOp::kExcludeBetween, set_2, set_1}, decl);
  }

  {
    CollisionFilterDeclaration decl(decl_original);
    decl.ExcludeWithin(set_2);
    SCOPED_TRACE("ExcludeWithin");
    validate({Tester::StatementOp::kExcludeWithin, set_2, GeometrySet()}, decl);
  }
}

}  // namespace
}  // namespace geometry
}  // namespace drake
