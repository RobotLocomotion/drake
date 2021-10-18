#include "drake/common/yaml/yaml_node.h"

#include <fmt/format.h>
#include <fmt/ostream.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/test_utilities/limit_malloc.h"

namespace drake {
namespace yaml {
namespace internal {

// Make pretty googletest output.  (Per googletest, this cannot be defined in
// the anonymous namespace.)
static void PrintTo(const NodeType& node_type, std::ostream* os) {
  *os << "NodeType::k" << Node::GetTypeString(node_type);
}

namespace {

// Check that the tag constants exist.
GTEST_TEST(YamlNodeTest, TagConstants) {
  EXPECT_GT(Node::kTagFloat.size(), 0);
  EXPECT_GT(Node::kTagInt.size(), 0);
  EXPECT_GT(Node::kTagNull.size(), 0);
  EXPECT_GT(Node::kTagStr.size(), 0);
}

// Check the null-returning factory function.
GTEST_TEST(YamlNodeTest, MakeNull) {
  Node dut = Node::MakeNull();
  EXPECT_EQ(dut.GetType(), NodeType::kScalar);
  EXPECT_EQ(dut.GetScalar(), "null");
  EXPECT_EQ(dut.GetTag(), Node::kTagNull);

  EXPECT_TRUE(Node::MakeNull() == Node::MakeNull());
}

// Parameterize the remainder of the tests across the three possible types.
using Param = std::tuple<NodeType, std::string_view>;
class YamlNodeParamaterizedTest : public testing::TestWithParam<Param> {
 protected:
  // Returns the test suite's desired type.
  NodeType GetExpectedType() {
    return std::get<0>(GetParam());
  }

  // Returns the test suite's desired type string.
  std::string_view GetExpectedTypeString() {
    return std::get<1>(GetParam());
  }

  // Returns a new, empty Node with using the test suite's desired type.
  Node MakeEmptyDut() {
    switch (GetExpectedType()) {
      case NodeType::kScalar:   return Node::MakeScalar();
      case NodeType::kSequence: return Node::MakeSequence();
      case NodeType::kMapping:  return Node::MakeMapping();
    }
    DRAKE_UNREACHABLE();
  }

  // Return a new, non-empty Node with using the test suite's desired type.
  Node MakeNonEmptyDut() {
    switch (GetExpectedType()) {
      case NodeType::kScalar: {
        return Node::MakeScalar("foo");
      }
      case NodeType::kSequence: {
        Node result = Node::MakeSequence();
        result.Add(Node::MakeScalar("item"));
        return result;
      }
      case NodeType::kMapping: {
        Node result = Node::MakeMapping();
        result.Add("key", Node::MakeScalar("value"));
        return result;
      }
    }
    DRAKE_UNREACHABLE();
  }

  // Given a function name, returns the expected exception message in case the
  // runtime type of the Node is incorect.
  std::string GetExpectedCannot(std::string_view operation) {
    return fmt::format(
        ".*Cannot.*{}.*on a {}.*",
        operation, GetExpectedTypeString());
  }
};

// Check runtime type interrogation.
TEST_P(YamlNodeParamaterizedTest, GetType) {
  Node dut = MakeEmptyDut();
  EXPECT_EQ(dut.GetType(), GetExpectedType());
  EXPECT_EQ(dut.GetTypeString(), GetExpectedTypeString());
  EXPECT_EQ(dut.IsScalar(), GetExpectedType() == NodeType::kScalar);
  EXPECT_EQ(dut.IsSequence(), GetExpectedType() == NodeType::kSequence);
  EXPECT_EQ(dut.IsMapping(), GetExpectedType() == NodeType::kMapping);
}

// Check static type string conversion.
TEST_P(YamlNodeParamaterizedTest, StaticTypeString) {
  EXPECT_EQ(Node::GetTypeString(GetExpectedType()), GetExpectedTypeString());
}

// Check tag getting and setting.
TEST_P(YamlNodeParamaterizedTest, GetSetTag) {
  Node dut = MakeEmptyDut();
  EXPECT_EQ(dut.GetTag(), "");
  dut.SetTag("tag");
  EXPECT_EQ(dut.GetTag(), "tag");
}

// It is important for our YAML subsystem performance that the Node's move
// operations actually move the stored data, instead of copying it.
TEST_P(YamlNodeParamaterizedTest, EfficientMoveConstructor) {
  Node dut = MakeNonEmptyDut();

  auto guard = std::make_unique<test::LimitMalloc>();
  Node foo(std::move(dut));
  guard.reset();

  // The moved-to object must equal the full, original value.
  EXPECT_EQ(foo, MakeNonEmptyDut());

  // The moved-from object must be in some valid (but unspecified) state.
  EXPECT_FALSE(dut == MakeNonEmptyDut());
}

// Ditto per the prior test case.
TEST_P(YamlNodeParamaterizedTest, EfficientMoveAssignment) {
  Node dut = MakeNonEmptyDut();
  Node foo = Node::MakeNull();

  auto guard = std::make_unique<test::LimitMalloc>();
  foo = std::move(dut);
  guard.reset();

  // The moved-to object must equal the full, original value.
  EXPECT_EQ(foo, MakeNonEmptyDut());

  // The moved-from object must be in some valid (but unspecified) state.
  EXPECT_FALSE(dut == MakeNonEmptyDut());
}

// Check (non-)equality as affected by the stored type of empty nodes.
TEST_P(YamlNodeParamaterizedTest, EqualityPerType) {
  Node dut = MakeEmptyDut();
  EXPECT_EQ(dut == Node::MakeScalar(), dut.IsScalar());
  EXPECT_EQ(dut == Node::MakeSequence(), dut.IsSequence());
  EXPECT_EQ(dut == Node::MakeMapping(), dut.IsMapping());
}

// Check (non-)equality as affected by the tag.
TEST_P(YamlNodeParamaterizedTest, EqualityPerTag) {
  Node dut = MakeEmptyDut();
  Node dut2 = MakeEmptyDut();
  EXPECT_TRUE(dut == dut2);
  dut2.SetTag("tag");
  EXPECT_FALSE(dut == dut2);
}

// Check Scalar-specific operations.
TEST_P(YamlNodeParamaterizedTest, ScalarOps) {
  Node dut = MakeEmptyDut();
  if (dut.IsScalar()) {
    Node dut2 = MakeNonEmptyDut();
    EXPECT_FALSE(dut == dut2);
    EXPECT_EQ(dut2.GetScalar(), "foo");
  } else {
    DRAKE_EXPECT_THROWS_MESSAGE(
        dut.GetScalar(), GetExpectedCannot("GetScalar"));
  }
}

// Check Sequence-specific operations.
TEST_P(YamlNodeParamaterizedTest, SequenceOps) {
  Node dut = MakeEmptyDut();
  if (dut.IsSequence()) {
    EXPECT_TRUE(dut.GetSequence().empty());
    Node dut2 = MakeNonEmptyDut();
    EXPECT_FALSE(dut == dut2);
    ASSERT_EQ(dut2.GetSequence().size(), 1);
    EXPECT_EQ(dut2.GetSequence().front().GetScalar(), "item");
  } else {
    DRAKE_EXPECT_THROWS_MESSAGE(
        dut.GetSequence(), GetExpectedCannot("GetSequence"));
    DRAKE_EXPECT_THROWS_MESSAGE(
        dut.Add(Node::MakeNull()), GetExpectedCannot("Add"));
  }
}

// Check Mapping-specific operations.
TEST_P(YamlNodeParamaterizedTest, MappingOps) {
  Node dut = MakeEmptyDut();
  if (dut.IsMapping()) {
    EXPECT_TRUE(dut.GetMapping().empty());
    DRAKE_EXPECT_THROWS_MESSAGE(
        dut.Remove("quux"),
        ".*No such key.*'quux'.*");
    Node dut2 = MakeNonEmptyDut();
    EXPECT_FALSE(dut == dut2);
    ASSERT_EQ(dut2.GetMapping().size(), 1);
    EXPECT_EQ(dut2.GetMapping().begin()->first, "key");
    EXPECT_EQ(dut2.At("key").GetScalar(), "value");
    DRAKE_EXPECT_THROWS_MESSAGE(
        dut2.Add("key", Node::MakeScalar()),
        "Cannot .*Add.* duplicate key 'key'");
    dut2.Remove("key");
    EXPECT_TRUE(dut.GetMapping().empty());
  } else {
    DRAKE_EXPECT_THROWS_MESSAGE(
        dut.GetMapping(), GetExpectedCannot("GetMapping"));
    DRAKE_EXPECT_THROWS_MESSAGE(
        dut.Add("key", Node::MakeNull()), GetExpectedCannot("Add"));
    DRAKE_EXPECT_THROWS_MESSAGE(
        dut.At("key"), GetExpectedCannot("At"));
    DRAKE_EXPECT_THROWS_MESSAGE(
        dut.Remove("key"), GetExpectedCannot("Remove"));
  }
}

// Helper to check visiting.
struct VisitorThatCopies {
  void operator()(const Node::ScalarData& data) {
    scalar = data.scalar;
  }
  void operator()(const Node::SequenceData& data) {
    sequence = data.sequence;
  }
  void operator()(const Node::MappingData& data) {
    mapping = data.mapping;
  }

  std::optional<std::string> scalar;
  std::optional<std::vector<Node>> sequence;
  std::optional<std::map<std::string, Node>> mapping;
};

// Check visiting.
TEST_P(YamlNodeParamaterizedTest, Visiting) {
  Node dut = MakeNonEmptyDut();
  VisitorThatCopies visitor;
  dut.Visit(visitor);
  switch (GetExpectedType()) {
    case NodeType::kScalar: {
      const std::string empty;
      EXPECT_EQ(visitor.scalar.value_or(empty), "foo");
      EXPECT_FALSE(visitor.sequence.has_value());
      EXPECT_FALSE(visitor.mapping.has_value());
      return;
    }
    case NodeType::kSequence: {
      const std::vector<Node> empty;
      ASSERT_EQ(visitor.sequence.value_or(empty).size(), 1);
      EXPECT_EQ(visitor.sequence->front().GetScalar(), "item");
      EXPECT_FALSE(visitor.scalar.has_value());
      EXPECT_FALSE(visitor.mapping.has_value());
      return;
    }
    case NodeType::kMapping: {
      const std::map<std::string, Node> empty;
      ASSERT_EQ(visitor.mapping.value_or(empty).size(), 1);
      EXPECT_EQ(visitor.mapping->begin()->first, "key");
      EXPECT_EQ(visitor.mapping->at("key").GetScalar(), "value");
      EXPECT_FALSE(visitor.scalar.has_value());
      EXPECT_FALSE(visitor.sequence.has_value());
      return;
    }
  }
  DRAKE_UNREACHABLE();
}

// Check debug printing using operator<<.
TEST_P(YamlNodeParamaterizedTest, ToString) {
  Node dut = MakeNonEmptyDut();

  // Confirm that the code is being run by checking for a notable
  // substring within the printed result.
  std::string needle;
  switch (GetExpectedType()) {
    case NodeType::kScalar:   { needle = "foo";  break; }
    case NodeType::kSequence: { needle = "item"; break; }
    case NodeType::kMapping:  { needle = "key";  break; }
  }
  EXPECT_THAT(fmt::format("{}", dut), testing::HasSubstr(needle));

  // Confirm that tags are represented somehow.
  dut.SetTag("MyTag");
  EXPECT_THAT(fmt::format("{}", dut), testing::HasSubstr("MyTag"));
}

INSTANTIATE_TEST_SUITE_P(Suite, YamlNodeParamaterizedTest, testing::Values(
    Param(NodeType::kScalar, "Scalar"),
    Param(NodeType::kSequence, "Sequence"),
    Param(NodeType::kMapping, "Mapping")));

}  // namespace
}  // namespace internal
}  // namespace yaml
}  // namespace drake
