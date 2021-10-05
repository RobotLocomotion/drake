#include "drake/common/yaml/yaml_node.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace yaml {
namespace internal {
namespace {

// Boilerplate for std::visit.
template<class... Ts> struct overloaded : Ts... { using Ts::operator()...; };
template<class... Ts> overloaded(Ts...) -> overloaded<Ts...>;

// Check the default constructor.
GTEST_TEST(YamlNodeTest, DefaultCtor) {
  Node dut;
  EXPECT_EQ(dut.GetType(), NodeType::kScalar);
  EXPECT_TRUE(dut.IsScalar());
  EXPECT_TRUE(dut.IsEmptyScalar());
  EXPECT_EQ(dut.GetScalar(), "");
}

// Sanity check of copy/move/assign operators.  We don't need to test them
// exhaustively, because they are defaulted.
GTEST_TEST(YamlNodeTest, CopyMove) {
  Node dut = Node::MakeScalar("foo");

  Node foo(dut);
  EXPECT_EQ(dut.GetScalar(), "foo");
  EXPECT_EQ(foo.GetScalar(), "foo");

  Node bar(std::move(dut));
  EXPECT_EQ(dut.GetScalar(), "");
  EXPECT_EQ(foo.GetScalar(), "foo");
  EXPECT_EQ(bar.GetScalar(), "foo");
}

// Check all operations on a Scalar, except visiting and copy/move.
GTEST_TEST(YamlNodeTest, ScalarOperations) {
  Node dut = Node::MakeScalar("foo");

  EXPECT_EQ(dut.GetType(), NodeType::kScalar);
  EXPECT_EQ(dut.GetTypeString(), "Scalar");
  EXPECT_EQ(Node::GetTypeString(NodeType::kScalar), "Scalar");
  EXPECT_TRUE(dut.IsScalar());
  EXPECT_FALSE(dut.IsEmptyScalar());
  EXPECT_FALSE(dut.IsSequence());
  EXPECT_FALSE(dut.IsMap());

  EXPECT_EQ(dut.GetTag(), "");
  EXPECT_EQ(dut.GetScalar(), "foo");

  Node dut2 = Node::MakeScalar("foo");
  EXPECT_TRUE(dut == dut2);
  EXPECT_FALSE(dut == Node::MakeMap());
  EXPECT_FALSE(dut == Node::MakeScalar());

  dut2.SetTag("bar");
  EXPECT_EQ(dut2.GetTag(), "bar");
  EXPECT_FALSE(dut == dut2);

  Node dut3 = Node::MakeScalar("foo3");
  EXPECT_FALSE(dut == dut3);

  DRAKE_EXPECT_THROWS_MESSAGE(
      dut.GetSequence(),
      ".*Cannot.*GetSequence.*on a Scalar.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      dut.GetMap(),
      ".*Cannot.*GetMap.*on a Scalar.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      dut.Add(Node{}),
      ".*Cannot.*Add.*on a Scalar.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      dut.Add("key", Node{}),
      ".*Cannot.*Add.*on a Scalar.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      dut.At("key"),
      ".*Cannot.*At.*on a Scalar.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      dut.Remove("key"),
      ".*Cannot.*Remove.*on a Scalar.*");
}

// Check all operations on a Sequence, except visiting and copy/move.
GTEST_TEST(YamlNodeTest, SequenceOperations) {
  Node dut = Node::MakeSequence();

  EXPECT_EQ(dut.GetType(), NodeType::kSequence);
  EXPECT_EQ(dut.GetTypeString(), "Sequence");
  EXPECT_EQ(Node::GetTypeString(NodeType::kSequence), "Sequence");
  EXPECT_FALSE(dut.IsScalar());
  EXPECT_FALSE(dut.IsEmptyScalar());
  EXPECT_TRUE(dut.IsSequence());
  EXPECT_FALSE(dut.IsMap());

  EXPECT_EQ(dut.GetTag(), "");
  EXPECT_TRUE(dut.GetSequence().empty());

  Node dut2 = Node::MakeSequence();
  EXPECT_TRUE(dut == dut2);
  EXPECT_FALSE(dut == Node::MakeScalar());
  EXPECT_FALSE(dut == Node::MakeMap());

  dut2.SetTag("bar");
  EXPECT_EQ(dut2.GetTag(), "bar");
  EXPECT_FALSE(dut == dut2);

  Node dut3 = Node::MakeSequence();
  dut3.Add(Node::MakeScalar("foo"));
  EXPECT_FALSE(dut == dut3);

  ASSERT_EQ(dut3.GetSequence().size(), 1);
  EXPECT_EQ(dut3.GetSequence().front().GetScalar(), "foo");

  DRAKE_EXPECT_THROWS_MESSAGE(
      dut.GetScalar(),
      ".*Cannot.*GetScalar.*on a Sequence.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      dut.GetMap(),
      ".*Cannot.*GetMap.*on a Sequence.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      dut.Add("key", Node{}),
      ".*Cannot.*Add.*on a Sequence.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      dut.At("key"),
      ".*Cannot.*At.*on a Sequence.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      dut.Remove("key"),
      ".*Cannot.*Remove.*on a Sequence.*");
}

// Check all operations on a Map, except visiting and copy/move.
GTEST_TEST(YamlNodeTest, MapOperations) {
  Node dut = Node::MakeMap();

  EXPECT_EQ(dut.GetType(), NodeType::kMap);
  EXPECT_EQ(dut.GetTypeString(), "Map");
  EXPECT_EQ(Node::GetTypeString(NodeType::kMap), "Map");
  EXPECT_FALSE(dut.IsScalar());
  EXPECT_FALSE(dut.IsEmptyScalar());
  EXPECT_FALSE(dut.IsSequence());
  EXPECT_TRUE(dut.IsMap());

  EXPECT_EQ(dut.GetTag(), "");
  EXPECT_TRUE(dut.GetMap().empty());

  Node dut2 = Node::MakeMap();
  EXPECT_TRUE(dut == dut2);
  EXPECT_FALSE(dut == Node::MakeScalar());
  EXPECT_FALSE(dut == Node::MakeSequence());

  dut2.SetTag("bar");
  EXPECT_EQ(dut2.GetTag(), "bar");
  EXPECT_FALSE(dut == dut2);

  Node dut3 = Node::MakeMap();
  dut3.Add("key", Node::MakeScalar("value"));
  EXPECT_FALSE(dut == dut3);

  ASSERT_EQ(dut3.GetMap().size(), 1);
  EXPECT_EQ(dut3.GetMap().begin()->first, "key");
  EXPECT_EQ(dut3.At("key").GetScalar(), "value");

  dut3.Remove("key");
  EXPECT_TRUE(dut.GetMap().empty());
  DRAKE_EXPECT_THROWS_MESSAGE(
      dut.Remove("foo"),
      ".*No such key.*foo.*");

  DRAKE_EXPECT_THROWS_MESSAGE(
      dut.GetScalar(),
      ".*Cannot.*GetScalar.*on a Map.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      dut.GetSequence(),
      ".*Cannot.*GetSequence.*on a Map.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      dut.Add(Node{}),
      ".*Cannot.*Add.*on a Map.*");
}

// Helper to check visiting.
struct VisitorThatCopies {
  void operator()(const Node::ScalarData& data) {
    scalar = data.scalar;
  }
  void operator()(const Node::SequenceData& data) {
    sequence = data.sequence;
  }
  void operator()(const Node::MapData& data) {
    map = data.map;
  }

  std::optional<std::string> scalar;
  std::optional<std::vector<Node>> sequence;
  std::optional<std::map<std::string, Node>> map;
};

// Check visiting for all three value types.
GTEST_TEST(YamlNodeTest, Visiting) {
  Node dut1 = Node::MakeScalar("foo");
  Node dut2 = Node::MakeSequence();
  Node dut3 = Node::MakeMap();
  dut2.Add(Node::MakeScalar("bar"));
  dut3.Add("key", Node::MakeScalar("value"));

  VisitorThatCopies visitor1;
  VisitorThatCopies visitor2;
  VisitorThatCopies visitor3;
  dut1.Visit(visitor1);
  dut2.Visit(visitor2);
  dut3.Visit(visitor3);

  EXPECT_EQ(visitor1.scalar, std::optional<std::string>{"foo"});
  ASSERT_EQ(visitor2.sequence.value_or(std::vector<Node>{}).size(), 1);
  EXPECT_EQ(visitor2.sequence->front().GetScalar(), "bar");
  ASSERT_EQ(visitor3.map.value_or(std::map<std::string, Node>{}).size(), 1);
  EXPECT_EQ(visitor3.map->begin()->first, "key");
  EXPECT_EQ(visitor3.map->at("key").GetScalar(), "value");

  EXPECT_FALSE(visitor1.sequence.has_value());
  EXPECT_FALSE(visitor1.map.has_value());
  EXPECT_FALSE(visitor2.scalar.has_value());
  EXPECT_FALSE(visitor2.map.has_value());
  EXPECT_FALSE(visitor3.scalar.has_value());
  EXPECT_FALSE(visitor3.sequence.has_value());
}

}  // namespace
}  // namespace internal
}  // namespace yaml
}  // namespace drake
