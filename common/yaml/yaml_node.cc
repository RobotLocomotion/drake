#include "drake/common/yaml/yaml_node.h"

#include <stdexcept>
#include <type_traits>

#include <fmt/format.h>

#include "drake/common/drake_assert.h"

namespace drake {
namespace yaml {
namespace internal {
namespace {

// Boilerplate for std::visit.
template<class... Ts> struct overloaded : Ts... { using Ts::operator()...; };
template<class... Ts> overloaded(Ts...) -> overloaded<Ts...>;

// Converts a type T from our variant<> into a string name for errors.
template <typename T> std::string_view GetNiceVariantName(const T&) {
  if constexpr (std::is_same_v<T, Node::ScalarData>) {
    return "Scalar";
  } else if constexpr (std::is_same_v<T, Node::SequenceData>) {
    return "Sequence";
  } else if constexpr (std::is_same_v<T, Node::MappingData>) {
    return "Mapping";
  }
}

}  // namespace

Node::Node() = default;

Node Node::MakeScalar(std::string value) {
  Node result;
  result.data_ = ScalarData{std::move(value)};
  return result;
}

Node Node::MakeSequence() {
  Node result;
  result.data_ = SequenceData{};
  return result;
}

Node Node::MakeMapping() {
  Node result;
  result.data_ = MappingData{};
  return result;
}

Node Node::MakeNull() {
  Node result;
  result.data_ = ScalarData{"null"};
  result.tag_ = kTagNull;
  return result;
}

NodeType Node::GetType() const {
  return std::visit(overloaded{
    [](const ScalarData&) { return NodeType::kScalar; },
    [](const SequenceData&) { return NodeType::kSequence; },
    [](const MappingData&) { return NodeType::kMapping; },
  }, data_);
}

std::string_view Node::GetTypeString() const {
  return std::visit([](auto&& data) {
    return GetNiceVariantName(data);
  }, data_);
}

std::string_view Node::GetTypeString(NodeType type) {
  switch (type) {
    case NodeType::kScalar: {
      return GetNiceVariantName(ScalarData{});
    }
    case NodeType::kSequence: {
      return GetNiceVariantName(SequenceData{});
    }
    case NodeType::kMapping: {
      return GetNiceVariantName(MappingData{});
    }
  }
  DRAKE_UNREACHABLE();
}

bool Node::IsScalar() const {
  return std::holds_alternative<ScalarData>(data_);
}

bool Node::IsSequence() const {
  return std::holds_alternative<SequenceData>(data_);
}

bool Node::IsMapping() const {
  return std::holds_alternative<MappingData>(data_);
}

bool operator==(const Node& a, const Node& b) {
  return std::tie(a.tag_, a.data_) == std::tie(b.tag_, b.data_);
}

bool operator==(const Node::ScalarData& a, const Node::ScalarData& b) {
  return a.scalar == b.scalar;
}

bool operator==(const Node::SequenceData& a, const Node::SequenceData& b) {
  return a.sequence == b.sequence;
}

bool operator==(const Node::MappingData& a, const Node::MappingData& b) {
  return a.mapping == b.mapping;
}

const std::string& Node::GetTag() const {
  return tag_;
}

void Node::SetTag(std::string tag) {
  tag_ = std::move(tag);
}

const std::string& Node::GetScalar() const {
  return std::visit(overloaded{
    [](const ScalarData& data) -> const std::string& {
      return data.scalar;
    },
    [](auto&& data) -> const std::string& {
      throw std::logic_error(fmt::format(
          "Cannot Node::GetScalar on a {}", GetNiceVariantName(data)));
    },
  }, data_);
}

const std::vector<Node>& Node::GetSequence() const {
  return std::visit(overloaded{
    [](const SequenceData& data) -> const std::vector<Node>& {
      return data.sequence;
    },
    [](auto&& data) -> const std::vector<Node>& {
      throw std::logic_error(fmt::format(
          "Cannot Node::GetSequence on a {}", GetNiceVariantName(data)));
    },
  }, data_);
}

void Node::Add(Node value) {
  return std::visit(overloaded{
    [&value](SequenceData& data) -> void {
      data.sequence.push_back(std::move(value));
    },
    [](auto&& data) -> void {
      throw std::logic_error(fmt::format(
          "Cannot Node::Add(value) on a {}", GetNiceVariantName(data)));
    },
  }, data_);
}

const std::map<std::string, Node>& Node::GetMapping() const {
  return std::visit(overloaded{
    [](const MappingData& data) -> const std::map<std::string, Node>& {
      return data.mapping;
    },
    [](auto&& data) -> const std::map<std::string, Node>& {
      throw std::logic_error(fmt::format(
          "Cannot Node::GetMapping on a {}", GetNiceVariantName(data)));
    },
  }, data_);
}

void Node::Add(std::string key, Node value) {
  return std::visit(overloaded{
      [&key, &value](MappingData& data) -> void {
      const auto result = data.mapping.insert({
          std::move(key), std::move(value)});
      const bool inserted = result.second;
      if (!inserted) {
        // Our 'key' argument is now empty (because it has been moved-from), so
        // for the error message we need to dig the existing key out of the map.
        const std::string& old_key = result.first->first;
        throw std::logic_error(fmt::format(
            "Cannot Node::Add(key, value) usign duplicate key '{}'", old_key));
      }
    },
    [](auto&& data) -> void {
      throw std::logic_error(fmt::format(
          "Cannot Node::Add(key, value) on a {}", GetNiceVariantName(data)));
    },
  }, data_);
}

Node& Node::At(std::string_view key) {
  return std::visit(overloaded{
    [key](MappingData& data) -> Node& {
      return data.mapping.at(std::string{key});
    },
    [](auto&& data) -> Node& {
      throw std::logic_error(fmt::format(
          "Cannot Node::At(key) on a {}", GetNiceVariantName(data)));
    },
  }, data_);
}

void Node::Remove(std::string_view key) {
  return std::visit(overloaded{
    [key](MappingData& data) -> void {
      auto erased = data.mapping.erase(std::string{key});
      if (!erased) {
        throw std::logic_error(fmt::format(
            "No such key '{}' during Node::Remove(key)", key));
      }
    },
    [](auto&& data) -> void {
      throw std::logic_error(fmt::format(
          "Cannot Node::Remove(key) on a {}", GetNiceVariantName(data)));
    },
  }, data_);
}

std::ostream& operator<<(std::ostream& os, const Node& node) {
  if (!node.GetTag().empty()) {
    os << "!<" << node.GetTag() << "> ";
  }
  node.Visit(overloaded{
    [&](const Node::ScalarData& data) {
      os << '"' << data.scalar << '"';
    },
    [&](const Node::SequenceData& data) {
      os << "[";
      bool first = true;
      for (const auto& child : data.sequence) {
        if (!first) {
          os << ", ";
        }
        first = false;
        os << child;
      }
      os << "]";
    },
    [&](const Node::MappingData& data) {
      os << "{";
      bool first = true;
      for (const auto& [key, child] : data.mapping) {
        if (!first) {
          os << ", ";
        }
        first = false;
        os << '"' << key << '"' << ": " << child;
      }
      os << "}";
    },
  });
  return os;
}

}  // namespace internal
}  // namespace yaml
}  // namespace drake
