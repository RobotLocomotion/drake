#include "drake/common/yaml/yaml_node.h"

#include <stdexcept>
#include <type_traits>

#include <fmt/format.h>

#include "drake/common/drake_assert.h"
#include "drake/common/overloaded.h"

namespace drake {
namespace yaml {
namespace internal {
namespace {

// Converts a type T from our variant<> into a string name for errors.
template <typename T>
std::string_view GetNiceVariantName(const T&) {
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
  result.tag_ = JsonSchemaTagInfo{.value = JsonSchemaTag::kNull};
  return result;
}

NodeType Node::GetType() const {
  return std::visit<NodeType>(  // BR
      overloaded{
          [](const ScalarData&) {
            return NodeType::kScalar;
          },
          [](const SequenceData&) {
            return NodeType::kSequence;
          },
          [](const MappingData&) {
            return NodeType::kMapping;
          },
      },
      data_);
}

std::string_view Node::GetTypeString() const {
  return std::visit(
      [](auto&& data) {
        return GetNiceVariantName(data);
      },
      data_);
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
  // We need to compare the canonical form of a tag (i.e., its string).
  auto a_tag = a.GetTag();
  auto b_tag = b.GetTag();
  return std::tie(a_tag, a.data_, a.filename_, a.mark_) ==
         std::tie(b_tag, b.data_, b.filename_, b.mark_);
}

bool operator==(const Node::Mark& a, const Node::Mark& b) {
  return std::tie(a.line, a.column) == std::tie(b.line, b.column);
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

std::string_view Node::GetTag() const {
  return std::visit<std::string_view>(  // BR
      overloaded{
          [](const std::string& tag) {
            return std::string_view{tag};
          },
          [](const JsonSchemaTagInfo& info) {
            switch (info.value) {
              case JsonSchemaTag::kNull:
                return kTagNull;
              case JsonSchemaTag::kBool:
                return kTagBool;
              case JsonSchemaTag::kInt:
                return kTagInt;
              case JsonSchemaTag::kFloat:
                return kTagFloat;
              case JsonSchemaTag::kStr:
                return kTagStr;
            }
            DRAKE_UNREACHABLE();
          },
      },
      tag_);
}

bool Node::IsTagImportant() const {
  return std::visit<bool>(  // BR
      overloaded{
          [](const std::string&) {
            return false;
          },
          [](const JsonSchemaTagInfo& info) {
            return info.important;
          },
      },
      tag_);
}

void Node::SetTag(JsonSchemaTag tag, bool important) {
  tag_ = JsonSchemaTagInfo{.value = tag, .important = important};
}

void Node::SetTag(std::string tag) {
  if (tag.empty()) {
    tag_ = {};
  } else {
    tag_ = std::move(tag);
  }
}

void Node::SetFilename(std::optional<std::string> filename) {
  filename_ = std::move(filename);
}

const std::optional<std::string>& Node::GetFilename() const {
  return filename_;
}

void Node::SetMark(std::optional<Mark> mark) {
  mark_ = mark;
}

const std::optional<Node::Mark>& Node::GetMark() const {
  return mark_;
}

const std::string& Node::GetScalar() const {
  return *std::visit<const std::string*>(
      overloaded{
          [](const ScalarData& data) {
            return &data.scalar;
          },
          [](auto&& data) -> std::nullptr_t {
            throw std::logic_error(fmt::format("Cannot Node::GetScalar on a {}",
                                               GetNiceVariantName(data)));
          },
      },
      data_);
}

const std::vector<Node>& Node::GetSequence() const {
  return *std::visit<const std::vector<Node>*>(
      overloaded{
          [](const SequenceData& data) {
            return &data.sequence;
          },
          [](auto&& data) -> std::nullptr_t {
            throw std::logic_error(fmt::format(
                "Cannot Node::GetSequence on a {}", GetNiceVariantName(data)));
          },
      },
      data_);
}

void Node::Add(Node value) {
  return std::visit<void>(
      overloaded{
          [&value](SequenceData& data) {
            data.sequence.push_back(std::move(value));
          },
          [](auto&& data) {
            throw std::logic_error(fmt::format(
                "Cannot Node::Add(value) on a {}", GetNiceVariantName(data)));
          },
      },
      data_);
}

const string_map<Node>& Node::GetMapping() const {
  return *visit<const string_map<Node>*>(
      overloaded{
          [](const MappingData& data) {
            return &data.mapping;
          },
          [](auto&& data) -> std::nullptr_t {
            throw std::logic_error(fmt::format(
                "Cannot Node::GetMapping on a {}", GetNiceVariantName(data)));
          },
      },
      data_);
}

void Node::Add(std::string key, Node value) {
  return std::visit<void>(
      overloaded{
          [&key, &value](MappingData& data) {
            const auto result =
                data.mapping.insert({std::move(key), std::move(value)});
            const bool inserted = result.second;
            if (!inserted) {
              // Our 'key' argument is now empty (because it has been
              // moved-from), so for the error message we need to dig the
              // existing key out of the map.
              const std::string& old_key = result.first->first;
              throw std::logic_error(fmt::format(
                  "Cannot Node::Add(key, value) using duplicate key '{}'",
                  old_key));
            }
          },
          [](auto&& data) {
            throw std::logic_error(
                fmt::format("Cannot Node::Add(key, value) on a {}",
                            GetNiceVariantName(data)));
          },
      },
      data_);
}

Node& Node::At(std::string_view key) {
  return *std::visit<Node*>(
      overloaded{
          [key](MappingData& data) {
            return &data.mapping.at(std::string{key});
          },
          [](auto&& data) -> std::nullptr_t {
            throw std::logic_error(fmt::format("Cannot Node::At(key) on a {}",
                                               GetNiceVariantName(data)));
          },
      },
      data_);
}

void Node::Remove(std::string_view key) {
  return std::visit<void>(
      overloaded{
          [key](MappingData& data) {
            auto erased = data.mapping.erase(std::string{key});
            if (!erased) {
              throw std::logic_error(fmt::format(
                  "No such key '{}' during Node::Remove(key)", key));
            }
          },
          [](auto&& data) {
            throw std::logic_error(fmt::format(
                "Cannot Node::Remove(key) on a {}", GetNiceVariantName(data)));
          },
      },
      data_);
}

std::string to_string(const Node& node) {
  std::string result;
  if (!node.GetTag().empty()) {
    result.append(fmt::format("!<{}> ", node.GetTag()));
  }
  node.Visit(overloaded{
      [&](const Node::ScalarData& data) {
        result.append(fmt::format("\"{}\"", data.scalar));
      },
      [&](const Node::SequenceData& data) {
        result.append("[");
        bool first = true;
        for (const auto& child : data.sequence) {
          if (!first) {
            result.append(", ");
          }
          first = false;
          result.append(fmt::format("{}", child));
        }
        result.append("]");
      },
      [&](const Node::MappingData& data) {
        result.append("{");
        bool first = true;
        for (const auto& [key, child] : data.mapping) {
          if (!first) {
            result.append(", ");
          }
          first = false;
          result.append(fmt::format("\"{}\": {}", key, child));
        }
        result.append("}");
      },
  });
  return result;
}

}  // namespace internal
}  // namespace yaml
}  // namespace drake
