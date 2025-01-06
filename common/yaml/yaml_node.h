#pragma once

#include <map>
#include <optional>
#include <ostream>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/fmt_ostream.h"
#include "drake/common/string_map.h"

// Note that even though this file contains "class Node", the file is named
// "yaml_node.h" not "node.h" to avoid conflict with "yaml-cpp/node/node.h".

namespace drake {
namespace yaml {
namespace internal {

/* The three possible kinds of YAML nodes.

See https://yaml.org/spec/1.2.2/#nodes for the definition of a node.

Note that even though our links to the YAML specification point to YAML
version 1.2.2, we don't actually care about the version number in particular;
this class is not tied to a specific version.

<!--
As an implementation note for all of the below: the Node API for accessors,
modifiers, etc., is very spare compared to what we might imagine.  At, the
moment, because this is an internal class, the functions only provide the
minimal API that we need.  We can add more functions if and when we need them.
-->
*/
enum class NodeType {
  // See https://yaml.org/spec/1.2.2/#scalar for the definition.
  // See https://yaml.org/spec/1.2.2/#scalars for examples.
  //
  // Note that even though Drake most often uses "scalar" to refer to a
  // mathematical scalar type such as `double`, here we use "scalar" in the
  // sense of YAML.
  kScalar,

  // See https://yaml.org/spec/1.2.2/#sequence for the definition.
  // See https://yaml.org/spec/1.2.2/#collections for Example 2.1.
  kSequence,

  // See https://yaml.org/spec/1.2.2/#mapping for the definition.
  // See https://yaml.org/spec/1.2.2/#collections for Example 2.2.
  //
  // Note that even though YAML in general allows the keys of a mapping to be
  // any type of node, in our implementation we limit keys to be only strings,
  // for better compatibility with other serialization formats such as JSON.
  kMapping,
};

/* Denotes one of the "JSON Schema" tags.
Note that this schema incorporates the "failsafe schema" by reference.
See https://yaml.org/spec/1.2.2/#json-schema. */
enum class JsonSchemaTag {
  // https://yaml.org/spec/1.2.2/#null
  kNull,
  // https://yaml.org/spec/1.2.2/#boolean
  kBool,
  // https://yaml.org/spec/1.2.2/#integer
  kInt,
  // https://yaml.org/spec/1.2.2/#floating-point
  kFloat,
  // https://yaml.org/spec/1.2.2/#generic-string
  kStr,
};

/* Data type that represents a YAML node.  A Node can hold one of three
possible kinds of value at runtime:
- Scalar
- Sequence[Node]
- Mapping[string, Node]

Refer to https://yaml.org/spec/1.2.2/#nodes for details.

This class implements the https://yaml.org/spec/1.2.2/#321-representation-graph
concept, with two caveats for better compatibility with JSON serialization:
- graph cycles are not allowed;
- mapping keys must only be scalar strings.

Each node may also have a tag.  By default (i.e., at construction time),
the tag will be empty.  Use GetTag() and SetTag() to query and adjust it.

Refer to https://yaml.org/spec/1.2.2/#tags for details.
*/
class Node final {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Node);

  /* Returns a Scalar node with the given value.
  Note that even though Drake most often uses "scalar" to refer to a
  mathematical scalar type such as `double`, here we use "scalar" in the
  sense of YAML. */
  static Node MakeScalar(std::string value = {});

  /* Returns an empty Sequence node. */
  static Node MakeSequence();

  /* Returns an empty Mapping node. */
  static Node MakeMapping();

  /* Returns a null Scalar node.
  The returned node's tag is kTagNull and value is "null".
  Refer to https://yaml.org/spec/1.2.2/#null for details. */
  static Node MakeNull();

  /* Returns type of stored value. */
  NodeType GetType() const;

  /* Returns a description of the type of stored value, suitable for use in
  error messages, e.g., "Mapping". */
  std::string_view GetTypeString() const;

  /* Returns a description of the given type, suitable for use in error
  messages, e.g., "Mapping". */
  static std::string_view GetTypeString(NodeType);

  /* Returns true iff this Node's type is Scalar. */
  bool IsScalar() const;

  /* Returns true iff this Node's type is Sequence. */
  bool IsSequence() const;

  /* Returns true iff this Node's type is Mapping. */
  bool IsMapping() const;

  /* Compares two nodes for equality. */
  friend bool operator==(const Node&, const Node&);

  /* Gets this node's YAML tag.
  See https://yaml.org/spec/1.2.2/#tags.
  By default (i.e., at construction time), the tag will be empty. */
  std::string_view GetTag() const;

  /* Returns whether or not `important = true` was used during SetTag(). */
  bool IsTagImportant() const;

  /* Sets this node's YAML tag to one of the "JSON Schema" tags.
  Optionally allows marking the tag as "important", which means that when
  emitting the YAML document it will always appear in the output, even if
  it would have been implied by default.
  See https://yaml.org/spec/1.2.2/#json-schema. */
  void SetTag(JsonSchemaTag, bool important = false);

  /* Sets this node's YAML tag.
  See https://yaml.org/spec/1.2.2/#tags.
  The tag is not checked for well-formedness nor consistency with the node's
  type nor value.  The caller is responsible for providing a valid tag. */
  void SetTag(std::string);

  // https://yaml.org/spec/1.2.2/#null
  static constexpr std::string_view kTagNull{"tag:yaml.org,2002:null"};

  // https://yaml.org/spec/1.2.2/#boolean
  static constexpr std::string_view kTagBool{"tag:yaml.org,2002:bool"};

  // https://yaml.org/spec/1.2.2/#integer
  static constexpr std::string_view kTagInt{"tag:yaml.org,2002:int"};

  // https://yaml.org/spec/1.2.2/#floating-point
  static constexpr std::string_view kTagFloat{"tag:yaml.org,2002:float"};

  // https://yaml.org/spec/1.2.2/#generic-string
  static constexpr std::string_view kTagStr{"tag:yaml.org,2002:str"};

  // https://yaml.org/type/binary.html
  static constexpr std::string_view kTagBinary{"tag:yaml.org,2002:binary"};

  /* Sets the filename where this Node was read from. A nullopt indicates that
  the filename is not known. */
  void SetFilename(std::optional<std::string> filename);

  /* Gets the filename where this Node was read from. A nullopt indicates that
  the filename is not known. */
  const std::optional<std::string>& GetFilename() const;

  /* An indication of where in a file or string this Node was read from.
  The indexing is 1-based (the first character is line 1 column 1). */
  struct Mark {
    int line{};
    int column{};

    friend bool operator==(const Mark&, const Mark&);
  };

  /* Sets the line:column offset in the file or string where this Node was read
  from. A nullopt indicates that the Node's position is unknown. */
  void SetMark(std::optional<Mark> mark);

  /* Gets the line:column offset in the file or string where this Node was read
  from. A nullopt indicates that the Node's position is unknown. */
  const std::optional<Mark>& GetMark() const;

  // @name Scalar-only Functions
  // These functions may only be called when IsScalar() is true;
  // otherwise, they will throw an exception.
  //
  // Note that there is no SetScalar function provided; users should call
  // the Node::operator= function, instead.
  //@{

  /* Gets this node's Scalar data. */
  const std::string& GetScalar() const;

  //@}

  // @name Sequence-only Functions
  // These functions may only be called when IsSequence() is true;
  // otherwise, they will throw an exception.
  //
  // Note that there is no SetSequence function provided to bulk-overwrite the
  // sequence; users should call the Node::operator= function, instead.
  //@{

  /* Gets this node's Sequence data. */
  const std::vector<Node>& GetSequence() const;

  /* Appends a new node to the back of this Sequence.
  Any iterators based on GetSequence() are invalidated. */
  void Add(Node);

  //@}

  // @name Mapping-only Functions
  // These functions may only be called when IsMapping() is true;
  // otherwise, they will throw an exception.
  //
  // Note that there is no SetMapping function provided to bulk-overwrite the
  // mapping; users should call the Node::operator= function, instead.
  //@{

  /* Gets this node's Mapping data. */
  const string_map<Node>& GetMapping() const;

  /* Add a new node to this Mapping.
  Any iterators based on GetMapping() remain valid.
  @throws std::exception the given key was already in this mapping. */
  void Add(std::string key, Node value);

  /* Gets an existing node from this Mapping.
  Any iterators based on GetMapping() remain valid.
  @throws std::exception the given key does not exist. */
  Node& At(std::string_view key);

  /* Removes an existing node from this Mapping.
  Any iterators based on GetMapping() that referred to this key are invalidated.
  @throws std::exception the given key does not exist. */
  void Remove(std::string_view key);

  //@}

  /* Calls back into the given Visitor using operator(), with an argument
  type (see below) based on this Node's type. */
  template <class Visitor>
  void Visit(Visitor&& visitor) const {
    return std::visit(std::forward<Visitor>(visitor), data_);
  }

  /* The argument type for Visit on a Scalar node .*/
  struct ScalarData final {
    std::string scalar;

    friend bool operator==(const ScalarData&, const ScalarData&);
  };

  /* The argument type for Visit on a Sequence node .*/
  struct SequenceData final {
    std::vector<Node> sequence;

    friend bool operator==(const SequenceData&, const SequenceData&);
  };

  /* The argument type for Visit on a Mapping node .*/
  struct MappingData final {
    // Even though YAML mappings are notionally unordered, we use an ordered
    // map here to ensure program determinism.
    string_map<Node> mapping;

    friend bool operator==(const MappingData&, const MappingData&);
  };

  /* Displays the given node using flow style.  Intended only for debugging,
  not serialization. */
  friend std::ostream& operator<<(std::ostream&, const Node&);

 private:
  /* No-op for use only by the public "Make..." functions. */
  Node();

  using Variant = std::variant<ScalarData, SequenceData, MappingData>;
  Variant data_;

  // When our tag is one of the well-known JSON schema enum values, we also need
  // to store whether the tag is "important". Refer to SetTag() for details.
  struct JsonSchemaTagInfo {
    JsonSchemaTag value{JsonSchemaTag::kNull};
    bool important{false};
  };

  // A YAML tag is not required (our default value is the empty string), but can
  // be set to either a well-known enum or a bespoke string. The representation
  // here isn't canonical: it's possible to set a string value that's equivalent
  // to an enum's implied string.
  std::variant<std::string, JsonSchemaTagInfo> tag_;

  std::optional<Mark> mark_;
  std::optional<std::string> filename_;
};

}  // namespace internal
}  // namespace yaml
}  // namespace drake

#ifndef DRAKE_DOXYGEN_CXX
// TODO(jwnimmer-tri) Add a real formatter and deprecate the operator<<.
namespace fmt {
template <>
struct formatter<drake::yaml::internal::Node> : drake::ostream_formatter {};
}  // namespace fmt
#endif
