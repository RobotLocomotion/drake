#pragma once

#include <map>
#include <ostream>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

#include "drake/common/drake_copyable.h"

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

/* Data type that represents a YAML node.  A Node can hold one of three
possible kinds of value at runtime:
- Scalar
- Sequence[Node]
- Mapping[string, Node]

Refer to https://yaml.org/spec/1.2.2/#nodes for details.

Note that even though YAML in general allows the keys of a mapping to be
any type of node, in our implementation we limit keys to be only strings,
for better compatibility with other serialization formats such as JSON.

Each node may also have a tag.  By default (i.e., at construction time),
the tag will be empty.  Use GetTag() and SetTag() to query and adjust it.

Refer to https://yaml.org/spec/1.2.2/#tags for details.
*/
class Node final {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Node)

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
  const std::string& GetTag() const;

  /* Sets this node's YAML tag.
  See https://yaml.org/spec/1.2.2/#tags.
  The tag is not checked for well-formedness nor consistency with the node's
  type nor value.  The caller is responsible for providing a valid tag. */
  void SetTag(std::string);

  // https://yaml.org/spec/1.2.2/#floating-point
  static constexpr std::string_view kTagFloat{"tag:yaml.org,2002:float"};

  // https://yaml.org/spec/1.2.2/#integer
  static constexpr std::string_view kTagInt{"tag:yaml.org,2002:int"};

  // https://yaml.org/spec/1.2.2/#null
  static constexpr std::string_view kTagNull{"tag:yaml.org,2002:null"};

  // https://yaml.org/spec/1.2.2/#generic-string
  static constexpr std::string_view kTagStr{"tag:yaml.org,2002:str"};

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
  const std::map<std::string, Node>& GetMapping() const;

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
    std::map<std::string, Node> mapping;

    friend bool operator==(const MappingData&, const MappingData&);
  };

  /* Displays the given node using flow style.  Intended only for debugging,
  not serialization. */
  friend std::ostream& operator<<(std::ostream&, const Node&);

 private:
  /* No-op for use only by the public "Make..." functions. */
  Node();

  using Variant = std::variant<ScalarData, SequenceData, MappingData>;

  std::string tag_;
  Variant data_;
};

}  // namespace internal
}  // namespace yaml
}  // namespace drake
