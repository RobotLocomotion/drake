#pragma once

#include <map>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

#include "drake/common/drake_copyable.h"

namespace drake {
namespace yaml {
namespace internal {

/* Data type that represents a YAML node.  It can hold one of three
possible kinds of value at runtime: Scalar, Sequence, or Map.
*/
class Node final {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Node)


  /* Returns a Scalar node with the given initial value. */
  static Node MakeScalar(std::string value = {});

  /* Returns an empty Sequence node. */
  static Node MakeSequence();

  /* Returns an empty Map node. */
  static Node MakeMap();

  /* Creates an empty Scalar node. */
  Node() = default;

  /* Returns true iff this Node's type is Scalar. */
  bool IsScalar() const;

  /* Returns true iff this Node's type is Sequence. */
  bool IsSequence() const;

  /* Returns true iff this Node's type is Map. */
  bool IsMap() const;

  friend bool operator==(const Node& a, const Node& b) {
    return std::tie(a.tag_, a.data_) == std::tie(b.tag_, b.data_);
  }

  /* Gets this node's YAML tag. */
  const std::string& GetTag() const;

  /* Sets this node's YAML tag. */
  void SetTag(std::string);

  // @name Scalar-only Functions
  // These functions may only be called when IsScalar() is true;
  // otherwise, they will throw an exception.
  //@{

  /* Gets this node's Scalar data. */
  const std::string& GetScalar() const;

  //@}

  // @name Sequence-only Functions
  // These functions may only be called when IsSequence() is true;
  // otherwise, they will throw an exception.
  //@{

  /* Gets this node's Sequence data. */
  const std::vector<Node>& GetSequence() const;

  /* Appends a new node to the back of this Sequence. */
  void Add(Node);

  //@}

  // @name Map-only Functions
  // These functions may only be called when IsMap() is true;
  // otherwise, they will throw an exception.
  //@{

  /* Gets this node's Map data. */
  const std::map<std::string, Node>& GetMap() const;

  /* Add a new node to this Map. */
  void Add(std::string key, Node value);

  /* Gets an existing node from this Map.
  @throws std::exception the given key does not exist. */
  Node& At(std::string_view key);

  /* Removes an existing node from this Map.
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

    friend bool operator==(const ScalarData& a, const ScalarData& b) {
      return a.scalar == b.scalar;
    }
  };

  /* The argument type for Visit on a Sequence node .*/
  struct SequenceData final {
    std::vector<Node> sequence;

    friend bool operator==(const SequenceData& a, const SequenceData& b) {
      return a.sequence == b.sequence;
    }
  };

  /* The argument type for Visit on a Map node .*/
  struct MapData final {
    std::map<std::string, Node> map;

    friend bool operator==(const MapData& a, const MapData& b) {
      return a.map == b.map;
    }
  };

 private:
  using Variant = std::variant<ScalarData, SequenceData, MapData>;

  std::string tag_;
  Variant data_;
};

}  // namespace internal
}  // namespace yaml
}  // namespace drake
