#pragma once

#include <optional>
#include <ostream>
#include <string>
#include <string_view>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/reset_after_move.h"

namespace drake {
namespace multibody {

namespace internal {
// Expose the delimeter string for reference, especially in tests.
constexpr char kScopedNameDelim[] = "::";
}  // namespace internal

/** A delimited string name for a multibody element, e.g., "robot1::torso".

The name is composed of two semantically separate pieces -- the `element` name
is the local name for the element (e.g., a joint, body, frame, etc.) and the
`namespace` name is the location of that element within the tree.

Typically, the `element` name is never empty. The element name will never
contain the delimiter string "::".

The `namespace` typically refers to the model instance name that contains the
element. Some elements live at global scope, in which case the `namespace` is
empty.

The string representation of a %ScopedName is "namespace::element" unless
the `namespace` is empty, in which case the string is just "element". */
class ScopedName final {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ScopedName)

  /** Creates an empty name. */
  ScopedName() = default;

  /** Creates a ScopedName for the given `namespace` and `element`.
  See ScopedName::Make() for an exception-free construction option.
  @throws std::exception if `element` already contains the delimiter "::". */
  ScopedName(std::string_view namespace_in, std::string_view element);

  /** Creates a ScopedName for the given `namespace` and `element`.
  Returns null if `element` already contains the delimiter "::". */
  static std::optional<ScopedName> Join(
      std::string_view namespace_in, std::string_view element);

  /** Parses the given `scoped_name` string. */
  static ScopedName Parse(std::string scoped_name);

  /** Returns the namespace portion of this scoped name, e.g., "robot1".
  This is typically the model instance name.
  When empty, it denotes global scope (i.e., the world). */
  std::string_view get_namespace() const;

  /** Returns the element portion of this scoped name, e.g., "torso".
  This is the local name of the joint, body, etc. within the model instance.
  Typically, it is never empty. */
  std::string_view get_element() const;

  /** Returns the full %ScopedName as a string, e.g., "robot1::torso" */
  std::string_view to_string() const;

 private:
  std::string name_;
  reset_after_move<size_t> element_begin_{0};
};

/** Streaming output operator.
@relates ScopedName */
std::ostream& operator<<(std::ostream& out, const ScopedName& value) {
  out << value.to_string();
  return out;
}

}  // namespace multibody
}  // namespace drake



#if 0
  /** ... */
  template <typename QuacksLikeMultibodyPlant>
  ScopedName(
      const QuacksLikeMultibodyPlant& plant,
      drake::multibody::ModelInstanceIndex model_instance,
      std::string_view element)
      : ScopedName(plant.internal_tree(), model_instance, element) {}

  /** (Internal use only.) */
  template <typename T>
  ScopedName(
      const MultibodyTree<T>& internal_tree,
      drake::multibody::ModelInstanceIndex model_instance,
      std::string_view element);
#endif
