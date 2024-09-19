#pragma once

#include <optional>
#include <string>
#include <string_view>

#include "drake/common/drake_copyable.h"
#include "drake/common/fmt.h"
#include "drake/common/reset_after_move.h"

namespace drake {
namespace multibody {

/** A delimited string name for a multibody element, e.g., "robot1::torso".

The name is composed of two semantically separate pieces -- the `element` name
is the local name for the element (e.g., a joint, body, frame, etc.) and the
`namespace` name is the location of that element within the tree. For
"robot1::torso" the namespace name is "robot1" and the element name is "torso".

The namespace name typically refers to the model instance name that contains the
element. Some temporary scoped names do not use a namespace (e.g., temporary
values created during input file parsing), in which case the namespace name can
be empty. The namespace name will never start or end with "::".

The element name is never empty, unless the %ScopedName was default-constructed
or moved-from. The element name will never contain the delimiter string "::".

When there is no namespace, the scoped name does not contain a leading "::",
e.g., for the element name "box" without any namespace, the scoped name is "box"
not "::box".

The namespace name may contain the "::" delimiter in the middle of the name
(possibly multiple times), e.g., for "robot1::left::arm::end_frame" the
namespace name is "robot1::left::arm" and the element is name "end_frame".

This class does not treat a single colon (":") specially. Those can appear
in either namespace names or element names. */
class ScopedName final {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ScopedName);

  /** Creates an empty name. */
  ScopedName() = default;

  /** Creates a %ScopedName for the given `namespace_name` and `element_name`.
  @throws std::exception if `namespace_name` starts or ends with "::".
  @throws std::exception if `element_name` contains "::" or is empty.
  @see ScopedName::Make() to use a null return value instead of exceptions.
  @see ScopedName::Join() for automatic cleanup of "::" tokens. */
  ScopedName(std::string_view namespace_name, std::string_view element_name);

  /** Creates a %ScopedName for the given `namespace_name` and `element_name`.
  Returns nullopt if `namespace_name` starts or ends with "::", or if
  `element_name` contains "::" or is empty.
  @see ScopedName::Join() for automatic coalescing of "::" tokens. */
  static std::optional<ScopedName> Make(std::string_view namespace_name,
                                        std::string_view element_name);

  /** Creates a %ScopedName for the given `name1::name2`. Unlike the constructor
  or ScopedName::Make(), this function allows "::" in either name. Any leading
  or trailing "::" on the names are removed before joining. After joining, the
  final word after all "::"s is the element name, and everything prior is the
  namespace name. */
  static ScopedName Join(std::string_view name1, std::string_view name2);

  /** Parses the given `scoped_name` string. Any leading or trailing "::"s on
  the name are removed (even multiple copies like "::::" are removed). */
  static ScopedName Parse(std::string scoped_name);

  /** Returns the namespace portion of this scoped name, e.g., "robot1". This is
  typically the model instance name. This is typically the model instance name
  but can be empty (see class overview for details). */
  std::string_view get_namespace() const;

  /** Returns the element portion of this scoped name, e.g., "torso". This is
  the local name of the joint, body, etc. within the model instance. It is never
  empty unless this %ScopedName was default-constructed or moved-from. */
  std::string_view get_element() const;

  /** Returns the full %ScopedName as a string, e.g., "robot1::torso". It is
  never empty unless this %ScopedName was default-constructed or moved-from. */
  std::string_view get_full() const;

  /** Returns get_full() as a string value instead of a string_view. */
  std::string to_string() const;

  /** Replaces the namespace name of this object, leaving the element name
  unchanged. The namespace name is allowed to be empty.
  @throws std::exception if `namespace_name` starts or ends with "::". */
  void set_namespace(std::string_view namespace_name);

  /** Replaces the element name of this object, leaving the namespace name
  unchanged.
  @throws std::exception if `element_name` contains "::" or is empty. */
  void set_element(std::string_view element_name);

 private:
  std::string name_;
  reset_after_move<size_t> element_begin_{0};
};

}  // namespace multibody
}  // namespace drake

DRAKE_FORMATTER_AS(, drake::multibody, ScopedName, x, x.to_string())
