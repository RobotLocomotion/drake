#pragma once

#include <functional>
#include <string>
#include <unordered_map>

#include "drake/common/drake_copyable.h"

namespace drake {
namespace internal {

// NameDeprecator supports deprecated alias names within some limited
// scope. If the deprecated name is given to MaybeTranslate(), emit text log
// warning once per process. The warning message will include the scope,
// deprecated name, correct name, and removal date.
//
// Clients should consider wrapping this API for convenience. For example:
//
// @code
// void Something::DeclareDeprecatedWidget(
//     const std::string& removal_date,
//     const std::string& deprecated_name,
//     const std::string& correct_name) {
//   if (!widget_deprecator_.HasScope()) {
//     widget_deprecator_.DeclareScope(
//         "something widgets",
//         [this](const std::string& x){ return IsWidget(x); });
//   }
//   widget_deprecator_.DeclareDeprecatedName(
//       removal_date, deprecated_name, correct_name);
// }
// @endcode
class NameDeprecator {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(NameDeprecator)

  NameDeprecator();

  // @returns true if scope has been declared.
  bool HasScope() const;

  // Declare the scope in which names are relevant, once per instance.
  // @param name       the name of the overall scope
  // @param predicate  returns true if a name is a "correct name" in scope
  // @pre HasScope() is false.
  //
  // It is permissible for the predicate to be aware of declared deprecated
  // names; predicates are checked and acted on for a potential declaration
  // before the declaration is made active.
  void DeclareScope(const std::string& name,
                    std::function<bool(const std::string&)> predicate);

  // Declare a deprecated name, the correct name it aliases, and the date when
  // the deprecated alias will be removed.
  // @pre HasScope() is true, `removal_date` roughly matches Y-m-d format,
  //      `deprecated_name` fails the scope predicate, `correct_name` satisfies
  //      the scope predicate, and `deprecated_name` has not already been
  //      declared.
  void DeclareDeprecatedName(
      const std::string& removal_date,
      const std::string& deprecated_name,
      const std::string& correct_name);

  // If the provided name is a deprecated name, return the correct
  // name. Otherwise return the passed-in reference. Note that the return
  // value lifetime will be the minimum of the lifetime of this deprecator,
  // and the lifetime of the passed name.
  const std::string& MaybeTranslate(const std::string& name) const;

 private:
  std::string scope_name_;
  std::function<bool(const std::string&)> scope_predicate_;

  struct NameRecord {
    std::string removal_date;
    std::string correct_name;
    mutable bool is_warning_issued{false};
  };
  std::unordered_map<std::string, NameRecord> deprecations_;
};

}  // namespace internal
}  // namespace drake
