#pragma once

#include <string>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"

namespace drake {

/// Models the outcome of drake::FindResource.  After a call to FindResource,
/// typical calling code would use get_absolute_path_or_throw().
/// Alternatively, get_absolute_path() will return an `optional<string>`, which
/// can be manually checked to contain a value before using the path.  If the
/// resource was not found, get_error_message() will contain an error message.
///
/// For a given FindResourceResult instance, exactly one of get_absolute_path()
/// or get_error_message() will contain a value.  (Similarly, exactly one of
/// them will not contain a value.)
class FindResourceResult {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(FindResourceResult);

  /// Returns the absolute path to the resource, iff the resource was found.
  optional<std::string> get_absolute_path() const;

  /// Either returns the get_absolute_path() iff the resource was found,
  /// or else throws runtime_error.
  std::string get_absolute_path_or_throw() const;

  /// Returns the error message, iff the resource was not found.
  /// The string will never be empty; only the optional can be empty.
  optional<std::string> get_error_message() const;

  /// Returns the resource_path asked of FindResource.
  /// (This may be empty only in the make_empty() case.)
  std::string get_resource_path() const;

  /// Returns a success result (the requested resource was found).
  /// @pre neither string parameter is empty
  /// @param resource_path the value passed to FindResource
  /// @param base_path an absolute base path that precedes resource_path
  static FindResourceResult make_success(
      std::string resource_path, std::string base_path);

  /// Returns an error result (the requested resource was NOT found).
  /// @pre neither string parameter is empty
  /// @param resource_path the value passed to FindResource
  static FindResourceResult make_error(
      std::string resource_path, std::string error_message);

  /// Returns an empty error result (no requested resource).
  static FindResourceResult make_empty();

 private:
  FindResourceResult() = default;
  void CheckInvariants();

  // The path as requested by the user.
  std::string resource_path_;

  // The base path (directory where resource_path was found), if found.
  optional<std::string> base_path_;

  // An error message, permitted to be present only when base_path is empty.
  //
  // All three of resource_path, base_path, and error_message can be empty
  // (e.g., a default-constructed and/or moved-from object), which represents
  // resource-not-found along with an unspecified non-empty default error
  // message from get_error_message().
  optional<std::string> error_message_;
};

/// Attempts to locate a Drake resource named by the given @p resource_path.
/// The @p resource_path refers to the relative path within the Drake
/// repository, e.g., `drake/examples/Pendulum/Pendulum.urdf`.
///
/// When called from within a source code workspace (i.e., what a Drake
/// developer would use), this finds the resource within the current workspace.
///
/// When called from an installed binary build of Drake, this is intended to
/// find the installed resource, but that feature is not yet implemented.
FindResourceResult FindResource(std::string resource_path);

/// The name of the environment variable that provides the first place where
/// FindResource attempts to look.  The environment variable is allowed to be
/// unset or empty; in that case, FindResource will attempt to use other
/// locations without complaint.
///
/// The value is guaranteed to be "DRAKE_RESOURCE_ROOT".  (For some users, it
/// may be easier to hard-code a value than refer to this constant.)
extern const char* const kDrakeResourceRootEnvironmentVariableName;

}  // namespace drake
