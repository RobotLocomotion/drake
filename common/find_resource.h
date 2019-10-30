#pragma once

#include <optional>
#include <string>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"

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
  std::optional<std::string> get_absolute_path() const;

  /// Either returns the get_absolute_path() iff the resource was found,
  /// or else throws std::runtime_error.
  std::string get_absolute_path_or_throw() const;

  /// Returns the error message, iff the resource was not found.
  /// The string will never be empty; only the optional can be empty.
  std::optional<std::string> get_error_message() const;

  /// Returns the resource_path asked of FindResource.
  /// (This may be empty only in the make_empty() case.)
  std::string get_resource_path() const;

  /// Returns a success result (the requested resource was found).
  /// @pre neither string parameter is empty
  /// @param resource_path the value passed to FindResource
  /// @param base_path an absolute base path that precedes resource_path
  static FindResourceResult make_success(
      std::string resource_path, std::string absolute_path);

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

  // The absolute path where resource_path was found, if success.
  std::optional<std::string> absolute_path_;

  // An error message, permitted to be present only when base_path is empty.
  //
  // All three of resource_path, base_path, and error_message can be empty
  // (e.g., a default-constructed and/or moved-from object), which represents
  // resource-not-found along with an unspecified non-empty default error
  // message from get_error_message().
  std::optional<std::string> error_message_;
};

/// Attempts to locate a Drake resource named by the given @p resource_path.
/// The @p resource_path refers to the relative path within the Drake source
/// repository, prepended with `drake/`.  For example, to find the source
/// file `examples/pendulum/Pendulum.urdf`, the @p resource_path would be
/// `drake/examples/pendulum/Pendulum.urdf`.  Paths that do not start with
/// `drake/` will return an error result.  The @p resource_path must refer
/// to a file (not a directory).
///
/// The search scans for the resource in the following resource roots and in
/// the following order:
///
/// 1. In the DRAKE_RESOURCE_ROOT environment variable.
/// 2. In the Bazel runfiles for a bazel-bin/pkg/program.
/// 3. In the Drake CMake install directory.
///
/// The first resource root from the list that exists is used to find any and
/// all Drake resources.  If the resource root does not contain the resource,
/// the result is an error even (if a resource root lower on the list happens
/// to have the resource).  If all three roots are unavailable, then returns an
/// error result.
FindResourceResult FindResource(const std::string& resource_path);

/// Convenient wrapper for querying FindResource(resource_path) followed by
/// FindResourceResult::get_absolute_path_or_throw().
std::string FindResourceOrThrow(const std::string& resource_path);

/// The name of the environment variable that provides the first place where
/// FindResource attempts to look.  The environment variable is allowed to be
/// unset or empty; in that case, FindResource will attempt to use other
/// locations without complaint.
///
/// The value is guaranteed to be "DRAKE_RESOURCE_ROOT".  (For some users, it
/// may be easier to hard-code a value than refer to this constant.)
///
/// When the environment variable is set, resources are sought in relation to
/// it by appending the FindResource() `resource_path` to the environment
/// variable (with an intermediate `/` as appropriate).  For example, if the
/// `resource_path` is `drake/examples/pendulum/Pendulum.urdf` and the
/// `DRAKE_RESOURCE_ROOT` is set to `/home/someuser/foo` then the resource will
/// be sought at `/home/someuser/foo/drake/examples/pendulum/Pendulum.urdf`.
///
/// The intended use of this variable is to seek resources from an installed
/// copy of Drake, in case other methods have failed.
extern const char* const kDrakeResourceRootEnvironmentVariableName;

}  // namespace drake
