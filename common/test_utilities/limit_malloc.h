#pragma once

namespace drake {
namespace test {

/// Parameters to control malloc limits.
struct LimitMallocParams {
  /// Maximum calls to malloc, calloc, or realloc (totaled as one).
  /// When less than zero, there is no limit on the number of calls.
  int max_num_allocations{-1};

  /// Minimum calls to malloc, calloc, or realloc (totaled as one).
  /// When less than zero, there is no limit on the number of calls.
  int min_num_allocations{-1};

  /// Whether a realloc() that leaves its `ptr` unchanged should be ignored.
  bool ignore_realloc_noops{false};
};

/// Instantiate this class in a unit test scope where malloc (and realloc,
/// etc.) should be disallowed or curtailed.
///
/// @note This class is currently a no-op in some build configurations:
///       - macOS
///       - leak sanitizer
///       - valgrind tools
///
/// Example:
/// @code
/// GTEST_TEST(LimitMallocTest, BasicTest) {
///   std::vector<double> foo(100);  // Heap allocation is OK.
///   {
///     LimitMalloc guard;
///     // The guarded code goes here.  Heap allocations result in aborts.
///     std::array<double, 100> stack;
///   }
///   std::vector<double> bar(100);  // Heap allocation is OK again.
/// }
/// @endcode
///
/// Currently, when the device under test violates its allocation limits, the
/// test terminates with an abort().  To better isolate what went wrong, re-run
/// the test in a debugger.
///
/// This class is only intended for use in test code.  To temporarily use it in
/// non-test code, hack the BUILD.bazel file to mark this library `testonly = 0`
/// instead of `testonly = 1`.
class LimitMalloc final {
 public:
  /// Applies malloc limits until this object's destructor is run.
  /// All allocations will fail.
  /// For now, only *one* instance of this class may be created at a time.
  /// (In the future, we may allow updating the limits by nesting these guards.)
  LimitMalloc();

  /// Applies malloc limits until this object's destructor is run.
  /// A allocations will succeed except for any limits designated in args.
  /// For now, only *one* instance of this class may be created at a time.
  /// (In the future, we may allow updating the limits by nesting these guards.)
  explicit LimitMalloc(LimitMallocParams args);

  /// Undoes this object's malloc limits.
  ~LimitMalloc();

  /// Returns the number of allocations observed so far.
  int num_allocations() const;

  /// Returns the parameters structure used to construct this object.
  const LimitMallocParams& params() const;

  // We write this out by hand, to avoid depending on Drake *at all*.
  /// @name Does not allow copy, move, or assignment
  //@{
  LimitMalloc(const LimitMalloc&) = delete;
  void operator=(const LimitMalloc&) = delete;
  LimitMalloc(LimitMalloc&&) = delete;
  void operator=(LimitMalloc&&) = delete;
  //@}
};

}  // namespace test
}  // namespace drake
