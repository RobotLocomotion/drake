#pragma once

#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <type_traits>
#include <unordered_set>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/name_value.h"
#include "drake/common/string_unordered_map.h"
#include "drake/common/string_unordered_set.h"
#include "drake/solvers/solver_id.h"
#include "drake/solvers/solver_options.h"

namespace drake {
namespace solvers {
namespace internal {

/* Helper class that propagates options for a specific solver from Drake's
generic SolverOptions struct into the specific solver's options API.

This class is intended as a short-lived helper (i.e., a local variable on the
call stack). It aliases an immutable list of overall SolverOptions, and provides
convenient tools to map those options into whatever back-end is necessary. Two
different mechanisms are offered, depending on whether the solver's option names
are dynamic or static: CopyToCallbacks or CopyToSerializableStruct.

(Method 1 - dynamic) CopyToCallbacks: This method is appropriate for solver
back-ends that offer an API like "set option via string name". The back-end
provides callbacks for each supported data type (i.e., double, int, string), and
this class loops over the stored options and invokes the callbacks. It is up to
the solver back-end to detect unknown or mis-typed options and throw.

(Method 2 - static) CopyToSerializableStruct: This method is appropriate for
solvers that offer a statically typed `struct MyOptions { ... }` which needs to
be filled in. The caller provides a Serialize() wrapper around that struct, and
this class directly writes the fields and reports errors for unknown names and
mismatched data types.

With both methods, there are also some ahead-of-time operations that are often
useful:

- Respell() can project CommonSolverOption values into the back-end vocabulary
  so that only the back-end specific names need to be implemented during options
  handling.

- Pop() can yank options that require special handling, so that they will not
  participate in the Method 1 or 2 copying / callbacks.

For examples of use, refer to any of the existing Drake solver wrappers. */
class SpecificOptions {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SpecificOptions);

  /* Creates a converter that reads the subset of `all_options` destined for the
  given `id`. Both arguments are aliased, so must outlive this object. */
  SpecificOptions(const SolverId* id, const SolverOptions* all_options);

  ~SpecificOptions();

  /* The `respell` callback will be used to respell the CommonSolverOption
  values. Any options returned (via the output argument) will be handled as if
  they were solver-specific options, at a lower priority than any solver-
  specific options the user already provided for those names (in our
  constructor). It is OK for the callback to have side-effects; it will be
  invoked exactly once and is not retained. As such, this can also serve as a
  way to inject default values (by outputting them from this callback).
  @pre This function may be called at most once (per converter object). */
  void Respell(
      const std::function<void(
          const CommonSolverOptionValues& /* common */,
          string_unordered_map<SolverOptions::OptionValue>* /* respelled */)>&
          respell);

  /* Returns and effectively removes the value of the option named `key`. (To
  be specific: the `all_options` object passed to our constructor is unchanged;
  instead, the `key` is memorized and future calls to either of the `CopyTo...`
  functions will skip over it.) If the option was not set or was set to a
  different type, returns nullopt. When checking if an option was set, this
  checks `all_options` for our `id` as well as any options added during a
  Respell().
  @tparam Result must be one of: double, int, std::string. */
  template <typename Result>
  std::optional<Result> Pop(std::string_view key);

  /* Helper for "Method 1 - dynamic", per our class overview. Converts options
  when the solver offers a generic key-value API where options are passed by
  string name. Any of the `set_...` arguments can be nullptr, which implies
  that the back-end does not support any options of that type, which implies
  that any options set to such a type will throw an exception. */
  void CopyToCallbacks(
      const std::function<void(const std::string& key, double value)>&
          set_double,
      const std::function<void(const std::string& key, int value)>& set_int,
      const std::function<void(const std::string& key,
                               const std::string& value)>& set_string) const;

  /* Helper for "Method 2 - static", per our class overview. Converts options
  when the solver uses an options struct with names known at compile time. The
  `Result` struct must obey Drake's Serialize() protocol, for us to interrogate
  the option names.
  @param [in,out] result The solver's specific options struct; only options that
  are defined by this SpecificOptions object will be overwritten. Visiting an
  option that is NOT defined by this SpecificOptions object is NOT an error. */
  template <typename Result>
  void CopyToSerializableStruct(Result* result) {
    DRAKE_DEMAND(result != nullptr);
    InitializePending();
    Serialize(this, *result);
    CheckNoPending();
  }

  /* (Internal use only) Helper function for CopyToSerializableStruct(). */
  template <typename T>
  void Visit(const NameValue<T>& x) {
    if constexpr (std::is_floating_point_v<T>) {
      CopyFloatingPointOption(x.name(), x.value());
    } else if constexpr (std::is_integral_v<T>) {
      CopyIntegralOption(x.name(), x.value());
    } else if constexpr (std::is_same_v<T, std::string>) {
      CopyStringOption(x.name(), x.value());
    } else {
      static_assert(std::is_void_v<T>, "Unsupported template argument T");
    }
  }

 private:
  /* Helper function for CopyToSerializableStruct(). Sets pending_keys_ to the
  union of all keys from all_options_ (for our id_) and respelled_, excluding
  anything already popped_. */
  void InitializePending();

  /* Helper function for CopyToSerializableStruct(). If pending_keys_ is
  non-empty, throws an error about the unhandled options. */
  void CheckNoPending() const;

  /* Helper function for CopyToSerializableStruct(). Finds the option with the
  given name, removes it from pending_keys_, and returns it. If the option was
  not set, returns nullptr. */
  const SolverOptions::OptionValue* PrepareToCopy(const char* name);

  /* Output helper functions for CopyToSerializableStruct().
  Each one sets its `output` argument to the option value for the given name,
  and removes the name from pending_keys_. If the name is not in options, does
  nothing (output keeps its prior value). */
  //@{
  template <typename T /* double, float */>
  void CopyFloatingPointOption(const char* name, T* output);
  template <typename T /* int, bool, uint32_t */>
  void CopyIntegralOption(const char* name, T* output);
  void CopyStringOption(const char* name, std::string* output);
  //@}

  // The common (Drake) options for all solvers.
  const CommonSolverOptionValues common_options_;

  // The direct options for the solver we're operating on behalf of.
  // These take precedence over the common_options_.
  // @warning When looking up options in this map, you must ALWAYS cross-check
  // the key against popped_ and pretend it's missing here when listed there.
  const string_unordered_map<SolverOptions::OptionValue>& direct_options_;

  // The name of the solver we're operating on behalf of.
  const std::string& solver_name_;

  // Keys of direct_options_ that have already been popped.
  string_unordered_set popped_;

  // The result of the Respell() callback (or empty, if never called).
  string_unordered_map<SolverOptions::OptionValue> respelled_;

  // Temporary storage during CopyToSerializableStruct() of option names that
  // are set but haven't yet been processed by Visit().
  std::unordered_set<std::string_view> pending_keys_;
};

}  // namespace internal
}  // namespace solvers
}  // namespace drake
