#pragma once

#include <string>
#include <vector>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/proto/call_matlab.h"

/// @file
/// @brief Utilities for calling Matlab from C++
///
/// Provides an interface similar to `call_matlab` (i.e., one-directional RPC),
/// leveraging an interface similar to `pybind11`.
///
/// @see call_python_test.cc for C++ examples.
// TODO(eric.cousineau): Add (untested) example usage in IPython notebook.

namespace drake {
namespace common {

// begin forward declarations
// Necessary since we have not yet defined `PythonRemoteVariable`.
class PythonRemoteVariable;
// TODO(eric.cousineau): Generalize RPC marhsaling so that we do not need to
// overload a function named `ToMatlabArray`.
void ToMatlabArray(const PythonRemoteVariable& var, MatlabArray* matlab_array);

template <typename... Types>
PythonRemoteVariable CallPython(const std::string& function_name,
                                Types... args);

template <typename... Types>
PythonRemoteVariable ToPythonTuple(Types... args);

template <typename T>
PythonRemoteVariable NewPythonVariable(T value);

class PythonRemoteVariable;

namespace internal {

class PythonItemPolicy;
class PythonAttrPolicy;
template <typename Policy>
class PythonAccessor;
// end forward declarations

using PythonItemAccessor = PythonAccessor<PythonItemPolicy>;
using PythonAttrAccessor = PythonAccessor<PythonAttrPolicy>;

// Mimic pybind11's `object_api` and `accessor<>` setup, such that we can
// chain operations together more conveniently.
template <typename Derived>
class PythonApi {
 public:
  /// Calls object with given arguments, returning the remote result.
  template <typename... Types>
  PythonRemoteVariable operator()(Types... args) const;

  /// Accesses an attribute.
  PythonAttrAccessor attr(const std::string& name) const;

  /// Accesses an item.
  template <typename Type>
  PythonItemAccessor operator[](Type key) const;

  /// Accesses a NumPy-friendly slice.
  template <typename... Types>
  PythonItemAccessor slice(Types... args) const;

 private:
  // Provides type-cast view for CRTP implementation.
  const Derived& derived() const {
    return static_cast<const Derived&>(*this);
  }
};

}  // namespace internal

/// Variable stored on the client (Python) side.
class PythonRemoteVariable : public internal::PythonApi<PythonRemoteVariable> {
 public:
  PythonRemoteVariable();
  // TODO(eric.cousineau): To support deletion, disable copy constructor, only
  // allow moving (if we want to avoid needing a global reference counter).
  int64_t unique_id() const { return unique_id_; }

 private:
  const int64_t unique_id_{};
};

namespace internal {

// Get/set an object's attribute.
class PythonAttrPolicy {
 public:
  using key_type = std::string;
  static PythonRemoteVariable get(PythonRemoteVariable obj,
                                  const key_type& key) {
    return CallPython("getattr", obj, key);
  }
  static PythonRemoteVariable set(PythonRemoteVariable obj,
                                  const key_type& key,
                                  PythonRemoteVariable value) {
    return CallPython("setattr", obj, key, value);
  }
};

// Get/set an object's item.
class PythonItemPolicy {
 public:
  using key_type = PythonRemoteVariable;
  static PythonRemoteVariable get(PythonRemoteVariable obj,
                                  const key_type& key) {
    return CallPython("getitem", obj, key);
  }
  static PythonRemoteVariable set(PythonRemoteVariable obj,
                                  const key_type& key,
                                  PythonRemoteVariable value) {
    return CallPython("setitem", obj, key, value);
  }
};

// API-consistent mechanism to access a portion of an object (item or attr).
template <typename Policy>
class PythonAccessor : public internal::PythonApi<PythonAccessor<Policy>> {
 public:
  using key_type = typename Policy::key_type;
  PythonAccessor(PythonRemoteVariable obj, const key_type& key)
      : obj_(obj), key_(key) {}

  operator PythonRemoteVariable() const { return value(); }

  PythonRemoteVariable operator=(const PythonAccessor& value) {
    return set_value(value);
  }

  PythonRemoteVariable operator=(const PythonRemoteVariable& value) {
    return set_value(value);
  }

  template <typename T>
  PythonRemoteVariable operator=(const T& value) {
    return set_value(NewPythonVariable(value));
  }

 private:
  PythonRemoteVariable value() const { return Policy::get(obj_, key_); }
  PythonRemoteVariable set_value(const PythonRemoteVariable& value) const {
    return Policy::set(obj_, key_, value);
  }
  PythonRemoteVariable obj_;
  key_type key_;
};

// Now that we have our types defined, we can implement the functionality for
// the API.
template <typename D>
PythonAttrAccessor PythonApi<D>::attr(const std::string& name) const {
  return {derived(), name};
}

template <typename D>
template <typename... Types>
PythonRemoteVariable PythonApi<D>::operator()(Types... args) const {
  return CallPython("call", derived(), args...);
}

template <typename D>
template <typename Type>
PythonItemAccessor PythonApi<D>::operator[](Type key) const {
  return {derived(), NewPythonVariable(key)};
}

template <typename D>
template <typename... Types>
PythonItemAccessor PythonApi<D>::slice(Types... args) const {
  return {derived(), CallPython("make_slice_arg", args...)};
}

void PublishCallPython(const MatlabRPC& msg);

}  // namespace internal

/// Call a Python client with a given function and arguments, returning
/// a handle to the result.
template <typename... Types>
PythonRemoteVariable CallPython(const std::string& function_name,
                                Types... args) {
  PythonRemoteVariable output;
  MatlabRPC msg;
  msg.add_lhs(output.unique_id());
  internal::AssembleCallMatlabMsg(&msg, args...);
  msg.set_function_name(function_name);
  internal::PublishCallPython(msg);
  return output;
}

/// Create a tuple in Python.
template <typename... Types>
PythonRemoteVariable ToPythonTuple(Types... args) {
  return CallPython("make_tuple", args...);
}

/// Create a keyword-argument list to be unpacked.
/// @param args Argument list in the form of (key1, value1, key2, value2, ...).
template <typename... Types>
PythonRemoteVariable ToPythonKwargs(Types... args) {
  return CallPython("make_kwargs", args...);
}

/// Creates a new remote variable with the corresponding value set.
template <typename T>
PythonRemoteVariable NewPythonVariable(T value) {
  return CallPython("pass_through", value);
}

}  // namespace common
}  // namespace drake
