#pragma once

#include <string>

#include "drake/common/drake_deprecated.h"
#include "drake/common/eigen_types.h"
#include "drake/lcmt_call_python.hpp"

/// @file
/// Utilities for calling Python from C++ over an RPC.
///
/// For command-line examples, see the documentation in `call_python_client.py`.
/// For C++ examples, see `call_python_test.cc`.

namespace drake {
namespace common {

/// Initializes `CallPython` for a given file.  If this function is not called,
/// then the filename defaults to `/tmp/python_rpc`.
/// @throws std::runtime_error If either this function or `CallPython` have
/// already been called.
void CallPythonInit(const std::string& filename);

/// A proxy to a variable stored in Python side.
class PythonRemoteVariable;

/// Calls a Python client with a given function and arguments, returning
/// a handle to the result.  For example uses, see `call_python_test.cc`.
template <typename... Types>
PythonRemoteVariable CallPython(const std::string& function_name,
                                Types... args);

/// Creates a tuple in Python.
template <typename... Types>
PythonRemoteVariable ToPythonTuple(Types... args);

/// Creates a keyword-argument list to be unpacked.
/// @param args Argument list in the form of (key1, value1, key2, value2, ...).
template <typename... Types>
PythonRemoteVariable ToPythonKwargs(Types... args);

// ===========================================================================
// All code below this point is implementation details.
// ===========================================================================
#ifndef DRAKE_DOXYGEN_CXX

namespace internal {

class PythonItemPolicy;
class PythonAttrPolicy;
template <typename Policy>
class PythonAccessor;

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

/// Creates a new remote variable with the corresponding value set.
template <typename T>
PythonRemoteVariable NewPythonVariable(T value) {
  return CallPython("pass_through", value);
}

// Gets/sets an object's attribute.
class PythonAttrPolicy {
 public:
  using KeyType = std::string;
  static PythonRemoteVariable get(PythonRemoteVariable obj,
                                  const KeyType& key) {
    return CallPython("getattr", obj, key);
  }
  static PythonRemoteVariable set(PythonRemoteVariable obj,
                                  const KeyType& key,
                                  PythonRemoteVariable value) {
    return CallPython("setattr", obj, key, value);
  }
};

// Gets/sets an object's item.
class PythonItemPolicy {
 public:
  using KeyType = PythonRemoteVariable;
  static PythonRemoteVariable get(PythonRemoteVariable obj,
                                  const KeyType& key) {
    return CallPython("getitem", obj, key);
  }
  static PythonRemoteVariable set(PythonRemoteVariable obj,
                                  const KeyType& key,
                                  PythonRemoteVariable value) {
    return CallPython("setitem", obj, key, value);
  }
};

// API-consistent mechanism to access a portion of an object (item or attr).
template <typename Policy>
class PythonAccessor : public PythonApi<PythonAccessor<Policy>> {
 public:
  using KeyType = typename Policy::KeyType;

  // Given a variable (and key), makes a PythonAccessor.
  PythonAccessor(PythonRemoteVariable obj, const KeyType& key)
      : obj_(obj), key_(key) {}

  // Copying a PythonAccessor aliases the original remote variable (reference
  // semantics), it does not create a new remote variable.
  PythonAccessor(const PythonAccessor&) = default;

  // Implicitly converts to a PythonRemoteVariable.
  operator PythonRemoteVariable() const { return value(); }

  // Assigning from another PythonAccessor delegates to set_value from that
  // `value`'s underlying PythonRemoteVariable.
  PythonRemoteVariable operator=(const PythonAccessor& value) {
    return set_value(value);
  }

  // Assigning from another PythonRemoteVariable delegates to set_value from it.
  PythonRemoteVariable operator=(const PythonRemoteVariable& value) {
    return set_value(value);
  }

  // Assigning from some literal value creates a new PythonRemoveVariable to
  // bind the value.
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
  KeyType key_;
};

// Now that we have our types defined, we can implement the functionality for
// the API.
template <typename Derived>
PythonAttrAccessor PythonApi<Derived>::attr(const std::string& name) const {
  return {derived(), name};
}

template <typename Derived>
template <typename... Types>
PythonRemoteVariable PythonApi<Derived>::operator()(Types... args) const {
  return CallPython("call", derived(), args...);
}

template <typename Derived>
template <typename Type>
PythonItemAccessor PythonApi<Derived>::operator[](Type key) const {
  return {derived(), NewPythonVariable(key)};
}

template <typename Derived>
template <typename... Types>
PythonItemAccessor PythonApi<Derived>::slice(Types... args) const {
  return {derived(), CallPython("make_slice_arg", args...)};
}

void ToPythonRemoteData(const PythonRemoteVariable&,
                        lcmt_call_python_data*);

template <typename Derived>
void ToPythonRemoteData(const Eigen::MatrixBase<Derived>&,
                        lcmt_call_python_data*);

void ToPythonRemoteData(double scalar,
                        lcmt_call_python_data*);

void ToPythonRemoteData(int scalar,
                        lcmt_call_python_data*);

void ToPythonRemoteData(const std::string&, lcmt_call_python_data*);

void ToPythonRemoteDataMatrix(const Eigen::Ref<const MatrixX<bool>>&,
                              lcmt_call_python_data*, bool is_vector);

void ToPythonRemoteDataMatrix(const Eigen::Ref<const Eigen::MatrixXd>&,
                              lcmt_call_python_data*, bool is_vector);

void ToPythonRemoteDataMatrix(const Eigen::Ref<const Eigen::MatrixXi>&,
                              lcmt_call_python_data*, bool is_vector);

template <typename Derived>
void ToPythonRemoteData(const Eigen::MatrixBase<Derived>& mat,
                        lcmt_call_python_data* message) {
  const bool is_vector = (Derived::ColsAtCompileTime == 1);
  return ToPythonRemoteDataMatrix(mat, message, is_vector);
}

inline void AssembleRemoteMessage(lcmt_call_python*) {
  // Intentionally left blank.  Base case for template recursion.
}

template <typename T, typename... Types>
void AssembleRemoteMessage(lcmt_call_python* message, T first,
                           Types... args) {
  message->rhs.emplace_back();
  ToPythonRemoteData(first, &(message->rhs.back()));
  AssembleRemoteMessage(message, args...);
}

void PublishCallPython(const lcmt_call_python& message);

}  // namespace internal

// These items are forward-declared atop the file.

template <typename... Types>
PythonRemoteVariable CallPython(const std::string& function_name,
                                Types... args) {
  PythonRemoteVariable output;
  lcmt_call_python message{};
  message.lhs = output.unique_id();
  internal::AssembleRemoteMessage(&message, args...);
  message.num_rhs = message.rhs.size();
  message.function_name = function_name;
  internal::PublishCallPython(message);
  return output;
}

template <typename... Types>
PythonRemoteVariable ToPythonTuple(Types... args) {
  return CallPython("make_tuple", args...);
}

template <typename... Types>
PythonRemoteVariable ToPythonKwargs(Types... args) {
  return CallPython("make_kwargs", args...);
}

#endif  // DRAKE_DOXYGEN_CXX
}  // namespace common
}  // namespace drake
