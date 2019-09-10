#pragma once

#include <string>
#include <vector>

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/eigen_types.h"
#include "drake/common/proto/python_remote_message.pb.h"

/// @file
/// @brief Utilities for calling Python from C++
///
/// Provides one-directional RPC, leveraging an API similar to `pybind11`.
///
/// For command-line examples, see the documentation in `call_python_client.py`.
/// For C++ examples, see `call_python_test.cc`.

namespace drake {
namespace common {

// begin forward declarations
// These are necessary for `PythonApi`.

class PythonRemoteVariable;

template <typename... Types>
PythonRemoteVariable CallPython(const std::string& function_name,
                                Types... args);

template <typename... Types>
PythonRemoteVariable ToPythonTuple(Types... args);

template <typename T>
PythonRemoteVariable NewPythonVariable(T value);

void ToPythonRemoteData(const PythonRemoteVariable& variable,
                        PythonRemoteData* data);

template <typename Derived>
void ToPythonRemoteData(const Eigen::MatrixBase<Derived>& mat,
                        PythonRemoteData* data);

void ToPythonRemoteData(double scalar, PythonRemoteData* data);

void ToPythonRemoteData(int scalar, PythonRemoteData* data);

void ToPythonRemoteData(const std::string& str, PythonRemoteData* data);

using MatlabRPC
    DRAKE_DEPRECATED("2019-12-01", "Use PythonRemoteMessage instead.")
    = PythonRemoteMessage;

using MatlabArray
    DRAKE_DEPRECATED("2019-12-01", "Use PythonRemoteData instead.")
    = PythonRemoteData;

DRAKE_DEPRECATED("2019-12-01", "Use ToPythonRemoteData() instead.")
void ToMatlabArray(const PythonRemoteVariable& variable,
                   PythonRemoteData* data);

template <typename Derived>
DRAKE_DEPRECATED("2019-12-01", "Use ToPythonRemoteData() instead.")
void ToMatlabArray(const Eigen::MatrixBase<Derived>& mat,
                   PythonRemoteData* data);

DRAKE_DEPRECATED("2019-12-01", "Use ToPythonRemoteData() instead.")
void ToMatlabArray(double scalar, PythonRemoteData* data);

DRAKE_DEPRECATED("2019-12-01", "Use ToPythonRemoteData() instead.")
void ToMatlabArray(int scalar, PythonRemoteData* data);

DRAKE_DEPRECATED("2019-12-01", "Use ToPythonRemoteData() instead.")
void ToMatlabArray(const std::string& str, PythonRemoteData* data);

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

/// Presents variable stored in Python side.
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
class PythonAccessor : public internal::PythonApi<PythonAccessor<Policy>> {
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

inline void AssembleRemoteMessage(PythonRemoteMessage*) {
  // Intentionally left blank.  Base case for template recursion.
}

template <typename T, typename... Types>
void AssembleRemoteMessage(PythonRemoteMessage* message, T first,
                           Types... args) {
  ToPythonRemoteData(first, message->add_rhs());
  AssembleRemoteMessage(message, args...);
}

void ToPythonRemoteDataMatrix(
    const Eigen::Ref<const Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>>&
        mat, PythonRemoteData* data, bool is_vector);

void ToPythonRemoteDataMatrix(const Eigen::Ref<const Eigen::MatrixXd>& mat,
                              PythonRemoteData* data, bool is_vector);

void ToPythonRemoteDataMatrix(const Eigen::Ref<const Eigen::MatrixXi>& mat,
                              PythonRemoteData* data, bool is_vector);

void PublishCallPython(const PythonRemoteMessage& message);

}  // namespace internal

/// Initializes `CallPython` for a given file.
/// If this function is not called, then the file defaults to `/tmp/python_rpc`.
/// @throws std::runtime_error If either this function or `CallPython` have
/// already been called.
void CallPythonInit(const std::string& filename);

/// Calls a Python client with a given function and arguments, returning
/// a handle to the result.
template <typename... Types>
PythonRemoteVariable CallPython(const std::string& function_name,
                                Types... args) {
  PythonRemoteVariable output;
  PythonRemoteMessage message;
  message.add_lhs(output.unique_id());
  internal::AssembleRemoteMessage(&message, args...);
  message.set_function_name(function_name);
  internal::PublishCallPython(message);
  return output;
}

/// Creates a tuple in Python.
template <typename... Types>
PythonRemoteVariable ToPythonTuple(Types... args) {
  return CallPython("make_tuple", args...);
}

/// Creates a keyword-argument list to be unpacked.
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

template <typename Derived>
void ToPythonRemoteData(const Eigen::MatrixBase<Derived>& mat,
                        PythonRemoteData* data) {
  const bool is_vector = (Derived::ColsAtCompileTime == 1);
  return internal::ToPythonRemoteDataMatrix(mat, data, is_vector);
}

template <typename Derived>
DRAKE_DEPRECATED("2019-12-01", "Use ToPythonRemoteData() instead.")
void ToMatlabArray(const Eigen::MatrixBase<Derived>& mat,
                   PythonRemoteData* data) {
  ToPythonRemoteData(mat, data);
}

}  // namespace common
}  // namespace drake
