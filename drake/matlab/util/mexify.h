#pragma once

//
// Created by Twan Koolen on 8/22/15.
//

#include <functional>
#include <sstream>
#include <stdexcept>

#include "drake/matlab/util/drakeMexUtil.h"

#if defined(_MSC_VER) && (_MSC_VER < 1900)
#define NOEXCEPT
#else
#define NOEXCEPT noexcept
#endif

/**
 * Functions that make it easy to create a mex file that calls a std::function.
 *
 * Example:
 * Calling a C++ function with signature
 *
 *   bool foo(int arg)
 *
 * can be done by creating a mex function with body
 *
 *   function<bool(int)> func = &foo;
 *   mexCallFunction(func, nlhs, plhs, nrhs, prhs);
 *
 * Overloads for isConvertibleFromMex and fromMexUnsafe should be provided for
 *each argument type.
 * isConvertibleFromMex and fromMexUnsafe functions should be of the form
 *
 *  bool isConvertibleFromMex(const mxArray* source, TriggerType*, std::ostream*
 *log) NOEXCEPT
 *  ArgumentType fromMexUnsafe(const mxArray* source, TriggerType*)
 *
 * where TriggerType is the type of the argument currently being processed
 *according to the signature
 * of the std::function that was passed in (with cv qualifiers and reference
 *removed)
 * and ArgumentType is the type that will actually be passed into the function
 *(which may or may not be the same as TriggerType).
 * isConvertibleFromMex functions should do minimal work when the log argument
 *is a nullptr. If it is not null, they should provide a helpful error message.
 *
 * toMex functions should be of the form
 *
 *   int toMex(const SourceType& source, mxArray* dest[], int nlhs)
 *
 * where SourceType is such that the return type of the std::function can be
 *converted to it. The toMex function should return the number of Matlab output
 *arguments it has set.
 */

/*
 * remove reference because in general you don't want to return a reference from
 * fromMex
 * remove const to have fewer template specializations
 */
template <typename T>
using FromMexType =
    typename std::remove_cv<typename std::remove_reference<T>::type>::type;

/**
 * mexCallFunctionUnsafe
 * Converts arguments and calls function *without* checking whether arguments
 * are convertible first.
 */
// base case with return
template <typename R>
void mexCallFunctionUnsafe(std::function<R(void)> func, int nlhs,
                           mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  // call function and convert to mxArray
  toMex(func(), plhs, nlhs);
}

// base case without return
template <>
void mexCallFunctionUnsafe(std::function<void(void)> func, int nlhs,
                           mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  func();
}

// recursive case
template <typename R, typename Arg0, typename... Args>
void mexCallFunctionUnsafe(std::function<R(Arg0, Args...)> func, int nlhs,
                           mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
  // bind the first argument
  auto partially_applied = [&](Args &&... tail) {
    return std::move(func)(
        fromMexUnsafe(prhs[0], static_cast<FromMexType<Arg0> *>(nullptr)),
        std::forward<Args>(tail)...);
  };

  // recursively call mexCallFunctionUnsafe with partially applied function
  return mexCallFunctionUnsafe(std::function<R(Args...)>{partially_applied},
                               nlhs, plhs, nrhs - 1, &prhs[1]);
}

/**
 * areArgumentsConvertibleFromMex
 * Checks whether the Matlab input arguments match the function signature
 * When the log argument is not a nullptr, writes diagnostic message to log upon
 * failure.
 */
// base case
bool areArgumentsConvertibleFromMex(int nrhs, const mxArray *prhs[],
                                    int arg_num, std::ostream *log) NOEXCEPT {
  return true;  // nothing to convert, all done
}

// recursive case
template <typename Arg0, typename... Args>
bool areArgumentsConvertibleFromMex(int nrhs, const mxArray *prhs[],
                                    int arg_num, std::ostream *log, Arg0 *,
                                    Args *... tail) NOEXCEPT {
  if (arg_num == 0) {
    // check number of input arguments when starting to process arguments
    const int expected_num_args = sizeof...(Args) + 1;  // don't forget Arg0
    if (nrhs != expected_num_args) {
      if (log)
        *log << "Expected " << expected_num_args << " arguments, but got "
             << nrhs << ".";
      return false;
    }
  }

  if (!isConvertibleFromMex(prhs[0], (FromMexType<Arg0> *)nullptr, log)) {
    if (log)
      *log << std::endl
           << "Error occurred while checking argument " << arg_num << " of "
           << nrhs << ".";
    return false;
  }

  return areArgumentsConvertibleFromMex(nrhs, &prhs[1], arg_num + 1, log,
                                        tail...);
}

/**
 * mexCallFunction
 * checks whether arguments are convertible, and then calls
 * mexCallFunctionUnsafe.
 * throws a runtime_error with a diagnostic message upon failure
 */
template <typename R, typename... Args>
bool mexCallFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[],
                     bool throw_on_error, std::function<R(Args...)> func) {
  if (areArgumentsConvertibleFromMex(
          nrhs, prhs, 0, nullptr,
          (static_cast<FromMexType<Args> *>(nullptr))...)) {
    mexCallFunctionUnsafe(func, nlhs, plhs, nrhs, prhs);
    return true;
  } else if (throw_on_error) {
    std::ostringstream log;
    log << "mexCallFunction: unable to convert mex arguments:" << std::endl;
    areArgumentsConvertibleFromMex(
        nrhs, prhs, 0, &log, (static_cast<FromMexType<Args> *>(nullptr))...);
    throw std::runtime_error(log.str());
  }
  return false;
}

/**
 * collectMexTryCallFunctionsErrorDiagnostics
 * write diagnostic information to log; called after mexTryToCallFunctions has
 * exhausted all options
 */
// base case
void collectMexTryCallFunctionsErrorDiagnostics(int nrhs, const mxArray *prhs[],
                                                std::ostream &log,
                                                int function_num) {
  // empty
}

// recursive case
template <typename R0, typename... Args0, typename... Funcs>
void collectMexTryCallFunctionsErrorDiagnostics(
    int nrhs, const mxArray *prhs[], std::ostream &log, int function_num,
    std::function<R0(Args0...)> function0, Funcs... functions) {
  log << std::endl
      << std::endl
      << "Trying function number " << function_num << std::endl;
  areArgumentsConvertibleFromMex(
      nrhs, prhs, 0, &log, (static_cast<FromMexType<Args0> *>(nullptr))...);
  collectMexTryCallFunctionsErrorDiagnostics(nrhs, prhs, log, function_num + 1,
                                             functions...);
}

/**
 * mexTryToCallFunctions
 * keep trying functions in order until one is called without error.
 * throws a runtime_error with a diagnostic message upon failure
 */
// base case
bool mexTryToCallFunctions(int nlhs, mxArray *plhs[], int nrhs,
                           const mxArray *prhs[], bool throw_on_error) {
  return false;  // no functions to try, so return failure
}

// recursive case
template <typename Func0, typename... Funcs>
bool mexTryToCallFunctions(int nlhs, mxArray *plhs[], int nrhs,
                           const mxArray *prhs[], bool throw_on_error,
                           Func0 function0, Funcs... functions) {
  bool success = mexCallFunction(nlhs, plhs, nrhs, prhs, false,
                                 function0);  // try the first one
  if (!success) {  // recurse to try the other options
    success = mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, false,
                                    functions...);  // don't throw immediately,
                                                    // wait until all options
                                                    // exhausted
    if (throw_on_error && !success) {
      std::ostringstream log;
      log << "mexTryToCallFunctions: failed to find a function to call after "
             "trying " << sizeof...(Funcs) + 1 << " options. Errors:";
      collectMexTryCallFunctionsErrorDiagnostics(nrhs, prhs, log, 1, function0,
                                                 functions...);
      throw std::runtime_error(log.str());
    }
  }
  return success;
}

/**
 * fromMex
 * For if you only want to convert a single argument.
 * First calls isConvertible, then calls fromMexUnsafe.
 * Throws a runtime_error if not convertible.
 */
template <typename T>
auto fromMex(const mxArray *source, T *trigger_type)
    -> decltype(fromMexUnsafe(source, trigger_type)) {
  if (isConvertibleFromMex(source, trigger_type, nullptr)) {
    return fromMexUnsafe(source, trigger_type);
  } else {
    std::ostringstream log;
    isConvertibleFromMex(source, trigger_type, &log);
    throw std::runtime_error(log.str());
  }
}
