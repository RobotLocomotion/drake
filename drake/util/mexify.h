//
// Created by Twan Koolen on 8/22/15.
//

#ifndef DRAKE_MEXIFY_H
#define DRAKE_MEXIFY_H

#include <functional>
#include <sstream>
#include <stdexcept>
#include "mex.h"
#include "drakeMexUtil.h"

/**
 * Functions that make it easy to create a mex function given a C++ function signature in the form of a std::function.
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
 * Overloads for the fromMex (resp. toMex) function should be provided for each argument type (resp. return type).
 *
 * fromMex functions should be of the form
 *
 *   ArgumentType fromMex(const mxArray* source, TriggerType*)
 *
 * where TriggerType is the type of the argument currently being processed according to the signature
 * of the std::function that was passed in (with cv qualifiers and reference removed)
 * and ArgumentType is the type that will actually be passed into the function, which may or may not be the same as TriggerType.
 * romMex functions should throw a MexToCppConversionError if the 'source' argument cannot be processed
 *
 * toMex functions should be of the form
 *
 *   int toMex(const SourceType& source, mxArray* dest[], int nlhs)
 *
 * where SourceType is such that the return type of the std::function can be converted to it. The toMex function should return the number of output arguments it has set.
 */

class MexToCppConversionError : public std::runtime_error {
public:
  MexToCppConversionError(const std::string& msg) : std::runtime_error(msg) { };

  MexToCppConversionError(const char *msg) : std::runtime_error(msg) { };
};

class MexCallFunctionError : public std::runtime_error {
public:
  MexCallFunctionError(const std::string& msg) : std::runtime_error(msg) { };

  MexCallFunctionError(const char *msg) : std::runtime_error(msg) { };
};

/**
 * mexCallFunction
 */
// base case with return
template <typename R>
void mexCallFunction(std::function<R(void)> func, int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[], int nrhs_already_processed) {
  // call function and convert to mxArray
  toMex(func(), plhs, nlhs);
}

// base case without return
template <>
void mexCallFunction(std::function<void(void)> func, int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[], int nrhs_already_processed) {
  func();
}

// recursive case
template<typename R, typename Arg0, typename ...Args>
void mexCallFunction(std::function<R(Arg0, Args...)> func, int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[], int nrhs_already_processed = 0)
{
  // check number of input arguments
  const int expected_num_args = sizeof...(Args) + 1; // don't forget Arg0
  if (nrhs != expected_num_args) {
    std::ostringstream buf;
    buf << "Expected " << expected_num_args << " arguments, but got " << nrhs << ".";
    throw MexCallFunctionError(buf.str());
  }

  // bind the first argument
  auto partially_applied = [&](Args&& ...tail)
  {
    // remove reference because in general you don't want to return a reference from fromMex
    // remove const to have fewer template specializations
    typedef typename std::remove_cv<typename std::remove_reference<Arg0>::type>::type FromMexType;

    try {
      return std::move(func)(fromMex(prhs[0], (FromMexType*) nullptr), std::forward<Args>(tail)...);
    }
    catch (MexToCppConversionError& e) {
      std::ostringstream buf;
      buf << "Error processing argument number " << nrhs_already_processed + 1 << ". Reason:\n";
      buf << e.what();
      throw MexCallFunctionError(buf.str().c_str());
    }
  };

  // recursively call mexCallFunction with partially applied function
  mexCallFunction(std::function<R(Args...)>{partially_applied}, nlhs, plhs, nrhs - 1, &prhs[1], nrhs_already_processed + 1);
};

/*
 * mexTryToCallFunctions: keep trying functions in order until one is called without error.
 */
class MexTryToCallFunctionError : public std::exception {
private:
  std::vector<MexCallFunctionError> errors;
  mutable std::string message;

public:
  MexTryToCallFunctionError() : std::exception() { };

  void addMexCallFunctionError(const MexCallFunctionError& error)
  {
    errors.push_back(error);
  }

  virtual const char* what() const noexcept
  {
    std::ostringstream buf;
    std::string functions = errors.size() == 1 ? "function" : "functions";
    buf << "No suitable function to call after trying " << errors.size() << " " << functions << ". Errors:";
    for (auto it = errors.rbegin(); it != errors.rend(); ++it) {
      buf << std::endl << it->what();
    }
    message = buf.str();
    return message.c_str();
  }
};

// recursive case
template <typename Func0, typename ...Funcs>
void mexTryToCallFunctions(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[], Func0 function0, Funcs... functions)
{
  try {
    mexCallFunction(function0, nlhs, plhs, nrhs, prhs);
    // Debug:
    // std::cout << "mexTryToCallFunctions called function:\n" << typeid(function0).name() << std::endl;
  }
  catch (MexCallFunctionError& call_function_error) {
    try {
      mexTryToCallFunctions(nlhs, plhs, nrhs, prhs, functions...);
    }
    catch (MexTryToCallFunctionError& options_exhausted_error) {
      options_exhausted_error.addMexCallFunctionError(call_function_error);
      throw options_exhausted_error;
    }
  }
}

// base case: no more functions left to try
template <typename Func0>
void mexTryToCallFunctions(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[], Func0 function0)
{
  try {
    mexCallFunction(function0, nlhs, plhs, nrhs, prhs);
    // Debug:
    // std::cout << "mexTryToCallFunctions called function:\n" << typeid(function0).name() << std::endl;
  }
  catch (MexCallFunctionError& e) {
    MexTryToCallFunctionError options_exhausted_error;
    options_exhausted_error.addMexCallFunctionError(e);
    throw options_exhausted_error;
  }
}

#endif //DRAKE_MEXIFY_H
