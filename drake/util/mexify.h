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

class MexToCppConversionError : public std::runtime_error {
public:
  MexToCppConversionError(const std::string& msg) : std::runtime_error(msg) { };

  MexToCppConversionError(const char *msg) : std::runtime_error(msg) { };
};

/**
 * fromMex
 */
int fromMex(const mxArray* source, int*) {
  if (mxGetM(source) != 1 || mxGetN(source) != 1)
    throw MexToCppConversionError("Expected scalar.");
  return static_cast<int>(mxGetScalar(source));
}

bool fromMex(const mxArray* source, bool*) {
  if (!mxIsLogicalScalar(source))
    throw MexToCppConversionError("Expected logical.");
  return mxGetLogicals(source)[0];
}

/**
 * toMex
 */
template <typename T>
void toMex(const T& source, mxArray* dest[], int nlhs);

/**
 * mexCallFunction
 */
class MexCallFunctionError : public std::runtime_error {
public:
  MexCallFunctionError(const std::string& msg) : std::runtime_error(msg) { };

  MexCallFunctionError(const char *msg) : std::runtime_error(msg) { };
};

// base case
template <typename R>
void mexCallFunction(std::function<R(void)> func, int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[], int nrhs_already_processed) {
  // call function and convert to mxArray
  toMex(func(), plhs, nlhs);
}

// void return type specialization
template <>
void mexCallFunction(std::function<void(void)> func, int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[], int nrhs_already_processed) {
  func();
}

// recursive case
template<typename R, typename Arg0, typename ...Args>
void mexCallFunction(std::function<R(Arg0, Args...)> func, int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[], int nrhs_already_processed = 0)
{
  // check number of input arguments
  const int expected_num_args = sizeof...(Args) + 1;
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

template <typename Func0, typename ...Funcs>
void mexTryToCallFunctions(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[], Func0 function0, Funcs... functions)
{
  try {
    mexCallFunction(function0, nlhs, plhs, nrhs, prhs);
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

template <typename Func0>
void mexTryToCallFunctions(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[], Func0 function0)
{
  try {
    mexCallFunction(function0, nlhs, plhs, nrhs, prhs);
  }
  catch (MexCallFunctionError& e) {
    MexTryToCallFunctionError options_exhausted_error;
    options_exhausted_error.addMexCallFunctionError(e);
    throw options_exhausted_error;
  }
}


#endif //DRAKE_MEXIFY_H
