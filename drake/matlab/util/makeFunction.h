#pragma once

#include <functional>

namespace drake {
/**
* make_function
* Note that a completely general make_function implementation is not possible
* due to ambiguities, but this works for all of the cases in this file
* Inspired by http://stackoverflow.com/a/21740143/2228557
*/

// plain function pointers
template <typename... Args, typename ReturnType>
auto make_function(ReturnType (*p)(Args...))
    -> std::function<ReturnType(Args...)> {
  return {p};
}

// nonconst member function pointers
// note the ClassType& as one of the arguments, which was erroneously omitted in
// the SO answer above
// also note the conversion to mem_fn, needed to work around an issue with MSVC
// 2013
template <typename... Args, typename ReturnType, typename ClassType>
auto make_function(ReturnType (ClassType::*p)(Args...))
    -> std::function<ReturnType(ClassType&, Args...)> {
  return {std::mem_fn(p)};
}

// const member function pointers
// note the const ClassType& as one of the arguments, which was erroneously
// omitted in the SO answer above
// also note the conversion to mem_fn, needed to work around an issue with MSVC
// 2013
template <typename... Args, typename ReturnType, typename ClassType>
auto make_function(ReturnType (ClassType::*p)(Args...) const)
    -> std::function<ReturnType(const ClassType&, Args...)> {
  return {std::mem_fn(p)};
}
}
