#pragma once

#include <utility>
#include <variant>

/** @file
The "overloaded" variant-visit pattern.

The C++ std::visit (typeswitch dispatch) is very useful for std::variant but
doesn't support it natively.  There is a commonly used two-line boilerplate
that bridges this gap; see
https://en.cppreference.com/w/cpp/utility/variant/visit

This file should be included by classes that wish to use the variant visit
pattern, i.e.:

@code {.cpp}
  using MyVariant = std::variant<int, std::string>;
  MyVariant v = 5;
  std::string result = std::visit<const char*>(overloaded{
    [](const int arg) { return "found an int"; },
    [](const std::string& arg) { return "found a string"; }
  }, v);
  EXPECT_EQ(result, "found an int");
@endcode

However, note that the prior example DOES NOT WORK yet in Drake. In order to
support C++17 we must also (for now) use a polyfill for `std::visit<T>` which
we've named `visit_overloaded<T>`, so within Drake we must spell it like this:

@code {.cpp}
  using MyVariant = std::variant<int, std::string>;
  MyVariant v = 5;
  std::string result = visit_overloaded<const char*>(overloaded{
    [](const int arg) { return "found an int"; },
    [](const std::string& arg) { return "found a string"; }
  }, v);
  EXPECT_EQ(result, "found an int");
@endcode

When we drop support for C++17, we'll be able to return back to the normal
pattern.

@warning This file must never be included by a header, only by a cc file.
         This is enforced by a lint rule in `tools/lint/drakelint.py`.
*/

// We put this in the anonymous namespace to ensure that it will not generate
// linkable symbols for every specialization in every library that uses it.
namespace {  // NOLINT(build/namespaces)

// Boilerplate for `std::visit`; see
// https://en.cppreference.com/w/cpp/utility/variant/visit
template <class... Ts>
struct overloaded : Ts... {
  using Ts::operator()...;
};
template <class... Ts>
overloaded(Ts...) -> overloaded<Ts...>;

// NOTE:  The second line above can be removed when we are compiling with
// >= C++20 on all platforms.

// This is a polyfill for C++20's std::visit<Return>(visitor, variant) that we
// need while we still support C++17. Once we drop C++17 (i.e., once we drop
// Ubuntu 20.04), we should switch back to the conventional spelling and remove
// this entire block of code.
#if __cplusplus > 201703L
// On reasonable platforms, we can just call std::visit.
template <typename Return, typename Visitor, typename Variant>
auto visit_overloaded(Visitor&& visitor, Variant&& variant) -> decltype(auto) {
  return std::visit<Return>(std::forward<Visitor>(visitor),
                            std::forward<Variant>(variant));
}
#else
// On Focal, we need to do a polyfill.
template <typename Return, typename Visitor, typename Variant>
auto visit_overloaded(Visitor&& visitor, Variant&& variant) -> Return {
  auto visitor_coerced = [&visitor]<typename Value>(Value&& value) -> Return {
    return visitor(std::forward<Value>(value));
  };
  return std::visit(visitor_coerced, std::forward<Variant>(variant));
}
#endif

}  // namespace
