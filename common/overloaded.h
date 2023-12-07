#pragma once

/** @file The "overloaded" variant-visit pattern.

The C++ std::visit (typeswitch dispatch) is very useful for std::variant but
doesn't support it natively.  There is a commonly used two-line boilerplate
that bridges this gap; see
https://en.cppreference.com/w/cpp/utility/variant/visit

This file should be included by classes that wish to
use the variant visit pattern, i.e.

@code {.cpp}
  using MyVariant = std::variant<int, std::string>;
  MyVariant v = 5;
  std::string result = std::visit(overloaded{
    [](const int arg) { return "found an int"; },
    [](const std::string& arg) { return "found a string"; }
  }, v);
  EXPECT_EQ(result, "found an int");
@endcode
*/

namespace drake {
// Boilerplate for `std::visit`; see
// https://en.cppreference.com/w/cpp/utility/variant/visit
template<class... Ts> struct overloaded : Ts... { using Ts::operator()...; };
template<class... Ts> overloaded(Ts...) -> overloaded<Ts...>;

// NOTE:  The second line above can be removed when we
// are compiling with >= C++20 on all platforms.
}  // namespace drake
