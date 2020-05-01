/* Portions copyright (c) 2014 Stanford University and the Authors.
   Authors: Chris Dembia, Michael Sherman

Licensed under the Apache License, Version 2.0 (the "License"); you may
not use this file except in compliance with the License. You may obtain a
copy of the License at http://www.apache.org/licenses/LICENSE-2.0. */

#include "drake/common/nice_type_name.h"

#include <algorithm>
#include <array>
#include <initializer_list>
#include <regex>
#include <string>

#include "drake/common/never_destroyed.h"
#include "drake/common/nice_type_name_override.h"

using std::string;

// __GNUG__ is defined both for gcc and clang. See:
// https://gcc.gnu.org/onlinedocs/libstdc++/libstdc++-html-USERS-4.3/a01696.html
#if defined(__GNUG__)
/* clang-format off */
#include <cxxabi.h>
#include <cstdlib>
/* clang-format on */
#endif

namespace drake {

// On gcc and clang typeid(T).name() returns an indecipherable mangled string
// that requires processing to become human readable. Microsoft returns a
// reasonable name directly.
string NiceTypeName::Demangle(const char* typeid_name) {
#if defined(__GNUG__)
  int status = -100;  // just in case it doesn't get set
  char* ret = abi::__cxa_demangle(typeid_name, NULL, NULL, &status);
  const char* const demangled_name = (status == 0) ? ret : typeid_name;
  string demangled_string(demangled_name);
  if (ret) std::free(ret);
  return demangled_string;
#else
  // On other platforms, we hope the typeid name is not mangled.
  return typeid_name;
#endif
}

// Given a demangled string, attempt to canonicalize it for platform
// indpendence. We'll remove Microsoft's "class ", "struct ", etc.
// designations, and get rid of all unnecessary spaces.
string NiceTypeName::Canonicalize(const string& demangled) {
  using SPair = std::pair<std::regex, string>;
  using SPairList = std::initializer_list<SPair>;
  // These are applied in this order.
  static const never_destroyed<std::vector<SPair>> subs{SPairList{
    // Remove unwanted keywords and following space. (\b is word boundary.)
    SPair(std::regex("\\b(class|struct|enum|union) "), ""),
    // Tidy up anonymous namespace.
    SPair(std::regex("[`(]anonymous namespace[')]"), "(anonymous)"),
    // Replace Microsoft __int64 with long long.
    SPair(std::regex("\\b__int64\\b"), "long long"),
    // Temporarily replace spaces we want to keep with "!". (\w is
    // alphanumeric or underscore.)
    SPair(std::regex("(\\w) (\\w)"), "$1!$2"),
    SPair(std::regex(" "), ""),  // Delete unwanted spaces.
    // Some compilers throw in extra namespaces like "__1" or "__cxx11".
    // Delete them.
    SPair(std::regex("\\b__[[:alnum:]_]+::"), ""),
    SPair(std::regex("!"), " "),  // Restore wanted spaces.

    // Recognize std::string's full name and abbreviate.
    SPair(std::regex("\\bstd::basic_string<char,std::char_traits<char>,"
                     "std::allocator<char>>"),
          "std::string"),

    // Recognize Eigen types ...
    // ... square matrices ...
    SPair(std::regex("\\bEigen::Matrix<([^,<>]*),(-?\\d+),\\2,0,\\2,\\2>"),
          "drake::Matrix$2<$1>"),
    // ... vectors ...
    SPair(std::regex("\\bEigen::Matrix<([^,<>]*),(-?\\d+),1,0,\\2,1>"),
          "drake::Vector$2<$1>"),
    // ... dynamic-size is "X" not "-1" ...
    SPair(std::regex("drake::(Matrix|Vector)-1<"), "drake::$1X<"),
    // ... for double, float, and int, prefer native Eigen spellings ...
    SPair(std::regex("drake::(Vector|Matrix)(X|\\d+)"
                     "<((d)ouble|(f)loat|(i)nt)>"),
          "Eigen::$1$2$4$5$6"),
    // ... AutoDiff.
    SPair(std::regex("Eigen::AutoDiffScalar<Eigen::VectorXd>"),
          "drake::AutoDiffXd"),
  }};

  string canonical(demangled);
  for (const auto& sp : subs.access()) {
    canonical = std::regex_replace(canonical, sp.first, sp.second);
  }
  return canonical;
}

string NiceTypeName::RemoveNamespaces(const string& canonical) {
  // Removes everything up to and including the last "::" that isn't part of a
  // template argument. If the string ends with "::", returns the original
  // string unprocessed. We are depending on the fact that namespaces can't be
  // templatized to avoid bad behavior for template types where the template
  // argument might include namespaces (those should not be touched).
  static const never_destroyed<std::regex> regex{"^[^<>]*::"};

  const std::string no_namespace =
      std::regex_replace(canonical, regex.access(), "");

  return no_namespace.empty() ? canonical : no_namespace;
}

std::string NiceTypeName::GetWithPossibleOverride(
      const void* ptr, const std::type_info& info) {
  internal::NiceTypeNamePtrOverride ptr_override =
      internal::GetNiceTypeNamePtrOverride();
  if (ptr_override) {
    return ptr_override(internal::type_erased_ptr{ptr, info});
  } else {
    return Get(info);
  }
}

}  // namespace drake
