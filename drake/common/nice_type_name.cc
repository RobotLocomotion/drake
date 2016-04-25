/* Portions copyright (c) 2014 Stanford University and the Authors.         
   Authors: Chris Dembia, Michael Sherman
                                                                         
Licensed under the Apache License, Version 2.0 (the "License"); you may  
not use this file except in compliance with the License. You may obtain a
copy of the License at http://www.apache.org/licenses/LICENSE-2.0. */                                           

#include "drake/common/nice_type_name.h"

#include <string>
#include <regex>
#include <array>
#include <algorithm>

using std::string;

// __GNUG__ is defined both for gcc and clang. See:
// https://gcc.gnu.org/onlinedocs/libstdc++/libstdc++-html-USERS-4.3/a01696.html
#if defined(__GNUG__)
  #include <cxxabi.h>
  #include <cstdlib>
#endif

// On gcc and clang typeid(T).name() returns an indecipherable mangled string
// that requires processing to become human readable. Microsoft returns a
// reasonable name directly.
string drake::common::DemangleTypeName(const char* typeid_name) {
  #if defined(__GNUG__)
    int status=-100; // just in case it doesn't get set
    char* ret = abi::__cxa_demangle(name, NULL, NULL, &status);
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
string drake::common::CanonicalizeTypeName(string&& demangled) {
  using SPair = std::pair<std::regex,string>;
  // These are applied in this order.
  static const std::array<SPair,7> subs{
    // Remove unwanted keywords and following space.
    SPair(std::regex("\\b(class|struct|enum|union) "),      ""),
    // Tidy up anonymous namespace.
    SPair(std::regex("[`(]anonymous namespace[')]"),        "(anonymous)"),
    // Replace Microsoft __int64 with long long.
    SPair(std::regex("\\b__int64\\b"),                      "long long"),
    // Temporarily replace spaces we want to keep with "!". (\w is 
    // alphanumeric or underscore.)
    SPair(std::regex("(\\w) (\\w)"),    "$1!$2"),
    SPair(std::regex(" "),              ""), // Delete unwanted spaces.
    // OSX clang throws in extra namespaces like "__1". Delete them.
    SPair(std::regex("\\b__[0-9]+::"),  ""),
    SPair(std::regex("!"),              " ") // Restore wanted spaces.
  };
  string canonical(std::move(demangled));
  for (const auto& sp : subs) {
    canonical = std::regex_replace(canonical, sp.first, sp.second);
  }
  return canonical;
}
