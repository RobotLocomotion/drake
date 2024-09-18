#pragma once

#include <functional>
#include <map>
#include <string>

namespace drake {

/** Like `std::map<std::string, T>`, but with better defaults than the plain
`std::map<std::string, T>` spelling. We need `std::less<void>` as the comparison
function so that `std::string_view` and `const char*` can be used as lookup keys
without copying them to a `std::string`. */
template <typename T>
class string_map : public std::map<std::string, T, std::less<void>> {
 public:
  template <class K>
  T& operator[](K&& x) {
    auto iter = this->find(x);
    if (iter != this->end()) {
        return iter->second;
    }
    return this->emplace(std::move(x), T{}).first->second;
  }

  template <class K>
  const T& at(const K& x) const {
    auto iter = this->find(x);
    if (iter != this->end()) {
        return iter->second;
    }
    throw std::out_of_range("string_map::at");
  }
};

/** Like `std::multimap<std::string, T>`, but with better defaults than the
plain `std::multimap<std::string, T>` spelling. We need `std::less<void>` as the
comparison function so that `std::string_view` and `const char*` can be used as
lookup keys without copying them to a `std::string`. */
template <typename T>
using string_multimap = std::multimap<std::string, T, std::less<void>>;

}  // namespace drake
