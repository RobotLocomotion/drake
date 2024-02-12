#pragma once

#include <ostream>

#include "drake/common/drake_deprecated.h"
#include "drake/common/fmt_ostream.h"
#include "drake/common/hash.h"

// N.B. The namespaces in this file are weird (to simplify deprecation).

namespace drake {
namespace deprecated {
namespace internal {

template <typename T>
struct Color {
  T r;
  T g;
  T b;

  bool operator==(const Color<T>& other) const {
    return this->r == other.r && this->g == other.g && this->b == other.b;
  }

  template <class HashAlgorithm>
  friend void hash_append(HashAlgorithm& hasher, const Color& item) noexcept {
    using drake::hash_append;
    hash_append(hasher, item.r);
    hash_append(hasher, item.g);
    hash_append(hasher, item.b);
  }
};

template <typename T>
std::ostream& operator<<(std::ostream& out, const Color<T>& color) {
  out << "(" << color.r << ", " << color.g << ", " << color.b << ")";
  return out;
}

}  // namespace internal
}  // namespace deprecated
}  // namespace drake

namespace std {
template <typename T>
struct hash<drake::deprecated::internal::Color<T>> : public drake::DefaultHash {
};
}  // namespace std

namespace fmt {
template <typename T>
struct formatter<drake::deprecated::internal::Color<T>>
    : drake::ostream_formatter {};
}  // namespace fmt

namespace drake {
namespace systems {
namespace sensors {

template <typename T>
using Color DRAKE_DEPRECATED("2024-05-01", "This class is being removed") =
    drake::deprecated::internal::Color<T>;

using ColorI DRAKE_DEPRECATED("2024-05-01", "This class is being removed") =
    drake::deprecated::internal::Color<int>;

using ColorD DRAKE_DEPRECATED("2024-05-01", "This class is being removed") =
    drake::deprecated::internal::Color<double>;

}  // namespace sensors
}  // namespace systems
}  // namespace drake
