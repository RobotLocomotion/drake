#pragma once

#include <optional>
#include <string>
#include <string_view>
#include <utility>

namespace drake {
namespace multibody {
namespace internal {

// This class exists because unordered_map in C++17 does not allow using
// different keys for storage and lookup. This class serves two purposes:
// 1. It stores a string and provides a view into it. This is necessary
//    for insertion into the hashtable because string_view acts only as
//    a pointer.
// 2. It stores a string_view only. This permits querying the hashtable
//    with a string_view without copying the underlying string. Note that
//    the underlying memory pointed to by the string_view must outlive
//    `this` and all copies in this mode.
class StringViewMapKey {
 public:
  StringViewMapKey() = default;

  // Default move constructor may not respect short-string optimization.
  // See note in move assignment operator below.
  StringViewMapKey(StringViewMapKey&& s) { operator=(std::move(s)); }
  StringViewMapKey(const StringViewMapKey& s) { operator=(s); }

  StringViewMapKey& operator=(const StringViewMapKey& s) {
    // Note: We don't need to guard against self-assigment here, since we're not
    // managing resources.
    storage_ = s.storage_;
    if (storage_.has_value()) {
      // s is used for Mode 1.
      view_ = storage_.value();
    } else {
      // s is used for Mode 2.
      view_ = s.view();
    }
    return *this;
  }

  // @note default move assignment may fail when short-string-optimization
  //       (SSO) is used by the underlying std::string because the moved view
  //       would still point to the array in the source.
  // @note Contents of `s` will be undefined after move.
  StringViewMapKey& operator=(StringViewMapKey&& s) {
    // Note: We don't need to guard against self-assigment here, since we're not
    // managing resources.
    storage_ = std::move(s.storage_);
    view_ = (storage_.has_value()) ? storage_.value() : s.view();
    return *this;
  }

  // Mode 1 constructor with implicit conversion.
  // NOLINTNEXTLINE
  StringViewMapKey(std::string&& s)
      : storage_(std::move(s)),
        view_(storage_.value()) {}

  // Mode 1 constructor with implicit conversion.
  // NOLINTNEXTLINE
  StringViewMapKey(const std::string& s)
      : storage_(s), view_(storage_.value()) {}

  // Mode 2 constructor with implicit conversion.
  // NOLINTNEXTLINE
  StringViewMapKey(std::string_view view) : view_(view) {}

  std::string_view view() const { return view_; }

  // Underlying optional storage, exposed for testing.
  const std::optional<std::string>& storage() const { return storage_; }

 private:
  std::optional<std::string> storage_;
  std::string_view view_;

  friend bool operator==(const StringViewMapKey& a, const StringViewMapKey& b) {
    return a.view_ == b.view_;
  }
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake

namespace std {
template<> struct hash<drake::multibody::internal::StringViewMapKey> {
std::size_t operator()(
  const drake::multibody::internal::StringViewMapKey& s) const noexcept {
      return std::hash<std::string_view>{}(s.view());
}
};
}  // namespace std
