#pragma once

#include <optional>
#include <string>
#include <string_view>
#include <utility>

namespace drake {
namespace multibody {
namespace internal {

// This class allows us to create unordered maps that accept both std::string
// and std::string_view as keys in query operations. It encapsulates the two
// types and provides implicit conversions from those types to this type. It
// operates in one of two modes:
//
// 1. String-owning - the key maintains its own string value which is guaranteed
// to live as long as the key itself. When creating an entry in the map, it is
// critical that the key own its own storage.
// 2. Non-owning - this is the light-weight, non-allocating mode which will
// typically participate in performing look ups in the map (although keys in
// both modes will serve equally well). The external string that the non-owning
// key references must remain alive at least as long as the key.
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
