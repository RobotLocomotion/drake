#pragma once

/// Type wrapper that initializes a wrapped type to its default-constructed
/// value by default, and guarantees that when moving from this type, the donor
/// object is reset to its default-constructed value.
template <typename T>
class default_value {
 public:
  default_value() {}

  // NOLINTNEXTLINE(runtime/explicit)
  default_value(const T& value) : value_(value) {}

  // Copyable (default).
  default_value(const default_value&) = default;
  default_value& operator=(const default_value&) = default;

  // Moveable (custom) with a guarantee to reset the donor back to its
  // intrinsic value.
  default_value(default_value&& other) {
    value_ = other.value_;
    other.value_ = T{};
  }
  default_value& operator=(default_value&& other) {
    if (&other != this) {
      value_ = other.value_;
      other.value_ = T{};
    }
    return *this;
  }

  // Access operators to make it behave as the wrapped type.
  operator T&() { return value_; }
  operator const T&() const { return value_; }

 private:
  T value_{};
};
