#pragma once

#include <optional>
#include <string>

namespace drake {
namespace test {

/* RAII to temporarily change an environment variable. */
class SetEnv {
 public:
  SetEnv(const std::string& var_name, std::optional<std::string> var_value)
      : var_name_(var_name) {
    const char* orig = std::getenv(var_name.c_str());
    if (orig != nullptr) {
      orig_value_ = orig;
    }
    if (var_value.has_value()) {
      ::setenv(var_name.c_str(), var_value->c_str(), 1);
    } else {
      ::unsetenv(var_name.c_str());
    }
  }

  ~SetEnv() {
    if (orig_value_.has_value()) {
      ::setenv(var_name_.c_str(), orig_value_->c_str(), 1);
    } else {
      ::unsetenv(var_name_.c_str());
    }
  }

 private:
  std::string var_name_;
  std::optional<std::string> orig_value_;
};

}  // namespace test
}  // namespace drake
