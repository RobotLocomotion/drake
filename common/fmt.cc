#include "drake/common/fmt.h"

namespace drake {

template <typename T>
std::string fmt_floating_point(T x)
  requires(std::is_same_v<T, float> || std::is_same_v<T, double>)
{
  std::string result = fmt::format("{:#}", x);
  if (result.back() == '.') {
    result.push_back('0');
  }
  return result;
}

std::string fmt_debug_string(std::string_view x) {
  return fmt::format("{:?}", x);
}

template std::string fmt_floating_point<float>(float);
template std::string fmt_floating_point<double>(double);

}  // namespace drake
