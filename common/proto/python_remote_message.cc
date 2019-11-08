#include "drake/common/proto/python_remote_message.h"

#include "nlohmann/json.hpp"

#include "drake/common/drake_assert.h"

namespace drake {
namespace common {
namespace internal {

using json = nlohmann::json;

std::vector<uint8_t> PythonRemoteMessage::ToMsgpack() const {
  json result;

  result["lhs"] = lhs;
  for (const auto& x : rhs) {
    json item;
    item["shape_type"] = x.shape_type;
    item["type"] = x.type;
    item["rows"] = x.rows;
    item["cols"] = x.cols;
    item["data"] = x.data;
    result["rhs"].push_back(item);
  }
  result["function_name"] = function_name;

  return json::to_msgpack(result);
}

}  // namespace internal
}  // namespace common
}  // namespace drake
