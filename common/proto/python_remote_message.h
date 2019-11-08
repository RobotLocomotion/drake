#pragma once

#include <cstring>
#include <string>
#include <vector>

#include "drake/common/drake_assert.h"

namespace drake {
namespace common {
namespace internal {

// Generic data structure to wrap Python data types. For historical reasons,
// interface is modeled on mxArray (see
// https://www.mathworks.com/help/matlab/matlab_external/matlab-data.html).
struct PythonRemoteData final {
  enum DataType {
    REMOTE_VARIABLE_REFERENCE = 0,
    DOUBLE = 1,
    CHAR = 2,
    LOGICAL = 3,
    INT = 4,
  };

  enum ShapeType {
    MATRIX = 0,
    VECTOR = 1,
    SCALAR = 2,
  };

  // Sets our `data` member to a memory range.
  void set_data(const void* p, int num_bytes) {
    DRAKE_DEMAND(num_bytes >= 0);
    data.resize(num_bytes);
    std::memcpy(data.data(), p, num_bytes);
  }

  ShapeType shape_type{SCALAR};
  DataType type{DOUBLE};
  int rows{0};
  int cols{0};
  std::vector<uint8_t> data;
};

// Message to support asynchronous remote procedure calls to a Python
// client. For historical reasons, interface is modeled on mexCallMATLAB
// (see https://www.mathworks.com/help/matlab/apiref/mexcallmatlab.html) but
// with output arguments assigned in the remote client workspace due to the
// asynchronous nature of the protocol.
struct PythonRemoteMessage final {
  // Unique id for variables held in the client workspace.
  std::vector<int64_t> lhs;

  // Input argument data.
  std::vector<PythonRemoteData> rhs;

  // Any expression that resolves to a callable Python object.
  std::string function_name;

  // Convert to MessagePack (aka msgpack) format, https://msgpack.org/.
  std::vector<uint8_t> ToMsgpack() const;
};

}  // namespace internal
}  // namespace common
}  // namespace drake
