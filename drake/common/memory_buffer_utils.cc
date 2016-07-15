#include "drake/common/memory_buffer_utils.h"

#include <iomanip>
#include <sstream>
#include <string>

#include "drake/common/drake_assert.h"

using std::string;
using std::stringstream;

namespace drake {

std::string MemoryBufferToString(const uint8_t* const buffer,
    int buffer_length) {
  DRAKE_ABORT_UNLESS(buffer);
  stringstream string_stream;
  for (int ii = 0; ii < buffer_length; ++ii) {
    string_stream << "0x" << std::uppercase << std::setfill('0') << std::setw(4)
                  << std::hex << buffer[ii] << " ";
  }
  return string_stream.str();
}

}  // namespace drake

