#include "drake/common/memory_buffer_utils.h"

#include <iomanip>
#include <sstream>
#include <string>

#include "drake/common/drake_assert.h"

using std::string;
using std::stringstream;

namespace drake {

// Defines the number of bytes to include on a line when creating the string
// representation of a memory buffer.
const int kNumValuesPerLine = 10;

std::string MemoryBufferToString(const uint8_t* const buffer,
    int buffer_length) {
  DRAKE_ABORT_UNLESS(buffer);
  stringstream string_stream;
  for (int ii = 0; ii < buffer_length; ++ii) {
    string_stream << "0x" << std::uppercase << std::setfill('0') << std::setw(2)
                  << std::hex << static_cast<unsigned>(buffer[ii]);
    if (ii > 0 && (ii + 1) % kNumValuesPerLine == 0)
      string_stream << "\n";
    else
      string_stream << " ";
  }
  return string_stream.str();
}

}  // namespace drake

