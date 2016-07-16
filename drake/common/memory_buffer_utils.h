/// @file
/// This file contains methods for working with memory buffers.

#pragma once

#include <string>

#include "drake/drakeCommon_export.h"

namespace drake {

/**
 * Creates a string representation of a memory buffer.
 *
 * @param[in] buffer The buffer to represent as a string. This pointer cannot
 * be nullptr.
 *
 * @param[in] buffer_length The number of bytes in the buffer.
 *
 * @return A string representation of the buffer.
 */
DRAKECOMMON_EXPORT
std::string MemoryBufferToString(const uint8_t* const buffer,
    int buffer_length);

}  // namespace drake
