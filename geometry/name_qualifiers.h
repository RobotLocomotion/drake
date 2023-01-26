#pragma once

#include <cstdint>

namespace drake {
namespace geometry {

using NameQualifiers = uint8_t;
static constexpr NameQualifiers kQualifierFrame = 1;
static constexpr NameQualifiers kQualifierFrameGroup = 2;
static constexpr NameQualifiers kQualifierSource = 4;

}  // namespace geometry
}  // namespace drake
