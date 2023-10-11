#pragma once

#include "drake/common/drake_deprecated.h"
#include "drake/common/package_map.h"

namespace drake {
namespace multibody {

using PackageMap DRAKE_DEPRECATED("2024-02-01",
                                  "Use drake::common::PackageMap instead.") =
    drake::PackageMap;

}  // namespace multibody
}  // namespace drake
