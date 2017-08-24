#pragma once

#include "drake/geometry/identifier.h"

namespace drake {
namespace multibody {

/// Type used to identify Link objects in MultibodyModeler.
using LinkId = geometry::Identifier<class LinkTag>;

/// Type used to identify Joint objects in MultibodyModeler.
using JointId = geometry::Identifier<class JointTag>;

}  // namespace multibody
}  // namespace drake
