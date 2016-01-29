#ifndef _ATLAS_UTIL_H_
#define _ATLAS_UTIL_H_

#include <memory>
#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/drakeAtlasUtil_export.h"

/*
 * The ankle limits is specified by a set of halfspace constraint 
 * A*[akx;aky]<=b
 * ankleCloseToLimits will return true if any row of 
 * A*[akx;aky]-b is greater than -tol
 */
DRAKEATLASUTIL_EXPORT bool ankleCloseToLimits(double akx, double aky, double tol);

enum class DRAKEATLASUTIL_EXPORT AtlasHand {
  NONE,
  ROBOTIQ,
  ROBOTIQ_WEIGHT
};

struct DRAKEATLASUTIL_EXPORT AtlasKinematicOptions {
  AtlasKinematicOptions():
    collision_groups_to_keep({"heel", "toe"}),
    hands({{Side::RIGHT, AtlasHand::NONE},
           {Side::LEFT, AtlasHand::NONE}}) {
      // empty
  }

  std::map<Side, AtlasHand> hands;
  std::set<std::string> collision_groups_to_keep;
};

DRAKEATLASUTIL_EXPORT std::unique_ptr<RigidBodyTree> constructAtlasV5(std::unique_ptr<RigidBodyTree> robot, const RobotPropertyCache& rpc, const AtlasKinematicOptions& options=AtlasKinematicOptions());
DRAKEATLASUTIL_EXPORT std::unique_ptr<RigidBodyTree> constructAtlasV5(const std::string& urdf_filename, const std::string& control_config_filename, const AtlasKinematicOptions& options=AtlasKinematicOptions());

#endif
