#ifndef _ATLAS_UTIL_H_
#define _ATLAS_UTIL_H_

#include <memory>
#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/systems/controllers/QPCommon.h"
#include "drake/drakeAtlasUtil_export.h"
#include "drake/systems/robotInterfaces/Side.h"

namespace Atlas {
/*
 * The ankle limits is specified by a set of halfspace constraint 
 * A*[akx;aky]<=b
 * ankleCloseToLimits will return true if any row of 
 * A*[akx;aky]-b is greater than -tol
 */
DRAKEATLASUTIL_EXPORT bool ankleCloseToLimits(double akx, double aky, double tol);

DRAKEATLASUTIL_EXPORT void setupAtlas(std::unique_ptr<RigidBodyTree>& robot, const KinematicModifications& modifications=KinematicModifications());
DRAKEATLASUTIL_EXPORT std::unique_ptr<RigidBodyTree> constructAtlas(const std::string& urdf_filename, const KinematicModifications& modifications=KinematicModifications());
DRAKEATLASUTIL_EXPORT std::unique_ptr<RigidBodyTree> constructAtlas(const std::string& urdf_filename, const std::string& urdf_modifications_filename);

}

#endif
