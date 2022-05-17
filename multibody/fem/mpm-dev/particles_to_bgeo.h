#pragma once

#include <string>
#include <vector>

#include "drake/common/eigen_types.h"

namespace drake {
namespace multibody {
namespace mpm {
namespace internal {

/* Writes particle information to file.
 @param[in] filename  Absolute path to the file.
 @param[in] q  Positions of the particles.
 @param[in] v  Velocities of the particles.
 @param[in] m  Masses of the particles.
 @pre q.size() == v.size() == m.size().
 @throws exception if the file with `filename` cannot be written to. */
void WriteParticlesToBgeo(const std::string& filename,
                          const std::vector<Vector3<double>>& q,
                          const std::vector<Vector3<double>>& v,
                          const std::vector<double>& m);

/* Reads particle information from file
 @param[in] filename  Absolute path to the file.
 @param[out] q  Positions of the particles.
 @param[out] v  Velocities of the particles.
 @param[out] m  Masses of the particles.
 @pre q, v, and m are not nullptrs.
 @throws exception if the file with `filename` doesn't exist. */
void ReadParticlesFromBgeo(const std::string& filename,
                           std::vector<Vector3<double>>* q,
                           std::vector<Vector3<double>>* v,
                           std::vector<double>* m);

}  // namespace internal
}  // namespace mpm
}  // namespace multibody
}  // namespace drake
