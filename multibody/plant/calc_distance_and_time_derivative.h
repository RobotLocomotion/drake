#pragma once

#include "drake/common/sorted_pair.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {

/**
 * Given a pair of geometries and the generalized position/velocity of the
 * plant, compute the signed distance betwen the pair of geometries, and the
 * time derivative of the signed distance.
 * @param plant The plant on which the geometries are attached. This plant must
 * have been connected to a SceneGraph.
 * @param geometry_pair The pair of geometries whose distance and time
 * derivative are computed.
 * @param context The context of the plant. This must store both q and v.
 * @param[out] distance The signed distance between the pair of geometry.
 * @param[out] distance_time_derivative The time derivative of the signed
 * distance.
 */
void CalcDistanceAndTimeDerivative(
    const multibody::MultibodyPlant<double>& plant,
    const SortedPair<geometry::GeometryId>& geometry_pair,
    const systems::Context<double>& context, double* distance,
    double* distance_time_derivative);
}  // namespace multibody
}  // namespace drake
