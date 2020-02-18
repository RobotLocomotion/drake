#pragma once

#include "drake/common/sorted_pair.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {

/**
 * The struct containing the signed distance and its time derivative between
 * a pair of geometries.
 */
struct SignedDistanceWithTimeDerivative {
  double distance;
  double distance_time_derivative;
};

/**
 * Given a pair of geometries and the generalized position/velocity of the
 * plant, compute the signed distance between the pair of geometries and the
 * time derivative of the signed distance.
 * This function is similar to
 * QueryObject::ComputeSignedDistancePairClosestPoints(), but it also provides
 * the time derivative of the signed distance.
 * @param plant The plant on which the geometries are attached. This plant must
 * have been connected to a SceneGraph.
 * @param geometry_pair The pair of geometries whose distance and time
 * derivative are computed.
 * @param context The context of the plant. This must store both q and v. This
 * context must have been extracted from the diagram context which contains
 * both MultibodyPlant and SceneGraph contexts.
 * @param[out] distance The signed distance between the pair of geometry.
 * @param[out] distance_time_derivative The time derivative of the signed
 * distance.
 */
SignedDistanceWithTimeDerivative CalcDistanceAndTimeDerivative(
    const multibody::MultibodyPlant<double>& plant,
    const SortedPair<geometry::GeometryId>& geometry_pair,
    const systems::Context<double>& context);
}  // namespace multibody
}  // namespace drake
