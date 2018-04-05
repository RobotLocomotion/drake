#pragma once

#include "drake/systems/framework/diagram_builder.h"

/** @file
 Provides a set of functions to facilitate visualization operations based on
 geometry world state. */

namespace drake {
namespace geometry {

template <typename T> class GeometrySystem;

/** Configures the diagram to interface with drake_visualizer. This should be
 invoked as the _last_ thing before building the diagram; registration of all
 frames and geometries should be done, but not context allocated.
 @param system      The system whose geometry will be sent in an LCM message.
 @param builder     The diagram builder to which the system belongs; additional
                    systems will be added to enable visualization updates.
 @throws std::logic_error if the system has already had its context allocated.
 */
void ConfigureVisualization(const GeometrySystem<double>& system,
                            systems::DiagramBuilder<double>* builder);

}  // namespace geometry
}  // namespace drake
