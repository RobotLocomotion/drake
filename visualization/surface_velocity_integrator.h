#pragma once

#include <string>
#include <vector>

#include "drake/geometry/meshcat_visualizer.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/bus_value.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/discrete_values.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace visualization {

/** Integrates instantaneous surface speeds into cumulative displacements for
meshcat surface-velocity visualization.

This system reads per-body surface speeds from a BusValue input port and
accumulates them into scalar displacements over time (displacement += speed *
dt at each discrete update). The resulting displacements are output as a
BusValue keyed by body scoped name, suitable for connection to
MeshcatVisualizer's `surface_displacement_input_port`.

 @system
 name: SurfaceVelocityIntegrator
 input_ports:
 - surface_speeds
 output_ports:
 - surface_displacements
 @endsystem

Use AddToBuilder() to wire this system into a diagram alongside an existing
MeshcatVisualizer.

This system is only available for `double` scalar type (not AutoDiff or
symbolic) because meshcat visualization is double-only.

@ingroup visualization */
class SurfaceVelocityIntegrator final : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SurfaceVelocityIntegrator);

  /** Constructs an integrator for the given `body_names`.

   @param body_names  Scoped names of the bodies with surface velocity (e.g.
                      `"model::belt"`). Each must be a distinct string.
   @param period      Discrete update period in seconds. Should match the
                      MeshcatVisualizer publish period. */
  SurfaceVelocityIntegrator(std::vector<std::string> body_names, double period);

  ~SurfaceVelocityIntegrator() final;

  /** Returns the `surface_speeds` input port (BusValue<double>). Signal keys
   are body scoped names; values are the scalar surface speed in m/s. Missing
   signals are treated as zero speed. */
  const systems::InputPort<double>& surface_speeds_input_port() const {
    return this->get_input_port(surface_speeds_input_port_index_);
  }

  /** Returns the `surface_displacements` output port (BusValue<double>).
   Signal keys are body scoped names; values are the cumulative surface
   displacement in meters. */
  const systems::OutputPort<double>& surface_displacements_output_port() const {
    return this->get_output_port(surface_displacements_output_port_index_);
  }

  /** Adds a %SurfaceVelocityIntegrator to `builder`, connects
   `surface_speeds_port` to its speed input, and connects its displacement
   output to `visualizer`'s `surface_displacement_input_port`.

   @param builder              The diagram builder.
   @param plant                A finalized MultibodyPlant. Bodies registered
                               via SetSurfaceVelocityAxis() are discovered
                               automatically.
   @param surface_speeds_port  An output port producing BusValue<double> with
                               per-body surface speeds. Typically the same port
                               connected to
   plant.get_surface_speeds_input_port().
   @param visualizer           The MeshcatVisualizer to receive displacements.
   @param period               Discrete update period (seconds). Should match
                               `visualizer`'s publish period.
   @returns a reference to the newly added system. */
  static SurfaceVelocityIntegrator& AddToBuilder(
      systems::DiagramBuilder<double>* builder,
      const multibody::MultibodyPlant<double>& plant,
      const systems::OutputPort<double>& surface_speeds_port,
      geometry::MeshcatVisualizer<double>& visualizer, double period);

 private:
  void DiscreteUpdate(const systems::Context<double>& context,
                      systems::DiscreteValues<double>* updates) const;

  void CalcOutputBus(const systems::Context<double>& context,
                     systems::BusValue* output) const;

  std::vector<std::string> body_names_;
  double period_{};
  int surface_speeds_input_port_index_{};
  int surface_displacements_output_port_index_{};
  systems::DiscreteStateIndex state_index_;
};

}  // namespace visualization
}  // namespace drake
