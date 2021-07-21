#pragma once

#include <memory>
#include <string>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/lcm/drake_lcm_interface.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace multibody {
namespace fem {

/** A class for visualizing deformable meshes in `drake_visualizer`.
 Specifically, it dispatches LCM messages to initialize mesh definitions and
 then updates vertex positions at a fixed frequency. Although the inputs are
 volume (aka tetrahedral) meshes, only the surface triangular mesh gets
 visualized.
 @system
 name: DeformableVisualizer
 input_ports:
 - vertex_positions
 @endsystem
 The input port is an abstract-valued port containing
 std::vector<VectorX<double>>. There is one VectorX for each mesh with which the
 visualizer was instantiated. The ith mesh corresponds to the ith VectorX. The
 ith VectorX has 3N doubles where N + 1 is the largest vertex index referenced
 by the ith tetrahedral mesh. For mesh i, the x-, y-, and z-positions (measured
 and expressed in the world frame) of the jth vertex are 3j, 3j + 1, and 3j + 2
 in the ith VectorX from the input port. */
class DeformableVisualizer : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DeformableVisualizer)

  /** Constructs a visualizer for a vector of tetrahedral meshes.
   @param update_period  The duration (in seconds) between publications of mesh
                         state.
   @param mesh_names     The names for the meshes (as they will appear in
                         drake_visualizer).
   @param tet_meshes     The definition of the tetrahedral mesh topologies.
   @param lcm            If non-nullptr, `this` will use the provided lcm
                         interface to broadcast lcm messages. The system will
                         keep a reference to the instance and it should stay
                         alive as long as this class. Otherwise, `this` will
                         create its own instance.
   @pre update_period > 0.
   @pre mesh_names.size() == tet_meshes.size().  */
  DeformableVisualizer(
      double update_period, std::vector<std::string> mesh_names,
      const std::vector<geometry::VolumeMesh<double>>& tet_meshes,
      lcm::DrakeLcmInterface* lcm = nullptr);

  // TODO(xuchenhan-tri): Rethink the initialization/update paradigm as the
  //  initialization event may be received after update events.
  /** Send the mesh initialization message. This can be invoked explicitly but
   is generally not necessary. The initialization method is also called by
   an initialization event.  */
  void SendMeshInit() const;

  /** Returns the input port for vertex positions.  */
  const systems::InputPort<double>& vertex_positions_input_port() const {
    return systems::System<double>::get_input_port(vertex_positions_port_);
  }

 private:
  /* Analyzes the tet mesh topologies to do the following:
    1. Build a surface mesh from each volume mesh.
    2. Create a mapping from surface vertex to volume vertex for each mesh.
    3. Record the expected number of vertices referenced by each tet mesh.  */
  void AnalyzeTets(const std::vector<geometry::VolumeMesh<double>>& tet_meshes);

  /* Call SendMeshInit in an initialization event.  */
  systems::EventStatus PublishMeshInit(const systems::Context<double>&) const;

  /* Broadcast the current vertex positions for the mesh.  */
  void PublishMeshUpdate(const systems::Context<double>& context) const;

  /* The LCM interface used to broadcast its messages. This system can
   optionally own its lcm interface.  */
  lcm::DrakeLcmInterface* lcm_{};
  std::unique_ptr<lcm::DrakeLcmInterface> owned_lcm_{};

  /* The names of the meshes. Included in all lcm messages.  */
  const std::vector<std::string> mesh_names_;

  /* An *implicit* map from *surface* vertex indices to volume vertex indices.
   For the iᵗʰ mesh, the jᵗʰ surface vertex corresponds to the volume vertex
   with index `surface_to_volume_vertices_[i][j]`.  */
  std::vector<std::vector<int>> surface_to_volume_vertices_;

  /* Surface meshes representing the topology of the volume meshes' surfaces. */
  std::vector<std::vector<Vector3<int>>> surface_triangles_;

  /* The total number of vertices expected on the input port as implied by the
   tetrahedra definitions.  */
  std::vector<int> volume_vertex_counts_{};

  /* Port Indexes.  */
  systems::InputPortIndex vertex_positions_port_{};
};
}  // namespace fem
}  // namespace multibody
}  // namespace drake
