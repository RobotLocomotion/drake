#pragma once
#include <string>
#include <utility>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
/** A dummy softsim system that moves registered geometries in sinusoidal
 motion. */
class DummySoftsimSystem final : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DummySoftsimSystem)

  /** Construct a DummySoftSimSystem. */
  DummySoftsimSystem() {
    this->DeclareAbstractOutputPort(
        "vertex_positions", &DummySoftsimSystem::CopyVertexPositionsOut);
  }

  /** Adds a body modeled with the given geometry to the system.
   @param[in] amplitude The amplitude of the sinusoidal motion for the new
   body.
   @param[in] velocity The velocity of the sinusoidal motion for the new body.
   @param[in] p_WM The origin of the input mesh in world frame. */
  void RegisterBody(const geometry::VolumeMesh<double>& mesh, std::string name,
                    double amplitude, double velocity,
                    const Vector3<double>& p_WM) {
    const std::vector<geometry::VolumeVertex<double>>& verts = mesh.vertices();
    std::vector<geometry::VolumeElement> elements = mesh.tetrahedra();
    std::vector<geometry::VolumeVertex<double>> verts_W;
    /* Shift the vertices to world frame. */
    for (const auto& v : verts) {
      verts_W.emplace_back(v.r_MV() + p_WM);
    }
    meshes_.emplace_back(std::move(elements), std::move(verts_W));
    names_.emplace_back(std::move(name));
    amplitudes_.emplace_back(amplitude);
    velocities_.emplace_back(velocity);
  }

  /** Get the abstract-valued port containing an `std::vector` of vector
   values for the vertex positions of each body. */
  const systems::OutputPort<double>& get_vertex_positions_output_port() const {
    return systems::System<double>::get_output_port(0);
  }

  int num_bodys() const { return meshes_.size(); }

  /** Get the volume meshes of the registered bodies. */
  const std::vector<geometry::VolumeMesh<double>>& get_meshes() const {
    return meshes_;
  }

  /** Get the names of all the registered bodys. */
  const std::vector<std::string>& get_names() const { return names_; }

 private:
  /* Copies the generalized positions of each model to the given `output`.
   The order of the model follows that of soft_body_indexes. */
  void CopyVertexPositionsOut(const systems::Context<double>& context,
                              std::vector<VectorX<double>>* output) const {
    output->resize(num_bodys());
    const double t = context.get_time();
    for (int i = 0; i < num_bodys(); ++i) {
      const std::vector<geometry::VolumeVertex<double>> vertices =
          meshes_[i].vertices();
      const int num_vertices = meshes_[i].num_vertices();
      VectorX<double> q(num_vertices * 3);
      const double v = velocities_[i];
      const double h = amplitudes_[i];
      for (int j = 0; j < num_vertices; ++j) {
        Vector3<double> r_MV = vertices[j].r_MV();
        r_MV(2) += h * std::sin(frequency_ * (r_MV(0) + v * t));
        q.template segment<3>(3 * j) = r_MV;
      }
      (*output)[i] = q;
    }
  }

  /* The amplitude of the sinusoidal motion for each body. */
  std::vector<double> amplitudes_{};
  /* The velocity of the sinusoidal motion for each body. */
  std::vector<double> velocities_{};
  /* The frequency of the sinusoidal wave with unit m⁻¹. */
  const double frequency_{30};
  std::vector<geometry::VolumeMesh<double>> meshes_{};
  std::vector<std::string> names_{};
};
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
