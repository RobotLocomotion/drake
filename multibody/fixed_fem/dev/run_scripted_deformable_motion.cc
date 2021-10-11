/**
A demo to test deformable visualizer.

To run the demo. First ensure that you have the visualizer and the demo itself
built:

```
bazel build //tools:drake_visualizer
bazel build //multibody/fixed_fem/dev:run_scripted_deformable_motion
```

Then, in one terminal, launch the visualizer
```
bazel-bin/tools/drake_visualizer
```

In another terminal, launch the demo
```
bazel-bin/multibody/fixed_fem/dev/run_scripted_deformable_motion \
    --simulator_target_realtime_rate=1
```

Notice that without the realtime flag the simulation is likely to progress too
fast to visualize. */

#include <cstdlib>
#include <memory>

#include <gflags/gflags.h>

#include "drake/geometry/proximity/make_box_mesh.h"
#include "drake/multibody/fixed_fem/dev/deformable_visualizer.h"
#include "drake/systems/analysis/simulator_gflags.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace multibody {
namespace fem {

/** A dummy softsim system that deforms registered geometry with a sinusoidal
 ripple motion. */
class DummySoftsimSystem final : public systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DummySoftsimSystem)

  /** Construct a DummySoftSimSystem. */
  DummySoftsimSystem() {
    this->DeclareAbstractOutputPort(
        "vertex_positions", &DummySoftsimSystem::CopyVertexPositionsOut);
  }

  /** Adds a geometry modeled with the given geometry to the system.
   @param[in] mesh The input tetrahedral mesh that describes the connectivity
   and the positions of the vertices that make up the geometry.
   @param[in] name Name of the newly added geometry.
   @param[in] amplitude The amplitude of the sinusoidal motion for the new
   geometry.
   @param[in] velocity The velocity of the sinusoidal motion for the new
   geometry.
   @param[in] p_WM The origin of the input mesh in world frame. */
  void RegisterBody(const geometry::VolumeMesh<double>& mesh, std::string name,
                    double amplitude, double velocity,
                    const Vector3<double>& p_WM) {
    const std::vector<Vector3<double>>& verts = mesh.vertices();
    std::vector<geometry::VolumeElement> elements = mesh.tetrahedra();
    std::vector<Vector3<double>> verts_W;
    /* Shift the vertices to world frame. */
    for (const auto& v : verts) {
      verts_W.emplace_back(v + p_WM);
    }
    meshes_.emplace_back(std::move(elements), std::move(verts_W));
    names_.emplace_back(std::move(name));
    amplitudes_.emplace_back(amplitude);
    velocities_.emplace_back(velocity);
  }

  /** Get the abstract-valued port containing an `std::vector` of `VectorX`
   values for the vertex positions of each geometry. */
  const systems::OutputPort<double>& get_vertex_positions_output_port() const {
    return systems::System<double>::get_output_port(0);
  }

  int num_geometries() const { return meshes_.size(); }

  /** Get the volume meshes of the registered geometries. */
  const std::vector<geometry::VolumeMesh<double>>& get_meshes() const {
    return meshes_;
  }

  /** Get the names of all the registered geometries. */
  const std::vector<std::string>& get_names() const { return names_; }

 private:
  /* Copies the generalized positions of each geometry to the given `output`. */
  void CopyVertexPositionsOut(const systems::Context<double>& context,
                              std::vector<VectorX<double>>* output) const {
    output->resize(num_geometries());
    const double t = context.get_time();
    for (int i = 0; i < num_geometries(); ++i) {
      const std::vector<Vector3<double>> vertices = meshes_[i].vertices();
      const int num_vertices = meshes_[i].num_vertices();
      VectorX<double> q(num_vertices * 3);
      const double v = velocities_[i];
      const double h = amplitudes_[i];
      for (int j = 0; j < num_vertices; ++j) {
        Vector3<double> r_MV = vertices[j];
        r_MV(2) += h * std::sin(frequency_ * (r_MV(0) + v * t));
        q.template segment<3>(3 * j) = r_MV;
      }
      (*output)[i] = q;
    }
  }

  /* The amplitude of the sinusoidal motion for each geometry. */
  std::vector<double> amplitudes_{};
  /* The velocity of the sinusoidal motion for each geometry. */
  std::vector<double> velocities_{};
  /* The frequency of the sinusoidal wave with unit m⁻¹. */
  const double frequency_{30};
  std::vector<geometry::VolumeMesh<double>> meshes_{};
  std::vector<std::string> names_{};
};

int DoMain() {
  systems::DiagramBuilder<double> builder;
  auto* dummy_softsim_system = builder.AddSystem<DummySoftsimSystem>();
  const double dx = 0.03;
  const int Nx = 20;
  const int Ny = 4;
  const int Nz = 4;
  const geometry::Box box(Nx * dx, Ny * dx, Nz * dx);
  const geometry::VolumeMesh<double> box_mesh =
      geometry::internal::MakeBoxVolumeMesh<double>(box, dx);
  dummy_softsim_system->RegisterBody(box_mesh, "box1", 0.03, 0.2, {0, 0, 0});
  dummy_softsim_system->RegisterBody(box_mesh, "box2", 0.015, 0.6,
                                       {0, 0.6, 0});

  auto& visualizer = *builder.AddSystem<DeformableVisualizer>(
      1.0 / 32.0, dummy_softsim_system->get_names(),
      dummy_softsim_system->get_meshes());
  builder.Connect(*dummy_softsim_system, visualizer);
  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();
  auto simulator =
      systems::MakeSimulatorFromGflags(*diagram, std::move(context));
  simulator->AdvanceTo(10);
  return 0;
}
}  // namespace fem
}  // namespace multibody
}  // namespace drake

int main(int argc, char** argv) {
  gflags::SetUsageMessage("Test for deformable visualizer.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::multibody::fem::DoMain();
}
