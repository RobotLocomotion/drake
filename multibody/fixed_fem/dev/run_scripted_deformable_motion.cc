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
bazel-bin/multibody/fixed_fem/dev/run_scripted_deformable_motion
-simulator_target_realtime_rate 1
```

Notice that without the realtime flag the simulation is likely to progress too
fast to visualize. */

#include <cstdlib>
#include <memory>

#include <gflags/gflags.h>

#include "drake/geometry/proximity/make_box_mesh.h"
#include "drake/multibody/fixed_fem/dev/deformable_visualizer.h"
#include "drake/multibody/fixed_fem/dev/dummy_softsim_system.h"
#include "drake/systems/analysis/simulator_gflags.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace multibody {
namespace fixed_fem {

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
      1.0 / 30.0, dummy_softsim_system->get_names(),
      dummy_softsim_system->get_meshes());
  builder.Connect(*dummy_softsim_system, visualizer);
  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();
  auto simulator =
      systems::MakeSimulatorFromGflags(*diagram, std::move(context));
  simulator->AdvanceTo(10);
  return 0;
}
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake

int main(int argc, char** argv) {
  gflags::SetUsageMessage("Test for deformable visualizer.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::multibody::fixed_fem::DoMain();
}
