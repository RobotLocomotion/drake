///
/// @brief  A discrete mass-spring system example.
///

#include <cstdlib>
#include <limits>
#include <memory>

#include <gflags/gflags.h>

#include "drake/examples/mass_spring_system/mass_spring_system.h"
#include "drake/examples/mass_spring_system/mass_spring_system_geometry.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"

DEFINE_int64(nx, 20, "Number of points in the x direction");
DEFINE_int64(ny, 20, "Number of points in the y direction");
DEFINE_double(dx, 0.05, "Separation between neighboring points");
DEFINE_bool(run_discrete, true, "Whether to run discrete update");
DEFINE_double(dt, 1.0 / 100.0, "Time step size for discrete system");
DEFINE_double(realtime_rate, 1.0,
              "Rate at which to run the simulation, relative to realtime");
DEFINE_double(simulation_time, std::numeric_limits<double>::infinity(),
              "How long to simulate the system");

namespace drake {
namespace examples {
namespace mass_spring_system {
namespace {
/// A sample diagram for visualizing a cloth modeled as a discrete mass-spring
/// systems.
class MassSpringCloth : public systems::Diagram<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MassSpringCloth)

  /// A constructor that sets up the states of the mass-spring system.
  ///
  /// @param[in] nx number of points in the x-direction.
  /// @param[in] ny number of points in the y-direction.
  /// @param[in] dx in m units.
  explicit MassSpringCloth(int nx, int ny, double dx, bool is_discrete,
                           double dt);

  /// Creates a context using AllocateContext() and sets
  /// state variables according to the size of the system supplied.
  ///
  /// @param[in] nx number of points in the x-direction.
  /// @param[in] ny number of points in the y-direction.
  /// @param[in] dx in m units is the spacing between neighboring points.
  /// @param[in] is_discrete whether the system uses discrete update.
  /// @return a newly created Context.
  std::unique_ptr<systems::Context<double>> CreateContext(
      int nx, int ny, double dx, bool is_discrete) const;

 private:
  /// Creates a rectangular grid of nx by ny points and initialize their
  /// velocity to 0.
  ///
  /// @param[out] x positions of the points.
  /// @param[out] v velocities of the points.
  /// @param[in] nx number of points in the x-direction.
  /// @param[in] ny number of points in the y-direction.
  /// @param[in] dx in m units is the spacing between neighboring points.
  template <class VectorType>
  void InitializePositionAndVelocity(VectorType* x, VectorType* v, int nx,
                                     int ny, double dx) const {
    for (int i = 0; i < nx; ++i) {
      for (int j = 0; j < ny; ++j) {
        int node_index = i * nx + j;
        (*x)[3 * node_index] = i * dx;
        (*x)[3 * node_index + 1] = j * dx;
        (*x)[3 * node_index + 2] = 0.0;
        (*v)[3 * node_index] = 0.0;
        (*v)[3 * node_index + 1] = 0.0;
        (*v)[3 * node_index + 2] = 0.0;
      }
    }
  }
};

MassSpringCloth::MassSpringCloth(int nx, int ny, double dx, bool is_discrete,
                                 double dt) {
  // Building diagram.
  systems::DiagramBuilder<double> builder;
  // Adding the mass spring system.
  auto mass_spring_system =
      builder.AddSystem<MassSpringSystem<double>>(nx, ny, dx, is_discrete, dt);
  // Adding visualization.
  auto scene_graph = builder.AddSystem<geometry::SceneGraph>();
  MassSpringSystemGeometry::AddToBuilder(
      &builder, mass_spring_system->get_output_port(0), scene_graph, nx * ny);
  ConnectDrakeVisualizer(&builder, *scene_graph);
  builder.BuildInto(this);
}

std::unique_ptr<systems::Context<double>> MassSpringCloth::CreateContext(
    int nx, int ny, double dx, bool is_discrete) const {
  // Allocate context.
  auto context = this->AllocateContext();
  if (is_discrete) {
    // Extract mutable position and velocity states and initialize.
    auto state_values = context->get_mutable_discrete_state()
                            .get_mutable_vector()
                            .get_mutable_value();
    auto x = state_values.head(3 * nx * ny);
    auto v = state_values.tail(3 * nx * ny);
    InitializePositionAndVelocity(&x, &v, nx, ny, dx);
  } else {
    // Extract mutable position and velocity states and initialize.
    systems::VectorBase<double>& x = context->get_mutable_continuous_state()
                                         .get_mutable_generalized_position();
    systems::VectorBase<double>& v = context->get_mutable_continuous_state()
                                         .get_mutable_generalized_velocity();
    InitializePositionAndVelocity(&x, &v, nx, ny, dx);
  }
  return context;
}

///
/// Main function for demo.
///
int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "A simple demonstration of a cloth modeled as a mass-spring system");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  // Instantiate discrete example system.
  auto system = std::make_unique<MassSpringCloth>(FLAGS_nx, FLAGS_ny, FLAGS_dx,
                                                  FLAGS_run_discrete, FLAGS_dt);
  // Get context with initial conditions.
  auto context =
      system->CreateContext(FLAGS_nx, FLAGS_ny, FLAGS_dx, FLAGS_run_discrete);
  // Instantiate and configure simulator.
  auto simulator =
      std::make_unique<systems::Simulator<double>>(*system, std::move(context));
  simulator->set_target_realtime_rate(FLAGS_realtime_rate);
  simulator->Initialize();
  // Run simulation.
  simulator->AdvanceTo(FLAGS_simulation_time);
  return 0;
}

}  // namespace
}  // namespace mass_spring_system
}  // namespace examples
}  // namespace drake

int main(int argc, char** argv) {
  return drake::examples::mass_spring_system::main(argc, argv);
}
