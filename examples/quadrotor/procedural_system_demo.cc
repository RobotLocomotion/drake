/// @file
///
/// This demo sets up a controlled quadrotor that uses a simple procedural
/// script to move the vehicle between stable hovering fixed points.  It was
/// chosen to demonstrate that the procedural code can live in harmony with a
/// high-bandwidth control task.

#include <memory>
#include <thread>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/common/is_approx_equal_abstol.h"
#include "drake/examples/quadrotor/quadrotor_geometry.h"
#include "drake/examples/quadrotor/quadrotor_plant.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_scope_system.h"
#include "drake/systems/primitives/matrix_gain.h"
#include "drake/systems/primitives/multiplexer.h"

DEFINE_double(simulation_real_time_rate, 1.0, "Real time rate");
DEFINE_double(trial_duration, 13.2, "Duration of execution of each trial");

namespace drake {

using Eigen::Vector3d;
using systems::DiagramBuilder;
using systems::Simulator;
using systems::Context;
using systems::ContinuousState;
using systems::VectorBase;

namespace examples {
namespace quadrotor {
namespace {

// Demonstrates a simple(?) multi-threading approach to writing a procedural
// script in the systems framework.
//
// Design notes:
// This class spawns a thread on an initialization event to run the procedural
// script.  Conceptually, only one of the two threads is active at once; the
// SystemThread notifies the ScriptThread on each call to
// DoCalcDiscreteVariableUpdates, the Script runs until it calls
// WaitForNextContext(), which then blocks and notifies the main simulation
// thread to continue.
//
// This class has undeclared state -- specifically the thread stack of
// the Script thread.  It has every opportunity to produce deterministic
// simulations, but the outputs are *not* only a function of the Context but of
// the entire history from simulator initialization.  See #4330 for more
// discussion.
//
// The main virtue of this approach is that one can write full branching logic
// in the typical procedural fashion, and work with the code using standard
// tools like debugging breakpoints.  It should be natural to support
// T=AutoDiffXd, but not symbolic::Expression.
class ProceduralSystem : public systems::LeafSystem<double> {
 public:
  using UpdateArgs = std::pair<const systems::Context<double>*,
                               systems::DiscreteValues<double>*>;

  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ProceduralSystem)

  ProceduralSystem(double time_step)
      : script_thread_(&ProceduralSystem::Script, this) {
    this->DeclarePeriodicDiscreteUpdate(time_step);
    this->DeclareInitializationPublishEvent(&ProceduralSystem::Initialize);
  }

  ~ProceduralSystem() override {}

 protected:
  virtual void DoScript(std::unique_lock<std::mutex>*) = 0;

  // This should *only* be called from the ScriptThread.  It blocks until the
  // next call to CalcDiscreteVariableUpdates(), and makes the necessary
  // arguments of that method available in the ScriptThread.  The returned
  // pointers will only remain valid until the lock is released (which should
  // only happen at the next call to WaitForUpdate or on termination of the
  // script).
  UpdateArgs WaitForUpdate(std::unique_lock<std::mutex>* lock) {
    // TODO: Make it hard for the user script to change the lock status outside
    // of this method.
    if (!lock->owns_lock()) {
      lock->lock();
    }
    // Mark that we've finished with any previous data.
    context_ = nullptr;
    discrete_state_ = nullptr;

    // Unlock before notify to avoid SystemThread immediately blocking.
    lock->unlock();
    condition_variable_.notify_one();

//    std::cout << "in script, waiting for system. " << std::endl;

    // Wait until the SystemThread notifies me.
    lock->lock();
    condition_variable_.wait(*lock, [&] { return context_ != nullptr; });

//    std::cout << "in script, got new context." << std::endl;

    DRAKE_DEMAND(discrete_state_ != nullptr);
    return UpdateArgs(context_, discrete_state_);
  }

 private:
  void DoCalcDiscreteVariableUpdates(
      const systems::Context<double>& context,
      const std::vector<const systems::DiscreteUpdateEvent<double>*>& events,
      systems::DiscreteValues<double>* discrete_state) const override {
    // Send the update arguments to the ScriptThread.
    {
        std::lock_guard<std::mutex> lock(mutex_);
        context_ = &context;
        discrete_state_ = discrete_state;
    }
//    std::cout << "in system, notifying script" << std::endl;
    condition_variable_.notify_one();

    {
        std::unique_lock<std::mutex> lock(mutex_);
        // TODO: Handle the case where the thread throws or simply terminates.
        condition_variable_.wait(lock, [&]{ return context_ == nullptr; });
//        std::cout << "in system, returned from script" << std::endl;
    }
  }

  void Script() { 
      std::unique_lock<std::mutex> lock(mutex_, std::defer_lock);
      DoScript(&lock);
  }

  systems::EventStatus Initialize(const Context<double>&) const {
    // TODO: figure out how to start thread here, instead of the constructor,
    // despite the thread needed `this` to be movable.
    return systems::EventStatus::Succeeded();
  }

  // We must violate the constness of the DoCalcDiscreteVariableUpdates() by
  // making the following variables mutable.
  mutable std::thread script_thread_{};
  mutable std::mutex mutex_{};
  mutable std::condition_variable condition_variable_{};
  mutable const systems::Context<double>* context_{nullptr};
  mutable systems::DiscreteValues<double>* discrete_state_{nullptr};
};

// A script to move the quadrotor between hovering fixed points, waiting for
// the state of the quadrotor to converge before going to the next step.
//
// @system
// name: MyProceduralSystem
// input_ports:
// - quadrotor_state
// output_ports:
// - desired_xyz_position
// @endsystem
class MyProceduralSystem final : public ProceduralSystem {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MyProceduralSystem)

  // You must declare at least one discrete state group.  You should not declare
  // any continuous state.
  MyProceduralSystem() : ProceduralSystem(0.1) {
    this->DeclareInputPort("quadrotor_state", systems::kVectorValued, 12);
    auto state_index = this->DeclareDiscreteState(3);
    this->DeclareStateOutputPort("desired_xyz_position", state_index);
  }

  ~MyProceduralSystem() override {}

 private:
  void MoveTo(const Vector3d& desired, double threshold,
              std::unique_lock<std::mutex>* lock) {
    const systems::Context<double>* context;
    systems::DiscreteValues<double>* discrete_state;
    Vector3d x;
    std::cout << "moving to " << desired.transpose() << std::endl;
    do {
      std::tie(context, discrete_state) = WaitForUpdate(lock);
      discrete_state->get_mutable_vector().set_value(desired);
      x = this->get_input_port().Eval(*context).head<3>();
//      std::cout << "t = " << context->get_time()
//                << ", err = " << (x - desired).norm() << std::endl;
    } while ((x - desired).norm() > threshold);
  }

  void DoScript(std::unique_lock<std::mutex>* lock) override {
    // Do *not* modify the lock yourself.  It should only be passed into
    // WaitForUpdate().  TODO: Enforce this programmatically with encapsulation,
    // but for now this is more readable.

    // Feel free to do some initialization here.

    // WaitForUpdate blocks until these variables are available.
    auto [context, discrete_state] = WaitForUpdate(lock);

    // The entire Context is now available, including time, state, parameters,
    // and any input ports.  But don't hold on to any references!  They will
    // become stale at the next WaitForUpdate.  (We could wrap the
    // Context to delete any methods that return internal if we really want to
    // protect people from this).
    std::cout << "Starting script with Context:" << std::endl;
    std::cout << *context << std::endl;

    // The discrete state is mutable here.  This should be the sole "output"
    // from a script. Output ports should only pull values from that
    // discrete state (note that it would be natural to extend this to
    // unrestricted updates).
    discrete_state->get_mutable_vector().set_value(Vector3d{0, 0, 1.0});

    // Go ahead and put your branching logic, and your breakpoints in here!
    for (int loop=0; loop<3; ++loop) {
      MoveTo(Vector3d{0, 0, 1}, 0.1, lock);
      if (loop==1) {
        MoveTo(Vector3d{0, 2, 1}, 0.1, lock);
        MoveTo(Vector3d{2, 0, 1}, 0.1, lock);
      } else {
        MoveTo(Vector3d{0, 1, 1}, 0.1, lock);
        MoveTo(Vector3d{1, 0, 1}, 0.1, lock);
      }
    }

    // When you're done, the discrete state will simply stay constant for the
    // remainder.
  }
};

int do_main() {
  lcm::DrakeLcm lcm;

  DiagramBuilder<double> builder;

  auto quadrotor = builder.AddSystem<QuadrotorPlant<double>>();
  quadrotor->set_name("quadrotor");

  // Set the controller to stabilize around zero, u = u0 + K*(0 - x).
  // Then pass (x-x_d) into the controller, to achieve u = u0 + K*(x_d - x),
  // with a x_d as an input.
  auto controller = builder.AddSystem(StabilizingLQRController(
      quadrotor, Vector3d::Zero()));
  controller->set_name("controller");
  builder.Connect(controller->get_output_port(), quadrotor->get_input_port(0));

  // output = [x[0:2] - x_d; x[3:end]]
  Eigen::Matrix<double, 12, 15> G = Eigen::Matrix<double, 12, 15>::Identity();
  G.block<3,3>(0,12) = -Eigen::Matrix3d::Identity();
  //std::cout << G << std::endl;
  auto gain = builder.AddSystem<systems::MatrixGain<double>>(G);
  builder.Connect(gain->get_output_port(), controller->get_input_port());
  auto mux =
      builder.AddSystem<systems::Multiplexer<double>>(std::vector<int>{12, 3});
  builder.Connect(mux->get_output_port(), gain->get_input_port());
  builder.Connect(quadrotor->get_output_port(), mux->get_input_port(0));

  // Now add the procedural script system and wire it up.
  auto procedural = builder.AddSystem<MyProceduralSystem>();
  procedural->set_name("procedural");
  builder.Connect(procedural->get_output_port(), mux->get_input_port(1));
  builder.Connect(quadrotor->get_output_port(),
                  procedural->get_input_port());

  // Set up visualization
  auto scene_graph = builder.AddSystem<geometry::SceneGraph>();
  QuadrotorGeometry::AddToBuilder(
      &builder, quadrotor->get_output_port(0), scene_graph);
  geometry::DrakeVisualizerd::AddToBuilder(&builder, *scene_graph, &lcm);

  // Debug
  systems::lcm::LcmScopeSystem::AddToBuilder(
      &builder, &lcm, procedural->get_output_port(), "procedural", 0.1);

  auto diagram = builder.Build();
  Simulator<double> simulator(*diagram);

  Vector<double, 12> x0 = Vector<double, 12>::Zero();
  x0[0] = .5;
  x0[1] = .5;  

  simulator.get_mutable_context()
      .get_mutable_continuous_state_vector()
      .SetFromVector(x0);

  simulator.set_target_realtime_rate(FLAGS_simulation_real_time_rate);
  simulator.AdvanceTo(FLAGS_trial_duration);

  std::cout << simulator.get_context()
      .get_continuous_state_vector().CopyToVector().head<3>().transpose() << std::endl;
  
  return 0;
}

}  // namespace
}  // namespace quadrotor
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::quadrotor::do_main();
}
