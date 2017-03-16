#include <memory>
#include <limits>
#include <stdlib.h>

#include "drake/common/drake_path.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsers/sdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"

#include "drake/examples/uniformly_accelerated_particle/degenerate_euler_joint.h"
#include "drake/examples/uniformly_accelerated_particle/particle.h"

DEFINE_double(initial_position, 0.0, "Particle initial x position");
DEFINE_double(initial_velocity, 0.0, "Particle initial x velocity");
DEFINE_double(acceleration, 1.0, "Particle constant x acceleration");
DEFINE_double(realtime_rate, 1.0, "Rate at which to run the simulation, relative to realtime");
DEFINE_double(simulation_time, std::numeric_limits<double>::infinity(), "How long to simulate the particle");

namespace drake {
  namespace particles {

    /// Fixed path to particle SDF model (for visualization purposes only)
    static const char* particle_sdf = "/examples/uniformly_accelerated_particle/models/particle.sdf";
    
    /// A sample diagram for visualizing a 1DOF particle to which a
    /// a constant acceleration is applied.
    ///
    /// @tparam T must be a valid Eigen ScalarType
    template <typename T>
    class UniformlyAcceleratedParticle : public systems::Diagram<T> {
    public:
      DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(UniformlyAcceleratedParticle)
      
      /// Constructor for the UniformlyAcceleratedParticle
      /// system diagram
      ///
      /// @param[in] acceleration in m/s^2 units
      explicit UniformlyAcceleratedParticle(const T& acceleration);

      /// Creates a context using @ref AllocateContext() and sets
      /// state variables according to the initial conditions supplied
      ///
      /// @param[in] position in m units
      /// @param[in] velocity in m/s units
      /// @returns a unique_ptr to a Context instance
      std::unique_ptr<systems::Context<T>> CreateContext(const T& position, const T& velocity);

      /// Wires up the whole system diagram
      ///
      /// @param[in] lcm interface to be used for visualization
      ////               related messaging
      void BuildAndConnect(lcm::DrakeLcmInterface* lcm);
    private:
      T accel_;
      bool built_{false};
      std::unique_ptr< RigidBodyTree<T> > tree_{
	std::make_unique< RigidBodyTree<T> >()};
    };
    
    template <typename T>
    UniformlyAcceleratedParticle<T>::UniformlyAcceleratedParticle(const T& acceleration)
      : accel_(acceleration) {}
    
    template <typename T>
    void UniformlyAcceleratedParticle<T>::BuildAndConnect(lcm::DrakeLcmInterface* lcm) {
      DRAKE_DEMAND(!built_);
      // build rigid body tree
      parsers::sdf::AddModelInstancesFromSdfFileToWorld(
	GetDrakePath() + particle_sdf, multibody::joints::kRollPitchYaw, tree_.get());
      tree_->compile();  // one more time to be sure
      // build diagram
      systems::DiagramBuilder<T> builder;
      // add constant acceleration source
      auto asrc = builder.template AddSystem< systems::ConstantVectorSource<T> >(accel_);
      // add particle
      auto particle = builder.template AddSystem< Particle<T> >();
      // add joint
      typename DegenerateEulerJoint<T, 1>::TranslatingMatrix m =
	DegenerateEulerJoint<T, 1>::TranslatingMatrix::Zero(); m(0, 0) = 1.0;
      auto joint = builder.template AddSystem < DegenerateEulerJoint<T, 1> >(m);
      // add visualizer client
      auto vis = builder.template AddSystem< systems::DrakeVisualizer >(*tree_, lcm);
      // connect all blocks
      builder.Connect(*asrc, *particle);
      builder.Connect(*particle, *joint);
      builder.Connect(*joint, *vis);
      builder.BuildInto(this);
      built_ = true;
    }
    
    template <typename T>
    std::unique_ptr<systems::Context<T>> UniformlyAcceleratedParticle<T>::CreateContext(const T& position, const T& velocity) {
      DRAKE_DEMAND(built_); 
      // allocate context
      auto context = this->AllocateContext();
      // set continuous state
      systems::VectorBase<T>* cstate =
	context->get_mutable_continuous_state_vector();
      cstate->SetAtIndex(0, position);
      cstate->SetAtIndex(1, velocity);
      return context;
    }

    int main (int argc, char* argv[]) {
      gflags::ParseCommandLineFlags(&argc, &argv, true);
      logging::HandleSpdlogGflags();
      // instantiate interface and spin it up
      auto interface = std::make_unique< lcm::DrakeLcm >();
      interface->StartReceiveThread();
      // instantiate and configure system
      auto system = std::make_unique < UniformlyAcceleratedParticle<double> >(FLAGS_acceleration);
      system->BuildAndConnect(interface.get());
      // get context with initial conditions
      auto context = system->CreateContext(FLAGS_initial_position, FLAGS_initial_velocity);
      // instantiate and configure simulator
      auto simulator = std::make_unique < systems::Simulator<double> >(*system, std::move(context));
      simulator->set_target_realtime_rate(FLAGS_realtime_rate);
      simulator->Initialize();      
      // run simulation
      simulator->StepTo(FLAGS_simulation_time);
      return 0;
    }

  }  // namespace particles
}  // namespace drake

int main (int argc, char **argv) {
  return drake::particles::main(argc, argv);
}
