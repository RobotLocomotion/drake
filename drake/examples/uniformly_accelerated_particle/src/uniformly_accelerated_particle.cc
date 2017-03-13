#include <memory>
#include <limits>
#include <stdlib.h>

#include "drake/common/drake_path.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/parsers/sdf_parser.h"

#include "drake/examples/uniformly_accelerated_particle/src/particle.hh"
#include "drake/examples/uniformly_accelerated_particle/src/degenerate_euler_joint.hh"

DEFINE_double(initial_position, 0.0, "Particle initial x position");
DEFINE_double(initial_velocity, 0.0, "Particle initial x velocity");
DEFINE_double(acceleration, 1.0, "Particle constant x acceleration");
DEFINE_double(realtime_rate, 1.0, "Rate at which to run the simulation, relative to realtime");
DEFINE_double(simulation_time, std::numeric_limits<double>::infinity(), "How long to simulate the particle");

namespace drake {
  namespace particles {
    
    template <typename T>
    class UniformlyAcceleratedParticle : public systems::Diagram<T> {
    public:
      DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(UniformlyAcceleratedParticle)

      UniformlyAcceleratedParticle(T acceleration, std::shared_ptr< lcm::DrakeLcmInterface > lcm);
      ~UniformlyAcceleratedParticle() override;

      void SetInitialConditions(T position, T velocity);
      void Initialize();
    private:
      Particle<T>* particle_;
      DegenerateEulerJoint<T, 1>* joint_;
      systems::DrakeVisualizer* vis_;
      systems::ConstantVectorSource<T>* asrc_;
      std::shared_ptr< lcm::DrakeLcmInterface > lcm_;
      std::unique_ptr< RigidBodyTree<T> > tree_{
	std::make_unique< RigidBodyTree<T> >()};
    };

    template <typename T>
    UniformlyAcceleratedParticle<T>::UniformlyAcceleratedParticle(
	T acceleration, std::shared_ptr< lcm::DrakeLcmInterface > lcm)
      : lcm_(lcm) {
      // build world
      static const std::string particle_sdf =
	GetDrakePath() + "/examples/uniformly_accelerated_particle/models/particle.sdf";
      parsers::sdf::AddModelInstancesFromSdfFileToWorld(
	particle_sdf, multibody::joints::kRollPitchYaw, tree_.get());
      tree_->compile();  // one more time to be sure
      // build system
      systems::DiagramBuilder<T> builder;
      asrc_ = builder.template AddSystem< systems::ConstantVectorSource<T> >(
	acceleration);
      particle_ = builder.template AddSystem< Particle<T> >();
      typename DegenerateEulerJoint<T, 1>::TranslatingMatrix m =
	DegenerateEulerJoint<T, 1>::TranslatingMatrix::Zero(); m(0, 0) = 1.0;
      joint_ = builder.template AddSystem < DegenerateEulerJoint<T, 1> >(m);
      vis_ = builder.template AddSystem< systems::DrakeVisualizer >(*tree_, lcm_.get());
      builder.Connect(*asrc_, *particle_);
      builder.Connect(*particle_, *joint_);
      builder.Connect(*joint_, *vis_);
      builder.BuildInto(this);
    }

    template <typename T>
    UniformlyAcceleratedParticle<T>::~UniformlyAcceleratedParticle() {
      lcm_.reset();
    }
    
    template <typename T>
    void UniformlyAcceleratedParticle<T>::SetInitialConditions(T position, T velocity) {
      particle_->SetInitialConditions(position, velocity);
    }

    int main (int argc, char* argv[]) {
      gflags::ParseCommandLineFlags(&argc, &argv, true);
      logging::HandleSpdlogGflags();
      // intantiate interface and spin it up
      auto interface = std::make_shared< lcm::DrakeLcm >();
      interface->StartReceiveThread();
      // instantiate and configure system
      auto system = std::make_unique < UniformlyAcceleratedParticle<double> >(
	FLAGS_acceleration, interface);
      system->SetInitialConditions(FLAGS_initial_position, FLAGS_initial_velocity);
      // instantiate and configure simulator
      auto simulator = std::make_unique < systems::Simulator<double> >(*system);
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
