#ifndef PARTICLE_HH
#define PARTICLE_HH

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
  namespace particles {

    template <typename T>
    class Particle : public systems::LeafSystem<T> {
    public:
      DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Particle)
      // Constructor for the Particle system.
      Particle();

      void SetInitialConditions(const T& position, const T& velocity);
      
      void DoCalcOutput(const systems::Context<T>& context,
			systems::SystemOutput<T>* output) const override;
      
      void DoCalcTimeDerivatives(const systems::Context<T>& context,
				 systems::ContinuousState<T>* derivatives) const override;
      
      void SetDefaultState(const systems::Context<T>& context,
			   systems::State<T>* state) const override;

    private:
      Vector2<T> ic_{Vector2<T>::Zero()};
    };

  }  // namespace particles
}  // namespace drake

#endif  // PARTICLE_HH
