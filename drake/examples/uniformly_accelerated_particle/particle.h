#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
  namespace particles {

    /// A linear 1DOF particle system implementation.
    ///
    /// @tparam T must be a valid Eigen ScalarType
    ///
    /// - Input:
    ///     - linear acceleration (input index 0), in m/s^2 units
    /// - Output/State: 
    ///     - linear position (state/output index 0), in m units
    ///     - linear velocity (state/output index 1), in m/s units
    ///  
    /// @note{
    /// Instantiated templates for the following scalar types @p T
    /// are provided:
    /// - double
    ///
    /// To use other specific scalar types see particle-inl.h
    /// }
    /// 
    template <typename T>
    class Particle : public systems::LeafSystem<T> {
    public:
      using ScalarType = T;

      DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Particle)

      /// Constructor for the Particle system.
      Particle();

    protected:
      void DoCalcOutput(const systems::Context<T>& context,
			systems::SystemOutput<T>* output) const override;
      
      void DoCalcTimeDerivatives(const systems::Context<T>& context,
				 systems::ContinuousState<T>* derivatives) const override;      
    };

  }  // namespace particles
}  // namespace drake
