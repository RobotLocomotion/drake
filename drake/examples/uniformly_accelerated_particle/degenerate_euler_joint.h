#pragma once

#include <Eigen/Core>

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
  namespace particles {

    /// @ brief A
    ///
    /// @tparam T must be a valid Eigen ScalarType
    /// @tparam N must be an integer in the (0, 6) range
    ///
    /// - Input:
    ///    - NDOF position (input indexes 0 to N-1)
    ///    - NDOF velocity (input indexes N to 2N-1)
    /// - Output:
    ///    - 6DOF position (output indexes 0 to 5)
    ///    - 6DOF velocity (output indexes 6 to 11)
    ///
    /// @note{
    /// Instantiated templates for the following combinations are
    /// provided 
    ///   - @p T is double, @p NDOF equals 1
    ///
    /// To use other other specific combinations, see degenerate_euler_joint-inl.h
    /// }
    template <typename T, int NDOF>
    class DegenerateEulerJoint : public systems::LeafSystem<T> {
      static_assert(NDOF > 0, "Cannot have negative degrees of freedom!");
      static_assert(NDOF < 6, "Cannot have more than 5 degrees of freedom!");
    public:
      using ScalarType = T;
      using TranslatingMatrix = Eigen::Matrix<T, 6, NDOF>;
      
      DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DegenerateEulerJoint)

      /// Constructor for the DegenerateEulerJoint system
      ///
      /// @param translator 6xDOF matrix to translate inputs to outputs
      explicit DegenerateEulerJoint(const TranslatingMatrix& translator);

    protected:
      void DoCalcOutput(const systems::Context<T>& context,
			systems::SystemOutput<T>* output) const override;

    private:
      TranslatingMatrix translator_; // Translating matrix 
    };
    
  }  // namespace particles
}   // namespace drake
