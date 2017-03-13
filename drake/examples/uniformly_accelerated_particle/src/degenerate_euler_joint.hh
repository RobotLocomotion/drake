#ifndef DEGENERATE_EULER_JOINT_HH
#define DEGENERATE_EULER_JOINT_HH

#include <Eigen/Core>
#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
  namespace particles {

    template <typename T, int DOF>
    class DegenerateEulerJoint : public systems::LeafSystem<T> {
    public:
      typedef Eigen::Matrix<T, 6, DOF> TranslatingMatrix;
      
    public:
      DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DegenerateEulerJoint)
      
      DegenerateEulerJoint();
      DegenerateEulerJoint(const TranslatingMatrix& translator);

      void DoCalcOutput(const systems::Context<T>& context,
			systems::SystemOutput<T>* output) const override;
    private:
      TranslatingMatrix translator_;
    };
    
  }  // namespace particles
}   // namespace drake

#endif  // DEGENERATE_EULER_JOINT_HH
