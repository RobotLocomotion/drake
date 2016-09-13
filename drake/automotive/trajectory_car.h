#pragma once

#include <stdexcept>

#include "drake/drakeCars_export.h"
#include "drake/examples/Cars/curve2.h"
#include "drake/examples/Cars/system1_cars_vectors.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace cars {

/// A car that follows a pre-established trajectory, neglecting all physics.
///
/// state vector
/// * none
///
/// input vector:
/// * none
///
/// output vector (planar for now):
/// * position: x, y, heading;
///   heading is 0 rad when pointed +x, pi/2 rad when pointed +y;
//    heading is defined around the +z axis, so positive-turn-left
/// * velocity
///
class DRAKECARS_EXPORT TrajectoryCar1 {
 public:
  /// Constructs a TrajectoryCar system that traces the given @p curve,
  /// at the given constant @p speed, starting at the given @p start_time.
  /// Throws an error if the curve is empty (a zero @p path_length).
  TrajectoryCar1(const Curve2<double>& curve, double speed, double start_time)
      : curve_(curve), speed_(speed), start_time_(start_time) {
    if (curve_.path_length() == 0.0) {
      throw std::invalid_argument{"empty curve"};
    }
  }

  // Noncopyable.
  TrajectoryCar1(const TrajectoryCar1&) = delete;
  TrajectoryCar1& operator=(const TrajectoryCar1&) = delete;

  /// @name Implement the Drake System concept.
  //@{

  template <typename ScalarType>
  using StateVector = drake::NullVector<ScalarType>;
  template <typename ScalarType>
  using InputVector = drake::NullVector<ScalarType>;
  template <typename ScalarType>
  using OutputVector = SimpleCarState1<ScalarType>;

  template <typename ScalarType>
  StateVector<ScalarType> dynamics(const ScalarType&,
                                   const StateVector<ScalarType>&,
                                   const InputVector<ScalarType>&) const {
    // No state means no dynamics.
    return StateVector<ScalarType>{};
  }

  template <typename ScalarType>
  OutputVector<ScalarType> output(const ScalarType& time,
                                  const StateVector<ScalarType>&,
                                  const InputVector<ScalarType>&) const {
    // N.B. Never use InputVector data, because we are !isDirectFeedthrough.

    // Trace the curve at a fixed speed.
    const double distance = speed_ * (time - start_time_);
    const Curve2<double>::PositionResult pose = curve_.GetPosition(distance);

    // Convert pose to OutputVector.
    OutputVector<ScalarType> result{};
    result.set_x(pose.position[0]);
    result.set_y(pose.position[1]);
    result.set_heading(std::atan2(pose.position_dot[1], pose.position_dot[0]));
    result.set_velocity(speed_);
    return result;
  }

  bool isTimeVarying() const {
    // Our dynamics() does NOT depend on time.
    // Our output() DOES depend on time.
    return true;
  }

  bool isDirectFeedthrough() const {
    // Our output() does not depend our InputVector data.
    return false;
  }

  //@}

 private:
  const Curve2<double> curve_;
  const double speed_;
  const double start_time_;
};

/// A System2 wrapper around the System1 TrajectoryCar1.
template <typename T>
class TrajectoryCar : public systems::LeafSystem<T> {
 public:
  TrajectoryCar(const Curve2<T>& curve, double speed, double start_time)
      : wrapped_(curve, speed, start_time) {
    this->DeclareOutputPort(systems::kVectorValued,
                            SimpleCarStateIndices::kNumCoordinates,
                            systems::kContinuousSampling);
  }

  void EvalOutput(const systems::Context<T>& context,
                  systems::SystemOutput<T>* output) const override {
    DRAKE_ASSERT(output != nullptr);
    DRAKE_ASSERT(output->get_num_ports() == 1);
    systems::BasicVector<T>* output_vector =
        output->GetMutableVectorData(0);
    DRAKE_ASSERT(output_vector != nullptr);

    // TODO(jwninmmer-tri) Once TrajectoryCar1 is otherwise unused and can be
    // deleted, replace this forwarding code the TrajetoryCar1::output code
    // directly, and delete TrajectoryCar1.
    const NullVector<T> none;
    output_vector->get_mutable_value() =
        toEigen(wrapped_.output<T>(context.get_time(), none, none));
  }

 protected:
  std::unique_ptr<systems::BasicVector<T>> AllocateOutputVector(
      const systems::SystemPortDescriptor<T>& descriptor) const override {
    return std::make_unique<SimpleCarState<T>>();
  }

 private:
  const TrajectoryCar1 wrapped_;
};

}  // namespace cars
}  // namespace drake
