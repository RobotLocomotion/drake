#pragma once

// This file is generated by a script.  Do not edit!
// See drake/examples/Cars/lcm_vector_gen.py.

#include <stdexcept>
#include <string>

#include <Eigen/Core>

#include "lcmtypes/drake/lcmt_idm_with_trajectory_agent_state_t.hpp"
#include "drake/drakeCars_export.h"
#include "drake/systems/framework/basic_state_and_output_vector.h"

namespace drake {

/// Describes the row indices of a IdmWithTrajectoryAgentState.
struct DRAKECARS_EXPORT IdmWithTrajectoryAgentStateIndices {
  /// The total number of rows (coordinates).
  static const int kNumCoordinates = 5;

  // The index of each individual coordinate.
  static const int kXE = 0;
  static const int kVE = 1;
  static const int kXA = 2;
  static const int kVA = 3;
  static const int kAA = 4;
};

/// Specializes BasicStateAndOutputVector with specific getters and setters.
template <typename T>
class IdmWithTrajectoryAgentState
    : public systems::BasicStateAndOutputVector<T> {
 public:
  // An abbreviation for our row index constants.
  typedef IdmWithTrajectoryAgentStateIndices K;

  /// Default constructor.  Sets all rows to zero.
  IdmWithTrajectoryAgentState()
      : systems::BasicStateAndOutputVector<T>(K::kNumCoordinates) {
    this->SetFromVector(VectorX<T>::Zero(K::kNumCoordinates));
  }

  /// @name Getters and Setters
  //@{
  const T x_e() const { return this->GetAtIndex(K::kXE); }
  void set_x_e(const T& x_e) { this->SetAtIndex(K::kXE, x_e); }
  const T v_e() const { return this->GetAtIndex(K::kVE); }
  void set_v_e(const T& v_e) { this->SetAtIndex(K::kVE, v_e); }
  const T x_a() const { return this->GetAtIndex(K::kXA); }
  void set_x_a(const T& x_a) { this->SetAtIndex(K::kXA, x_a); }
  const T v_a() const { return this->GetAtIndex(K::kVA); }
  void set_v_a(const T& v_a) { this->SetAtIndex(K::kVA, v_a); }
  const T a_a() const { return this->GetAtIndex(K::kAA); }
  void set_a_a(const T& a_a) { this->SetAtIndex(K::kAA, a_a); }
  //@}

  /// @name Implement the LCMVector concept
  //@{
  typedef drake::lcmt_idm_with_trajectory_agent_state_t LCMMessageType;
  static std::string channel() { return "IDM_WITH_TRAJECTORY_AGENT_STATE"; }
  //@}
};

template <typename ScalarType>
bool encode(const double& t,
            const IdmWithTrajectoryAgentState<ScalarType>& wrap,
            // NOLINTNEXTLINE(runtime/references)
            drake::lcmt_idm_with_trajectory_agent_state_t& msg) {
  msg.timestamp = static_cast<int64_t>(t * 1000);
  msg.x_e = wrap.x_e();
  msg.v_e = wrap.v_e();
  msg.x_a = wrap.x_a();
  msg.v_a = wrap.v_a();
  msg.a_a = wrap.a_a();
  return true;
}

template <typename ScalarType>
bool decode(const drake::lcmt_idm_with_trajectory_agent_state_t& msg,
            // NOLINTNEXTLINE(runtime/references)
            double& t,
            // NOLINTNEXTLINE(runtime/references)
            IdmWithTrajectoryAgentState<ScalarType>& wrap) {
  t = static_cast<double>(msg.timestamp) / 1000.0;
  wrap.set_x_e(msg.x_e);
  wrap.set_v_e(msg.v_e);
  wrap.set_x_a(msg.x_a);
  wrap.set_v_a(msg.v_a);
  wrap.set_a_a(msg.a_a);
  return true;
}

}  // namespace drake
