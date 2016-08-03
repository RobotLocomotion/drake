#pragma once
#include "drake/core/Vector.h"

namespace drake {
	/* Implements a Drake System (@see drake/system/System.h) that saves the
	** simulated output. This is useful for testing and debugging the simulation
	** results
	*/
	template <template<typename> class Vector>
	class TrajectoryLogger {
	public:
  	TrajectoryLogger(int traj_dim) : traj_dim_(traj_dim),t_(0), y_(0) {};

      // Noncopyable
  	TrajectoryLogger(const TrajectoryLogger&) = delete;
  	TrajectoryLogger& operator=(const TrajectoryLogger&) = delete;

    template <typename ScalarType>
    using StateVector = drake::NullVector<ScalarType>;
    template <typename ScalarType>
    using InputVector = Vector<ScalarType>;
    template <typename ScalarType>
    using OutputVector = Vector<ScalarType>;

    template <typename ScalarType>
    StateVector<ScalarType> dynamics(const double& t,
    	                               const StateVector<ScalarType>& x,
    	                               const InputVector<ScalarType>& u) {
    	t_.push_back(t);
    	y_.push_back(u);
      return StateVector<ScalarType>();
    }

    template<typename ScalarType>
    OutputVector<ScalarType> output(const double& t,
                                    const StateVector<ScalarType>& x,
    	                              const InputVector<ScalarType>& u) const {
    	return u;
    }
    bool isTimeVarying() const { return false; }
    bool isDirectFeedthrough() const { return true; }
    size_t getNumStates() const { return static_cast<size_t>(0); }
    size_t getNumInputs() const { return traj_dim_; }
    size_t getNumOutputs() const { return traj_dim_; }


    std::vector<double> getTrajectoryTime() const {
    	return t_;
    }

    auto getTrajectorySamples() const {
    	return y_;
    }
	private:
    size_t traj_dim_;
		std::vector<double> t_;
    std::vector<InputVector<double>,
         Eigen::aligned_allocator<InputVector<double>>> y_;
	};
}