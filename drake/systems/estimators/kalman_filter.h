#pragma once

#include <memory>

#include <Eigen/Dense>

#include "drake/systems/estimators/luenberger_observer.h"
#include "drake/systems/framework/primitives/linear_system.h"

namespace drake {
namespace systems {
namespace estimators {

Eigen::MatrixXd SteadyStateKalmanFilter(
    const Eigen::Ref<const Eigen::MatrixXd>& A,
    const Eigen::Ref<const Eigen::MatrixXd>& C,
    const Eigen::Ref<const Eigen::MatrixXd>& W,
    const Eigen::Ref<const Eigen::MatrixXd>& V);

std::unique_ptr<LuenbergerObserver<double>> SteadyStateKalmanFilter(
    std::unique_ptr<LinearSystem<double>> system,
    const Eigen::Ref<const Eigen::MatrixXd>& W,
    const Eigen::Ref<const Eigen::MatrixXd>& V);

std::unique_ptr<LuenbergerObserver<double>> SteadyStateKalmanFilter(
    std::unique_ptr<System<double>> system,
    std::unique_ptr<Context<double>> context,
    const Eigen::Ref<const Eigen::MatrixXd>& W,
    const Eigen::Ref<const Eigen::MatrixXd>& V);

}  // namespace estimators
}  // namespace systems
}  // namespace drake
