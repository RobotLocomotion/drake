# pragma once

#include <iostream>
#include <memory>

#include <lcm/lcm-cpp.hpp>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_path.h"
#include "drake/common/polynomial.h"
#include "drake/systems/plants/RigidBodyTree.h"
#include "drake/systems/trajectories/PiecewisePolynomial.h"
#include "drake/systems/vector.h"

#include "lcmtypes/drake/lcmt_iiwa_command.hpp"
#include "lcmtypes/drake/lcmt_iiwa_status.hpp"
#include "lcmtypes/drake/robot_plan_t.hpp"


#include <Eigen/Geometry>
#include <lcm/lcm-cpp.hpp>

#include "drake/drakeKukaIiwaArm_export.h"

#include "iiwa_status.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::VectorXi;
using drake::Vector1d;
using Eigen::Vector2d;
using Eigen::Vector3d;

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

//const char* kLcmCommandChannel = "IIWA_COMMAND";
const char* kLcmPlanChannel = "COMMITTED_ROBOT_PLAN";

typedef PiecewisePolynomial<double> PPType;
typedef PPType::PolynomialType PPPoly;
typedef PPType::PolynomialMatrix PPMatrix;

class DRAKEKUKAIIWAARM_EXPORT RobotPlanRunner {
 public:
  /// tree is aliased
  RobotPlanRunner(std::shared_ptr<lcm::LCM> lcm, const RigidBodyTree& tree);

  void Run();

 private:
  void HandleStatus(const lcm::ReceiveBuffer* rbuf, const std::string& chan,
                    const lcmt_iiwa_status* status);

  void HandlePlan(const lcm::ReceiveBuffer* rbuf, const std::string& chan,
                  const robot_plan_t* plan);
 private :

  static const int kNumJoints = 7;
  std::shared_ptr<lcm::LCM> lcm_;
  const RigidBodyTree& tree_;
  int plan_number_;
  std::unique_ptr<PPType> plan_;
  lcmt_iiwa_status iiwa_status_;
};

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
