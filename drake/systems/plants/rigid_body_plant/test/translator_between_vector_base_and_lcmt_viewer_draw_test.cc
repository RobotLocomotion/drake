#include <iostream>
#include <memory>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
#include "drake/common/eigen_types.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/systems/lcm/translator_between_lcmt_drake_signal.h"
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/systems/plants/joints/QuaternionFloatingJoint.h"
#include "drake/systems/plants/parser_model_instance_id_table.h"
#include "drake/systems/plants/parser_urdf.h"
#include "drake/systems/plants/rigid_body_plant/rigid_body_plant.h"
// NOLINT(readability/line_length)
#include "drake/systems/plants/rigid_body_plant/translator_between_vector_base_and_lcmt_viewer_draw.h"


namespace drake {
namespace systems {
namespace {

using std::make_unique;
using std::move;
using std::unique_ptr;

using Eigen::Isometry3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::VectorXd;

// Tests the basic functionality of the translator.
GTEST_TEST(TranslatorBetweenVectorBaseAndLcmtViewerDrawTest, BasicTest) {
  auto tree = make_unique<RigidBodyTree>();
  TranslatorBetweenVectorBaseAndLcmtViewerDraw translator(*tree.get());
}

}  // namespace
}  // namespace systems
}  // namespace drake
