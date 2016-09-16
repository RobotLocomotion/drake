#include <iostream>
#include <memory>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

// #include "drake/common/drake_path.h"
#include "drake/common/eigen_types.h"
// #include "drake/math/roll_pitch_yaw.h"
#include "drake/systems/lcm/translator_between_lcmt_drake_signal.h"
#include "drake/systems/plants/RigidBodySystem.h"
#include "drake/systems/plants/joints/QuaternionFloatingJoint.h"
// #include "drake/systems/plants/parser_model_instance_id_table.h"
// #include "drake/systems/plants/parser_urdf.h"
#include "drake/systems/plants/rigid_body_plant/lcmt_viewer_draw_translator_1.h"
#include "drake/systems/plants/rigid_body_plant/rigid_body_plant.h"

namespace drake {
namespace systems {
namespace {

using std::make_unique;
// using std::move;
// using std::unique_ptr;

using Eigen::Isometry3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::VectorXd;

// Tests the basic functionality of the translator.
GTEST_TEST(LcmtViewerDrawTranslator1Tests, BasicTest) {
  // Creates a RigidBodyTree with two rigid bodies.
  // auto tree = make_unique<RigidBodyTree>();


  // LcmtViewerDrawTranslator1 translator(*tree.get());

  // const double kTime = 2.23606797749978;
  // VectorX<double> eigen_vector;

  // eigen_vector.resize(10);
  // eigen_vector << 0.0, 1.1, 2.2, 3.3, 4.4, 5.5, 6.6, 7.7, 8.8, 9.9;

  // BasicVector<double> basic_vector(10);
  // basic_vector.set_value(eigen_vector);

  // std::vector<uint8_t> lcm_message_bytes;

  // translator.TranslateLcmToVectorBase();
  // translator.TranslateVectorBaseToLcm(kTime, basic_vector, lcm_message_bytes);

  // VectorBase<double>* vector_base = nullptr;
  // void LcmtViewerDrawTranslatorV1::TranslateLcmToVectorBase(
  //   const void* lcm_message_bytes, int lcm_message_length,
  //   VectorBase<double>* vector_base)

}

}  // namespace
}  // namespace systems
}  // namespace drake
