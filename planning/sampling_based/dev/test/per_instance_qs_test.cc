#include "drake/planning/sampling_based/dev/per_instance_qs.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/tree/revolute_joint.h"

namespace drake {
namespace planning {
namespace {

using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::multibody::RevoluteJoint;
using drake::multibody::RigidBody;

GTEST_TEST(PerInstanceQsTest, FullQThrow) {
  MultibodyPlant<double> plant{0.0};
  plant.Finalize();
  // Null output throws.
  DRAKE_EXPECT_THROWS_MESSAGE(SetInstanceQsInFullQ(plant, {}, {}),
                              ".*full_q != nullptr.*");
  // Wrong sized output throws.
  Eigen::VectorXd full_q = Eigen::VectorXd::Zero(10);
  DRAKE_EXPECT_THROWS_MESSAGE(SetInstanceQsInFullQ(plant, {}, &full_q),
                              ".*num_positions.* == full_q.*size.*");
  // Wrong model index throws.
  PerInstanceQs wrong_qs{{ModelInstanceIndex(22), Eigen::VectorXd::Zero(10)}};
  full_q = Eigen::VectorXd::Zero(plant.num_positions());
  DRAKE_EXPECT_THROWS_MESSAGE(SetInstanceQsInFullQ(plant, wrong_qs, &full_q),
                              ".*22.*bounds.*");
}

GTEST_TEST(PerInstanceQsTest, FullQEmpty) {
  MultibodyPlant<double> plant{0.0};
  plant.Finalize();
  Eigen::VectorXd full_q = Eigen::VectorXd::Zero(0);
  EXPECT_NO_THROW(SetInstanceQsInFullQ(plant, {}, &full_q));
}

/* Adds a new model consisting of `n` bodies connected by revolute joints. */
ModelInstanceIndex AddChain(MultibodyPlant<double>* plant, int n) {
  ModelInstanceIndex instance = plant->AddModelInstance(fmt::format("m{}", n));
  const RigidBody<double>* parent = nullptr;
  for (int k = 0; k < n; ++k) {
    const RigidBody<double>* body =
        &plant->AddRigidBody(fmt::format("b{}", k), instance);
    if (k > 0) {
      plant->AddJoint<RevoluteJoint>(fmt::format("j{}", k), *parent, {}, *body,
                                     {}, Eigen::Vector3d::UnitY());
    }
    parent = body;
  }
  return instance;
}

GTEST_TEST(PerInstanceQsTest, FullQChains) {
  MultibodyPlant<double> plant{0.0};
  std::vector<ModelInstanceIndex> models;
  for (int length : {3, 5, 7}) {
    models.push_back(AddChain(&plant, length));
  }
  plant.Finalize();
  Eigen::VectorXd full_q = Eigen::VectorXd::Zero(plant.num_positions());

  // Empty input does nothing.
  SetInstanceQsInFullQ(plant, {}, &full_q);
  EXPECT_EQ(full_q, Eigen::VectorXd::Zero(plant.num_positions()));

  // Fully populated input transfers all data.
  PerInstanceQs qs;
  for (auto instance : models) {
    int num_qs = plant.num_positions(instance);
    Eigen::VectorXd my_qs(num_qs);
    for (int k = 0; k < my_qs.size(); ++k) {
      // Fill my_qs with arbitrary, but recognizable, data.
      my_qs(k) = num_qs * 100 + k;
    }
    qs[instance] = my_qs;
  }
  SetInstanceQsInFullQ(plant, qs, &full_q);
  int offset = 0;
  for (auto instance : models) {
    int num_qs = plant.num_positions(instance);
    EXPECT_EQ(full_q.segment(offset, num_qs), qs[instance]);
    offset += num_qs;
  }

  // Partial input transfers the provided data.
  PerInstanceQs partial_qs{{models[1], qs[models[1]]}};
  full_q = Eigen::VectorXd::Zero(plant.num_positions());
  SetInstanceQsInFullQ(plant, partial_qs, &full_q);
  offset = 0;
  for (auto instance : models) {
    int num_qs = plant.num_positions(instance);
    if (instance == models[1]) {
      EXPECT_EQ(full_q.segment(offset, num_qs), qs[instance]);
    } else {
      EXPECT_EQ(full_q.segment(offset, num_qs), Eigen::VectorXd::Zero(num_qs));
    }
    offset += num_qs;
  }

  // Wrong-sized input vector for existing model throws.
  PerInstanceQs wrong_qs{{models[1], qs[models[0]]}};
  DRAKE_EXPECT_THROWS_MESSAGE(SetInstanceQsInFullQ(plant, wrong_qs, &full_q),
                              ".* not properly sized.*");
}

}  // namespace
}  // namespace planning
}  // namespace drake
