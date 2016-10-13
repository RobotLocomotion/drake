#include <iostream>
#include <memory>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include <drake/systems/plants/RigidBody.h>
#include <drake/systems/plants/RigidBodyTree.h>
#include <drake/systems/plants/joints/QuaternionFloatingJoint.h>
#include <drake/systems/plants/rigid_body_plant/rigid_body_plant.h>

using Eigen::Isometry3d;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using std::make_unique;
using std::move;
using std::unique_ptr;

namespace drake {
namespace systems {
namespace plants {
namespace rigid_body_plant {
namespace test {

template <class T>
std::unique_ptr<FreestandingInputPort> MakeInput(
    std::unique_ptr<BasicVector<T>> data) {
  return make_unique<FreestandingInputPort>(std::move(data));
}

class ContactResultTest : public ::testing::Test {
 protected:
  void SetUp() override {
  }

  // The plant owns this tree; this is a convenience pointer.
  RigidBodyTree* tree_{};
  unique_ptr<RigidBodyPlant<double>> plant_{};
  unique_ptr<Context<double>> context_{};
  unique_ptr<SystemOutput<double>> output_{};
  const double kRadius = 1.0;


  const ContactResults<double>& RunTest(double distance) {
    // Places two spheres are on the x-y plane mirrored across the origin from
    //  each other such there is `distance` units gap between them.  Negative
    //  numbers imply collision.

    tree_ = new RigidBodyTree();

    Vector3d pos;
    pos << -(kRadius + distance), 0, 0;
    AddSphere(pos, "sphere1");
    pos << (kRadius + distance), 0, 0;
    AddSphere(pos, "sphere2");

    tree_->compile();

    // Populate the plant
    plant_ = make_unique<RigidBodyPlant<double>>(
        move(unique_ptr<RigidBodyTree>(tree_))
    );
    context_ = plant_->CreateDefaultContext();
    output_ = plant_->AllocateOutput(*context_);
    context_->SetInputPort(0, MakeInput(
        make_unique<BasicVector<double>>(0)));
    plant_->SetZeroConfiguration(context_.get());
    plant_->EvalOutput(*context_.get(), output_.get());

    // TODO(SeanCurtis-TRI): This hard-coded value is unfortuante. However, there
    //  is no mechanism for finding out the port id for a known port (e.g.,
    //  contact results).
    return output_->get_data(2)->GetValue<ContactResults<double>>();
  }

  // Add a sphere with default radius, placed at the given position.
  void AddSphere(const Vector3d& pos, const std::string& name) {
    RigidBody *body;
    tree_->add_rigid_body(unique_ptr<RigidBody>(body = new RigidBody()));
    body->set_name(name);
    body->set_mass(1.0);
    body->set_spatial_inertia(Matrix6<double>::Identity());
    Isometry3d pose = Isometry3d::Identity();
    pose.translate(pos);
    body->add_joint(
        &tree_->world(),
        make_unique<QuaternionFloatingJoint>("base", pose));
    DrakeShapes::Sphere sphere(kRadius);
    DrakeCollision::Element cElement(sphere);
    cElement.set_body(body);
    tree_->addCollisionElement(cElement, *body, "group1");
  }
};

// Confirms a contact result for two colliding spheres.
TEST_F(ContactResultTest, NoCollision) {
  auto& contact_results = RunTest(0.1);
  ASSERT_EQ(contact_results.get_num_contacts(), 0);
}

// Confirms a contact result for two colliding spheres.
TEST_F(ContactResultTest, Touching) {
  // For now, osculation is not considered a "contact" for reporting purposes.
  //  If the definition changes, this will likewise change.
  auto& contact_results = RunTest(0.0);
  ASSERT_EQ(contact_results.get_num_contacts(), 0);
}

// Confirms a contact result for two colliding spheres.
TEST_F(ContactResultTest, SingleCollision) {
  auto& contact_results = RunTest(-0.1);
  ASSERT_EQ(contact_results.get_num_contacts(), 1);
}
} // test
} // rigid_body_plant
} // plants
} // systems
} // drake