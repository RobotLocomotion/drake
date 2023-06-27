#include "drake/multibody/plant/contact_results.h"

#include <gtest/gtest.h>

#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace multibody {

using Eigen::Vector3d;
using geometry::ContactSurface;
using geometry::GeometryId;
using geometry::PolygonSurfaceMesh;
using geometry::PolygonSurfaceMeshFieldLinear;
using geometry::PenetrationAsPointPair;

class ContactResultsTest : public ::testing::Test {
 public:
  void SetUp() override {
    // Empty surface mesh for hydroelastic contact surface
    auto surface_mesh = std::make_unique<PolygonSurfaceMesh<double>>();
    // Empty pressure field for hydroelastic contact surface
    auto field =
        std::make_unique<PolygonSurfaceMeshFieldLinear<double, double>>(
            std::vector<double>{}, surface_mesh.get(), false);
    contact_surface_ = std::make_unique<ContactSurface<double>>(
        GeometryId::get_new_id(), GeometryId::get_new_id(),
        std::move(surface_mesh), std::move(field));
    my_hydroelastic_contact_info_ =
        std::make_unique<HydroelasticContactInfo<double>>(
            contact_surface_.get(), SpatialForce<double>{},
            std::vector<HydroelasticQuadraturePointData<double>>{});
  }

 protected:
  const MultibodyPlant<double> plant_{/* time_step */ 0};
  const PointPairContactInfo<double> point_pair_info_{
      BodyIndex{0},
      BodyIndex{1},
      Vector3d::Zero(),
      Vector3d::Zero(),
      0,
      0,
      PenetrationAsPointPair<double>{}};

  std::unique_ptr<ContactSurface<double>> contact_surface_;
  std::unique_ptr<HydroelasticContactInfo<double>>
      my_hydroelastic_contact_info_;
};

TEST_F(ContactResultsTest, Default) {
  const ContactResults<double> default_contact_results;
  EXPECT_EQ(default_contact_results.num_point_pair_contacts(), 0);
  EXPECT_EQ(default_contact_results.num_hydroelastic_contacts(), 0);
  EXPECT_EQ(default_contact_results.plant(), nullptr);
}

// Tests the set, access, and clear operations. For the access and clear
// operations, the code has two branches that:
//   1. use alias pointers for hydroealstic_contact_info_
//   2. use unique pointers for hydroelastic_contact_info_ (happens after
//      assignment or copy constructor).
TEST_F(ContactResultsTest, SetAccessClear) {
  // Set
  // It will use alias pointer to my_hydroelastic_contact_info_.
  ContactResults<double> contact_results;
  contact_results.set_plant(&plant_);
  contact_results.AddContactInfo(point_pair_info_);
  contact_results.AddContactInfo(my_hydroelastic_contact_info_.get());

  // Access
  EXPECT_EQ(contact_results.plant(), &plant_);
  EXPECT_EQ(contact_results.num_point_pair_contacts(), 1);
  EXPECT_EQ(contact_results.num_hydroelastic_contacts(), 1);
  EXPECT_EQ(contact_results.point_pair_contact_info(0).bodyA_index(),
            point_pair_info_.bodyA_index());
  EXPECT_EQ(&contact_results.hydroelastic_contact_info(0),
            my_hydroelastic_contact_info_.get());
  // `copy` will use unique pointers for hydroelastic_contact_info_.
  ContactResults<double> copy(contact_results);
  EXPECT_EQ(copy.num_hydroelastic_contacts(), 1);
  EXPECT_NE(&copy.hydroelastic_contact_info(0),
            my_hydroelastic_contact_info_.get());
  // Clear
  // Test the alias-pointer variant.
  contact_results.Clear();
  EXPECT_EQ(contact_results.plant(), nullptr);
  EXPECT_EQ(contact_results.num_point_pair_contacts(), 0);
  EXPECT_EQ(contact_results.num_hydroelastic_contacts(), 0);
  // Test the unique-pointer variant.
  copy.Clear();
  EXPECT_EQ(copy.num_hydroelastic_contacts(), 0);
}

// This is just a coverage test of the assignment operator. It doesn't try to
// verify the results of assignment operator rigorously. It only checks that
// the three member variables are valid without looking at their values.
//
// The operator manages two variants of hydroelastic_contact_info_, which
// can be a vector of aliased pointers or a vector of unique pointers.
// We check the pointer values as returned by the
// hydroelastic_contact_info(int) function to distinguish the two cases.
TEST_F(ContactResultsTest, AssignHydroelasticContactInfo) {
  // Assign empty RHS to LHS.
  {
    ContactResults<double> lhs;
    const ContactResults<double> rhs_empty;
    lhs = rhs_empty;
    EXPECT_EQ(lhs.num_hydroelastic_contacts(), 0);
    EXPECT_EQ(lhs.num_point_pair_contacts(), 0);
    EXPECT_EQ(lhs.plant(), nullptr);
  }
  // Assign non-empty RHS to LHS.
  // LHS will create unique pointers of copies of hydroelastic contacts.
  {
    ContactResults<double> lhs;
    ContactResults<double> rhs_non_empty;
    rhs_non_empty.set_plant(&plant_);
    rhs_non_empty.AddContactInfo(point_pair_info_);
    rhs_non_empty.AddContactInfo(my_hydroelastic_contact_info_.get());

    lhs = rhs_non_empty;
    EXPECT_EQ(lhs.num_hydroelastic_contacts(), 1);
    EXPECT_EQ(lhs.num_point_pair_contacts(), 1);
    EXPECT_EQ(lhs.plant(), &plant_);
    // Confirm that the assignment operator copies the content to
    // a different memory address.
    EXPECT_NE(&lhs.hydroelastic_contact_info(0),
              &rhs_non_empty.hydroelastic_contact_info(0));
  }
}

TEST_F(ContactResultsTest, SelectHydroelastic) {
  ContactResults<double> contact_results;
  contact_results.set_plant(&plant_);
  contact_results.AddContactInfo(point_pair_info_);
  contact_results.AddContactInfo(my_hydroelastic_contact_info_.get());

  const ContactResults<double> no_hydro_contacts =
      contact_results.SelectHydroelastic(
          [](const HydroelasticContactInfo<double>&) -> bool {
            return false;
          });
  EXPECT_EQ(no_hydro_contacts.num_point_pair_contacts(), 1);
  EXPECT_EQ(no_hydro_contacts.num_hydroelastic_contacts(), 0);

  const ContactResults<double> one_hydro_contact =
      contact_results.SelectHydroelastic(
          [](const HydroelasticContactInfo<double>&) -> bool {
            return true;
          });
  EXPECT_EQ(one_hydro_contact.num_point_pair_contacts(), 1);
  EXPECT_EQ(one_hydro_contact.num_hydroelastic_contacts(), 1);
  // Sanity check of hydroelastic contact info
  EXPECT_EQ(
      one_hydro_contact.hydroelastic_contact_info(0).contact_surface().id_M(),
      my_hydroelastic_contact_info_->contact_surface().id_M());
  // Verify the deep copy by checking for different memory address.
  EXPECT_NE(
      &one_hydro_contact.hydroelastic_contact_info(0),
      my_hydroelastic_contact_info_.get());
}

}  // namespace multibody
}  // namespace drake
