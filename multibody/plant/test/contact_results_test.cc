#include "drake/multibody/plant/contact_results.h"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/limit_malloc.h"
#include "drake/multibody/plant/multibody_plant.h"

namespace drake {
namespace multibody {

using Eigen::Vector3d;
using geometry::ContactSurface;
using geometry::GeometryId;
using geometry::PenetrationAsPointPair;
using geometry::PolygonSurfaceMesh;
using geometry::PolygonSurfaceMeshFieldLinear;

class ContactResultsTest : public ::testing::Test {
 public:
  void SetUp() override {
    // Empty surface mesh for hydroelastic contact surface
    auto surface_mesh = std::make_unique<PolygonSurfaceMesh<double>>();
    // Empty pressure field for hydroelastic contact surface
    auto field =
        std::make_unique<PolygonSurfaceMeshFieldLinear<double, double>>(
            std::vector<double>{}, surface_mesh.get(),
            geometry::MeshGradientMode::kNone);
    contact_surface_ = std::make_unique<ContactSurface<double>>(
        GeometryId::get_new_id(), GeometryId::get_new_id(),
        std::move(surface_mesh), std::move(field));
    my_hydroelastic_contact_info_ =
        std::make_unique<HydroelasticContactInfo<double>>(
            contact_surface_.get(), F_);
    id_A_ = GeometryId::get_new_id();
    id_B_ = GeometryId::get_new_id();
    deformable_contact_info_ = std::make_unique<DeformableContactInfo<double>>(
        id_A_, id_B_, PolygonSurfaceMesh<double>(), F_);
  }

  std::vector<PointPairContactInfo<double>> MakePointPair() const {
    return {point_pair_info_};
  }

  std::vector<HydroelasticContactInfo<double>> MakeHydroelastic() const {
    return {*my_hydroelastic_contact_info_};
  }

  std::vector<DeformableContactInfo<double>> MakeDeformable() const {
    return {*deformable_contact_info_};
  }

 protected:
  const MultibodyPlant<double> plant_{/* time_step */ 0.01};
  const PointPairContactInfo<double> point_pair_info_{
      BodyIndex{0},
      BodyIndex{1},
      Vector3d::Zero(),
      Vector3d::Zero(),
      0,
      0,
      PenetrationAsPointPair<double>{}};
  GeometryId id_A_;
  GeometryId id_B_;
  SpatialForce<double> F_{Vector3d(1, 2, 3), Vector3d(4, 5, 6)};
  std::unique_ptr<DeformableContactInfo<double>> deformable_contact_info_;
  std::unique_ptr<ContactSurface<double>> contact_surface_;
  std::unique_ptr<HydroelasticContactInfo<double>>
      my_hydroelastic_contact_info_;
};

TEST_F(ContactResultsTest, Default) {
  const ContactResults<double> default_contact_results;
  EXPECT_EQ(default_contact_results.num_point_pair_contacts(), 0);
  EXPECT_EQ(default_contact_results.num_hydroelastic_contacts(), 0);
  EXPECT_EQ(default_contact_results.num_deformable_contacts(), 0);
  EXPECT_EQ(default_contact_results.plant(), nullptr);
}

TEST_F(ContactResultsTest, VectorConstructor) {
  // Call the vector-of-infos constructor.
  const ContactResults<double> dut{MakePointPair(), MakeHydroelastic(),
                                   MakeDeformable()};

  // Spot check the basic counts.
  EXPECT_EQ(dut.num_point_pair_contacts(), 1);
  EXPECT_EQ(dut.num_hydroelastic_contacts(), 1);
  EXPECT_EQ(dut.num_deformable_contacts(), 1);
  EXPECT_EQ(dut.plant(), nullptr);

  // Spot check one field on each sub-info.
  EXPECT_EQ(dut.point_pair_contact_info(0).bodyB_index(),
            point_pair_info_.bodyB_index());
  EXPECT_EQ(dut.hydroelastic_contact_info(0).F_Ac_W().get_coeffs(),
            F_.get_coeffs());
  EXPECT_EQ(dut.deformable_contact_info(0).id_B(), id_B_);
}

TEST_F(ContactResultsTest, BackingStore) {
  // We'll use this storage to interrogate exactly what contact results does
  // with its `backing_store` constructor argument.
  auto storage = std::make_shared<std::string>("Hello");
  auto weak = std::weak_ptr<const void>(storage);

  // We'll construct the dut into a unique_ptr so that we can call the
  // destructor to cover any effects it has that differ from assignment.
  auto dut = std::make_unique<ContactResults<double>>(
      MakePointPair(), MakeHydroelastic(), MakeDeformable(),
      /* backing_store = */ std::move(storage));

  // Nothing hereafter should use new heap allocations.
  drake::test::LimitMalloc guard({.max_num_allocations = 0});

  // The contact results should be keeping the storage alive.
  EXPECT_EQ(storage.get(), nullptr);
  EXPECT_FALSE(weak.expired());

  // The copy will likewise keep the storage alive.
  ContactResults<double> copy{*dut};
  EXPECT_FALSE(weak.expired());

  // Clear the original object.
  dut.reset();
  EXPECT_FALSE(weak.expired());

  // Clear the copy. Now the storage is finally deleted.
  copy = ContactResults<double>{};
  EXPECT_TRUE(weak.expired());
}

TEST_F(ContactResultsTest, Accessors) {
  ContactResults<double> contact_results{MakePointPair(), MakeHydroelastic(),
                                         MakeDeformable()};
  contact_results.set_plant(&plant_);

  // Check all accessors.
  EXPECT_EQ(contact_results.plant(), &plant_);
  EXPECT_EQ(contact_results.num_point_pair_contacts(), 1);
  EXPECT_EQ(contact_results.num_hydroelastic_contacts(), 1);
  EXPECT_EQ(contact_results.num_deformable_contacts(), 1);
  EXPECT_EQ(contact_results.point_pair_contact_info(0).bodyA_index(),
            point_pair_info_.bodyA_index());
  EXPECT_EQ(contact_results.hydroelastic_contact_info(0).F_Ac_W().get_coeffs(),
            my_hydroelastic_contact_info_->F_Ac_W().get_coeffs());
  EXPECT_EQ(contact_results.deformable_contact_info(0).id_A(), id_A_);
  EXPECT_EQ(contact_results.deformable_contact_info(0).id_B(), id_B_);
  EXPECT_EQ(contact_results.deformable_contact_info(0).F_Ac_W().translational(),
            deformable_contact_info_->F_Ac_W().translational());
  EXPECT_EQ(contact_results.deformable_contact_info(0).F_Ac_W().rotational(),
            deformable_contact_info_->F_Ac_W().rotational());

  // Quick sanity check after a copy, too.
  ContactResults<double> copy(contact_results);
  EXPECT_EQ(copy.num_point_pair_contacts(), 1);
  EXPECT_EQ(copy.num_hydroelastic_contacts(), 1);
  EXPECT_EQ(copy.num_deformable_contacts(), 1);
}

// This is just a coverage test of the assignment operator. It doesn't try to
// verify the results of assignment operator rigorously. It only checks that
// the three member variables are valid without looking at their values.
TEST_F(ContactResultsTest, Assignment) {
  // Assign empty RHS to LHS.
  {
    ContactResults<double> lhs;
    const ContactResults<double> rhs;
    lhs = rhs;
    EXPECT_EQ(lhs.num_hydroelastic_contacts(), 0);
    EXPECT_EQ(lhs.num_point_pair_contacts(), 0);
    EXPECT_EQ(lhs.num_deformable_contacts(), 0);
    EXPECT_EQ(lhs.plant(), nullptr);
  }

  // Assign non-empty RHS to empty LHS.
  {
    ContactResults<double> lhs;
    ContactResults<double> rhs{MakePointPair(), MakeHydroelastic(),
                               MakeDeformable()};
    rhs.set_plant(&plant_);

    lhs = rhs;
    EXPECT_EQ(lhs.num_hydroelastic_contacts(), 1);
    EXPECT_EQ(lhs.num_point_pair_contacts(), 1);
    EXPECT_EQ(lhs.num_deformable_contacts(), 1);
    EXPECT_EQ(lhs.plant(), &plant_);
  }

  // Assign empty RHS to non-empty LHS.
  {
    ContactResults<double> lhs{MakePointPair(), MakeHydroelastic(),
                               MakeDeformable()};
    lhs.set_plant(&plant_);
    ContactResults<double> rhs;

    lhs = rhs;
    EXPECT_EQ(lhs.num_hydroelastic_contacts(), 0);
    EXPECT_EQ(lhs.num_point_pair_contacts(), 0);
    EXPECT_EQ(lhs.num_deformable_contacts(), 0);
    EXPECT_EQ(lhs.plant(), nullptr);
  }
}

TEST_F(ContactResultsTest, HydroelasticNoHeapOperations) {
  // Set up contact results using a bare pointer to the contact surface. After
  // vector memory is reserved, this should only allocate once more to create
  // the nested Data structure. The contact surface is NOT copied.
  std::vector<HydroelasticContactInfo<double>> hydroelastic;
  hydroelastic.reserve(1);
  drake::test::LimitMalloc guard({.max_num_allocations = 1});
  hydroelastic.emplace_back(contact_surface_.get(), F_);
  const ContactResults<double> dut(std::vector<PointPairContactInfo<double>>{},
                                   std::move(hydroelastic));
  const ContactResults<double> copy1(dut);
  const ContactResults<double> copy2(copy1);
}

TEST_F(ContactResultsTest, SelectHydroelastic) {
  const ContactResults<double> contact_results{
      MakePointPair(), MakeHydroelastic(), MakeDeformable()};

  const ContactResults<double> no_hydro_contacts =
      contact_results.SelectHydroelastic(
          [](const HydroelasticContactInfo<double>&) -> bool {
            return false;
          });
  EXPECT_EQ(no_hydro_contacts.num_point_pair_contacts(), 1);
  EXPECT_EQ(no_hydro_contacts.num_hydroelastic_contacts(), 0);
  EXPECT_EQ(no_hydro_contacts.num_deformable_contacts(), 1);

  const ContactResults<double> one_hydro_contact =
      contact_results.SelectHydroelastic(
          [](const HydroelasticContactInfo<double>&) -> bool {
            return true;
          });
  EXPECT_EQ(one_hydro_contact.num_point_pair_contacts(), 1);
  EXPECT_EQ(one_hydro_contact.num_hydroelastic_contacts(), 1);
  EXPECT_EQ(no_hydro_contacts.num_deformable_contacts(), 1);
  // Sanity check of hydroelastic contact info
  EXPECT_EQ(
      one_hydro_contact.hydroelastic_contact_info(0).contact_surface().id_M(),
      my_hydroelastic_contact_info_->contact_surface().id_M());
  // Verify the deep copy by checking for different memory address.
  EXPECT_NE(&one_hydro_contact.hydroelastic_contact_info(0),
            my_hydroelastic_contact_info_.get());
}

}  // namespace multibody
}  // namespace drake
