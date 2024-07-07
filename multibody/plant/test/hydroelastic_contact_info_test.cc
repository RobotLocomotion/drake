#include "drake/multibody/plant/hydroelastic_contact_info.h"

namespace drake {
namespace multibody {

using geometry::ContactSurface;

// Returns a distinct spatial force.
SpatialForce<double> MakeSpatialForce() {
  return SpatialForce<double>(Vector3<double>(1, 2, 3),
                              Vector3<double>(4, 5, 6));
}

HydroelasticContactInfo<double> CreateContactInfo(
    std::unique_ptr<ContactSurface<double>>* contact_surface,
    std::unique_ptr<HydroelasticContactInfo<double>>* contact_info) {
  // Create the contact surface using a duplicated arbitrary ID and identity
  // pose; pose and geometry IDs are irrelevant for this test.
  GeometryId arbitrary_id = GeometryId::get_new_id();
  *contact_surface = CreateContactSurface(arbitrary_id, arbitrary_id,
                                          RigidTransform<double>::Identity());

  // Create the HydroelasticContactInfo using a particular spatial force.
  return HydroelasticContactInfo<double>(contact_surface->get(),
                                         MakeSpatialForce());
}

// Verifies that the HydroelasticContactInfo structure uses the raw pointer
// and the unique pointer, as appropriate, on copy construction.
GTEST_TEST(HydroelasticContactInfo, CopyConstruction) {
  std::unique_ptr<ContactSurface<double>> contact_surface;
  std::unique_ptr<HydroelasticContactInfo<double>> contact_info;
  HydroelasticContactInfo<double> copy =
      CreateContactInfo(&contact_surface, &contact_info);

  // Verify that copy construction used the raw pointer.
  EXPECT_EQ(contact_surface.get(), &copy.contact_surface());

  // Copy it again and make sure that the surface is new.
  HydroelasticContactInfo<double> copy2 = copy;
  EXPECT_NE(contact_surface.get(), &copy2.contact_surface());

  // Verify that the spatial force was copied.
  EXPECT_EQ(copy.F_Ac_W().translational(), MakeSpatialForce().translational());
  EXPECT_EQ(copy.F_Ac_W().rotational(), MakeSpatialForce().rotational());
}

// Verifies that the HydroelasticContactInfo structure transfers ownership of
// the ContactSurface.
GTEST_TEST(HydroelasticContactInfo, MoveConstruction) {
  std::unique_ptr<ContactSurface<double>> contact_surface;
  std::unique_ptr<HydroelasticContactInfo<double>> contact_info;
  HydroelasticContactInfo<double> copy =
      CreateContactInfo(&contact_surface, &contact_info);
  HydroelasticContactInfo<double> moved_copy = std::move(copy);

  // Verify that the move construction retained the raw pointer.
  EXPECT_EQ(contact_surface.get(), &moved_copy.contact_surface());

  // Verify that the spatial force was copied.
  EXPECT_EQ(moved_copy.F_Ac_W().translational(),
            MakeSpatialForce().translational());
  EXPECT_EQ(moved_copy.F_Ac_W().rotational(), MakeSpatialForce().rotational());
}

}  // namespace multibody
}  // namespace drake
