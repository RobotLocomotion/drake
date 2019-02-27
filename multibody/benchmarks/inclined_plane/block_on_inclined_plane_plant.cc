#include "drake/multibody/benchmarks/inclined_plane/block_on_inclined_plane_plant.h"

#include "drake/math/rigid_transform.h"
#include "drake/multibody/tree/uniform_gravity_field_element.h"

namespace drake {
namespace multibody {
namespace benchmarks {
namespace block_on_inclined_plane {

void AddBlockAndInclinedPlaneToPlant(
    double Lx, double Ly, double Lz, double mass, double slope, double gravity,
    const CoulombFriction<double>& coefficient_of_friction_block,
    const CoulombFriction<double>& coefficient_of_friction_inclined_plane,
    MultibodyPlant<double>* plant) {
  DRAKE_THROW_UNLESS(plant != nullptr);

  // Describe the block's mass, center of mass, and inertia properties.
  const UnitInertia<double> G_Bcm = UnitInertia<double>::SolidBox(Lx, Ly, Lz);
  const SpatialInertia<double> M_Bcm(mass, Vector3<double>::Zero(), G_Bcm);

  // Create a rigid body with the mass properties of a uniform solid block.
  const RigidBody<double>& block = plant->AddRigidBody("Block", M_Bcm);

  // Orient the inclined plane frame P with its unit vector Py = Wy (World y)
  // and unit vector Pz is outward normal to the plane and Px points downhill.
  const math::RotationMatrix<double> R_WP =
      math::RotationMatrix<double>::MakeYRotation(slope);

  // Express unit vector Pz (fixed in P) in terms of World Wx, Wy, Wz.
  const Vector3<double> Pz_W = R_WP.matrix().col(2);
  // Offset the inclined plane P so that when the block B's origin position in
  // World, expressed in World is p_WoBo_W = [0, 0, 0], the block's bottom
  // rectangular surface is in contact with the inclined plane's top surface.
  const Vector3<double> p_BoContact_W = -0.5 * Lz * Pz_W;

  // The inclined plane P's collision geometry is a half-space.
  const math::RigidTransform<double> X_WP(R_WP, p_BoContact_W);
  plant->RegisterCollisionGeometry(plant->world_body(),
                                   X_WP.GetAsIsometry3(),
                                   geometry::HalfSpace(),
                                   "collision",
                                   coefficient_of_friction_inclined_plane);

  // The inclined plane's visual geometry is a half-space.
  const Vector4<double> green(0.5, 1.0, 0.5, 1.0);
  plant->RegisterVisualGeometry(plant->world_body(),
                                X_WP.GetAsIsometry3(),
                                geometry::HalfSpace(),
                                "InclinedPlaneVisualGeometry",
                                green);

  // The block B's collision geometry is a solid box.  The pose X_BG of geometry
  // frame G in block's frame B is an identity transform.
  const math::RigidTransformd X_BG;   // Identity transform.
  plant->RegisterCollisionGeometry(block,
                                   X_BG.GetAsIsometry3(),
                                   geometry::Box(Lx, Ly, Lz),
                                   "collision",
                                   coefficient_of_friction_block);

  // The block's visual geometry is a solid box.
  const Vector4<double> orange(1.0, 0.55, 0.0, 1.0);
  plant->RegisterVisualGeometry(block,
                                X_BG.GetAsIsometry3(),
                                geometry::Box(Lx, Ly, Lz),
                                "visual",
                                orange);

  // Earth's local gravity acts in the World's -z direction.
  plant->AddForceElement<UniformGravityFieldElement>(
      -gravity * Vector3<double>::UnitZ());
}

}  // namespace block_on_inclined_plane
}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake
