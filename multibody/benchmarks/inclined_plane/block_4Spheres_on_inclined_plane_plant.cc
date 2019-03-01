#include "drake/multibody/benchmarks/inclined_plane/block_4Spheres_on_inclined_plane_plant.h"

#include "drake/math/rigid_transform.h"
#include "drake/multibody/tree/uniform_gravity_field_element.h"

namespace drake {
namespace multibody {
namespace benchmarks {
namespace block_4Spheres_on_inclined_plane {

void AddBlockWith4SpheresAndInclinedPlaneToPlant(
    double Lx, double Ly, double Lz, double mass, double slope, double gravity,
    const CoulombFriction<double>& coefficient_of_friction_block,
    const CoulombFriction<double>& coefficient_of_friction_inclined_plane,
    MultibodyPlant<double>* plant) {
  DRAKE_THROW_UNLESS(plant != nullptr);

  // Describe the block's mass, center of mass, and inertia properties.
  const Vector3<double> p_BoBcm_B = Vector3<double>::Zero();
  const UnitInertia<double> G_Bcm = UnitInertia<double>::SolidBox(Lx, Ly, Lz);
  const SpatialInertia<double> M_Bcm(mass, p_BoBcm_B, G_Bcm);

  // Create a rigid body with the mass properties of a uniform solid block.
  const RigidBody<double>& block = plant->AddRigidBody("Block4Spheres", M_Bcm);

  // The inclined-plane is oriented by initially setting Ax=Wx, Ay=Wy, Az=Wz, and
  // then subjecting A to a right-handed rotation in W about Ay=Wy, so that Ax
  // is directed downhill.  Wz points locally upward (opposite gravity).
  const math::RotationMatrix<double> R_WA =
      math::RotationMatrix<double>::MakeYRotation(slope);

  // Express unit vector Az (fixed in A) in terms of World Wx, Wy, Wz.
  const Vector3<double> Az_W = R_WA.matrix().col(2);
  // Offset the inclined-plane A so that when the block B's origin position in
  // World, expressed in World is p_WoBo_W = [0, 0, 0], the block's bottom
  // rectangular surface is in contact with the inclined plane's top surface.
  const Vector3<double> p_BoContact_W = -0.5 * Lz * Az_W;

  // The inclined-plane A's collision geometry is a half-space.
  const math::RigidTransform<double> X_WA(R_WA, p_BoContact_W);
  plant->RegisterCollisionGeometry(plant->world_body(),
                                   X_WA.GetAsIsometry3(),
                                   geometry::HalfSpace(),
                                   "InclinedPlaneCollisionGeometry",
                                   coefficient_of_friction_inclined_plane);

  // The inclined plane A's visual geometry is a half-space.
  const Vector4<double> green(0.5, 1.0, 0.5, 1.0);
  plant->RegisterVisualGeometry(plant->world_body(), X_WA.GetAsIsometry3(),
                                geometry::HalfSpace(),
                                "InclinedPlaneVisualGeometry",
                                green);

  // The block B's visual geometry is a solid box.
  // The pose X_BG of geometry frame G in block B is an identity transform.
  const math::RigidTransformd X_BG;   // Identity transform.
  const Vector4<double> lightBlue(0.5, 0.8, 1.0, 1.0);
  plant->RegisterVisualGeometry(block, X_BG.GetAsIsometry3(),
                                geometry::Box(Lx, Ly, Lz),
                                "BlockVisualGeometry",
                                lightBlue);

  // There are four identical spheres welded to the block B's bottom surface.
  // The rotation matrix for each of these spheres is the 3x3 identity matrix.
  const Vector3<double> p_BoSphere1_B(+Lx/2, +Ly/2, -Lz/2);
  const Vector3<double> p_BoSphere2_B(+Lx/2, -Ly/2, -Lz/2);
  const Vector3<double> p_BoSphere3_B(-Lx/2, +Ly/2, -Lz/2);
  const Vector3<double> p_BoSphere4_B(-Lx/2, -Ly/2, -Lz/2);
  const math::RigidTransform<double> X_BSphere1(p_BoSphere1_B);
  const math::RigidTransform<double> X_BSphere2(p_BoSphere2_B);
  const math::RigidTransform<double> X_BSphere3(p_BoSphere3_B);
  const math::RigidTransform<double> X_BSphere4(p_BoSphere4_B);

  // Register collision geometry for all 4 spheres.
  const double radius = Lz/2;
  plant->RegisterCollisionGeometry(block, X_BSphere1.GetAsIsometry3(),
                                   geometry::Sphere(radius),
                                   "Sphere1_CollisionGeometry",
                                   coefficient_of_friction_block);
  plant->RegisterCollisionGeometry(block, X_BSphere2.GetAsIsometry3(),
                                   geometry::Sphere(radius),
                                   "Sphere2_CollisionGeometry",
                                   coefficient_of_friction_block);
  plant->RegisterCollisionGeometry(block, X_BSphere3.GetAsIsometry3(),
                                   geometry::Sphere(radius),
                                   "Sphere3_CollisionGeometry",
                                   coefficient_of_friction_block);
  plant->RegisterCollisionGeometry(block, X_BSphere4.GetAsIsometry3(),
                                   geometry::Sphere(radius),
                                   "Sphere4_CollisionGeometry",
                                   coefficient_of_friction_block);

  // Register visual geometry for all 4 spheres.
  const Vector4<double> red(1.0, 0.0, 0.0, 1.0);
  plant->RegisterVisualGeometry(block, X_BSphere1.GetAsIsometry3(),
                                geometry::Sphere(radius),
                                "Sphere1_VisualGeometry", red);
  plant->RegisterVisualGeometry(block, X_BSphere2.GetAsIsometry3(),
                                geometry::Sphere(radius),
                                "Sphere2_VisualGeometry", red);
  plant->RegisterVisualGeometry(block, X_BSphere3.GetAsIsometry3(),
                                geometry::Sphere(radius),
                                "Sphere3_VisualGeometry", red);
  plant->RegisterVisualGeometry(block, X_BSphere4.GetAsIsometry3(),
                                geometry::Sphere(radius),
                                "Sphere4_VisualGeometry", red);

  // Earth's local gravity acts in the World's -z direction.
  plant->AddForceElement<UniformGravityFieldElement>(
      -gravity * Vector3<double>::UnitZ());
}

}  // namespace block_4Spheres_on_inclined_plane
}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake
