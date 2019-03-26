#include "drake/multibody/benchmarks/inclined_plane/inclined_plane_plant.h"

#include "drake/math/rigid_transform.h"
#include "drake/multibody/tree/uniform_gravity_field_element.h"

namespace drake {
namespace multibody {
namespace benchmarks {
namespace inclined_plane {

void AddInclinedPlaneAndEarthGravityToPlant(
    double gravity, double slope_radians,
    bool is_inclined_plane_half_space, double LAx, double LAy, double LAz,
    const CoulombFriction<double>& coefficient_friction_inclined_plane,
    MultibodyPlant<double>* plant) {
  DRAKE_THROW_UNLESS(plant != nullptr);

  // The inclined-plane A is oriented by initially setting Ax=Wx, Ay=Wy, Az=Wz,
  // and then subjecting A to a right-handed rotation in W about Ay=Wy, so that
  // Ax is directed downhill.  Wz points locally upward (opposite gravity).
  const math::RotationMatrix<double> R_WA =
      math::RotationMatrix<double>::MakeYRotation(slope_radians);

  // The inclined-plane is either a half-space or a box.
  const Vector4<double> green(0.5, 1.0, 0.5, 1.0);
  if (is_inclined_plane_half_space) {
    // Set inclined plane A's visual geometry to a half-space.
    // Make half-space A's top-surface pass through World origin Wo.
    const Vector3<double> p_WoAo_W = Vector3<double>::Zero();
    const math::RigidTransform<double> X_WA(R_WA, p_WoAo_W);
    plant->RegisterVisualGeometry(plant->world_body(), X_WA,
                                  geometry::HalfSpace(),
                                  "InclinedPlaneVisualGeometry",
                                  green);

    // Set inclined-plane A's collision geometry to a half-space.
    plant->RegisterCollisionGeometry(plant->world_body(), X_WA,
                                     geometry::HalfSpace(),
                                     "InclinedPlaneCollisionGeometry",
                                     coefficient_friction_inclined_plane);
  } else {
    // If using a box, ensure its dimensions are positive.
    DRAKE_THROW_UNLESS(LAx > 0 && LAy > 0 && LAz > 0);

    // Set inclined plane A's visual geometry to a box.
    // Make box A's top-surface pass through World origin Wo.  To do this,
    // set Ao's position from Wo as -0.5 * LAz * Az (half-width of box A).
    // First step is to express unit vector Az in terms of World Wx, Wy, Wz.
    const Vector3<double> Az_W = R_WA.col(2);
    const Vector3<double> p_WoAo_W = -0.5 * LAz * Az_W;
    const math::RigidTransform<double> X_WA(R_WA, p_WoAo_W);
    plant->RegisterVisualGeometry(plant->world_body(), X_WA,
                                  geometry::Box(LAx, LAy, LAz),
                                  "InclinedPlaneVisualGeometry",
                                  green);

    // Set inclined-plane A's collision geometry to a box.
    plant->RegisterCollisionGeometry(plant->world_body(), X_WA,
                                     geometry::Box(LAx, LAy, LAz),
                                     "InclinedPlaneCollisionGeometry",
                                     coefficient_friction_inclined_plane);
  }

  // Earth's local gravity acts in the World's -z direction.
  const Vector3<double> gravity_vector(0, 0, -gravity);
  plant->AddForceElement<UniformGravityFieldElement>(gravity_vector);
}

void AddInclinedPlaneWithBlockPlant(
    double gravity, double slope_radians,
    bool is_inclined_plane_half_space, double LAx, double LAy, double LAz,
    double LBx, double LBy, double LBz, double massB,
    const CoulombFriction<double>& coefficient_friction_inclined_plane,
    const CoulombFriction<double>& coefficient_friction_bodyB,
    bool is_block_with_4Spheres,
    MultibodyPlant<double>* plant) {
  DRAKE_THROW_UNLESS(plant != nullptr);
  DRAKE_THROW_UNLESS(LBx > 0 && LBy > 0 && LBz > 0 && massB > 0);

  // Add the inclined plane and gravity to the plant.
  AddInclinedPlaneAndEarthGravityToPlant(
      gravity, slope_radians, is_inclined_plane_half_space, LAx, LAy, LAz,
      coefficient_friction_inclined_plane, plant);

  // Describe body B's mass, center of mass, and inertia properties.
  const Vector3<double> p_BoBcm_B = Vector3<double>::Zero();
  const UnitInertia<double> G_BBcm_B =
      UnitInertia<double>::SolidBox(LBx, LBy, LBz);
  const SpatialInertia<double> M_BBcm_B(massB, p_BoBcm_B, G_BBcm_B);

  // Create a rigid body B with the mass properties of a uniform solid block.
  const RigidBody<double>& blockB = plant->AddRigidBody("BodyB", M_BBcm_B);

  // Body B's visual geometry is always a solid box.
  // The pose X_BG of block B's geometry frame G is an identity transform.
  const math::RigidTransformd X_BG;   // Identity transform.
  const Vector4<double> lightBlue(0.5, 0.8, 1.0, 1.0);
  plant->RegisterVisualGeometry(blockB, X_BG,
                                geometry::Box(LBx, LBy, LBz),
                                "BlockB_VisualGeometry", lightBlue);

  // Block B's contacting surface is either a box or 4 spheres.
  if (is_block_with_4Spheres) {
    // There are four identical spheres welded to the block B's bottom surface.
    // The rotation matrix for each of these spheres is the 3x3 identity matrix.
    const double radius = LBz/2;
    const Vector3<double> p_BoSphere1_B(+LBx/2, +LBy/2, -radius);  // Downhill.
    const Vector3<double> p_BoSphere2_B(+LBx/2, -LBy/2, -radius);  // Downhill.
    const Vector3<double> p_BoSphere3_B(-LBx/2, +LBy/2, -radius);  // Uphill.
    const Vector3<double> p_BoSphere4_B(-LBx/2, -LBy/2, -radius);  // Uphill.
    const math::RigidTransform<double> X_BSphere1(p_BoSphere1_B);
    const math::RigidTransform<double> X_BSphere2(p_BoSphere2_B);
    const math::RigidTransform<double> X_BSphere3(p_BoSphere3_B);
    const math::RigidTransform<double> X_BSphere4(p_BoSphere4_B);

    // Register collision geometry for all 4 spheres.

    plant->RegisterCollisionGeometry(blockB, X_BSphere1,
                                     geometry::Sphere(radius),
                                     "Sphere1_CollisionGeometry",
                                     coefficient_friction_bodyB);
    plant->RegisterCollisionGeometry(blockB, X_BSphere2,
                                     geometry::Sphere(radius),
                                     "Sphere2_CollisionGeometry",
                                     coefficient_friction_bodyB);
    plant->RegisterCollisionGeometry(blockB, X_BSphere3,
                                     geometry::Sphere(radius),
                                     "Sphere3_CollisionGeometry",
                                     coefficient_friction_bodyB);
    plant->RegisterCollisionGeometry(blockB, X_BSphere4,
                                     geometry::Sphere(radius),
                                     "Sphere4_CollisionGeometry",
                                     coefficient_friction_bodyB);

    // Register visual geometry for all 4 spheres.
    const Vector4<double> red(1.0, 0.0, 0.0, 1.0);
    plant->RegisterVisualGeometry(blockB, X_BSphere1,
                                  geometry::Sphere(radius),
                                  "Sphere1_VisualGeometry", red);
    plant->RegisterVisualGeometry(blockB, X_BSphere2,
                                  geometry::Sphere(radius),
                                  "Sphere2_VisualGeometry", red);
    plant->RegisterVisualGeometry(blockB, X_BSphere3,
                                  geometry::Sphere(radius),
                                  "Sphere3_VisualGeometry", red);
    plant->RegisterVisualGeometry(blockB, X_BSphere4,
                                  geometry::Sphere(radius),
                                  "Sphere4_VisualGeometry", red);
  } else {
    // Block B's collision geometry is a solid box.
    plant->RegisterCollisionGeometry(blockB, X_BG,
                                     geometry::Box(LBx, LBy, LBz),
                                     "BlockB_VisualGeometry",
                                     coefficient_friction_bodyB);
  }
}

void AddInclinedPlaneWithSpherePlant(
    double gravity, double slope_radians,
    bool is_inclined_plane_half_space, double LAx, double LAy, double LAz,
    double radiusB, double massB,
    const CoulombFriction<double>& coefficient_friction_inclined_plane,
    const CoulombFriction<double>& coefficient_friction_bodyB,
    MultibodyPlant<double>* plant) {
  DRAKE_THROW_UNLESS(plant != nullptr);
  DRAKE_THROW_UNLESS(radiusB > 0 && massB > 0);

  // Add the inclined plane and gravity to the plant.
  AddInclinedPlaneAndEarthGravityToPlant(
      gravity, slope_radians, is_inclined_plane_half_space, LAx, LAy, LAz,
      coefficient_friction_inclined_plane, plant);

  // Describe body B's mass, center of mass, and inertia properties.
  const Vector3<double> p_BoBcm_B = Vector3<double>::Zero();
  const UnitInertia<double> G_BBcm = UnitInertia<double>::SolidSphere(radiusB);
  const SpatialInertia<double> M_BBcm_B(massB, p_BoBcm_B, G_BBcm);

  // Create a rigid body B with the mass properties of a uniform sphere.
  const RigidBody<double>& sphereB = plant->AddRigidBody("BodyB", M_BBcm_B);

  // Body B's visual geometry and collision geometry are a sphere.
  // The pose X_BG of block B's geometry frame G is an identity transform.
  const math::RigidTransformd X_BG;   // Identity transform.
  const Vector4<double> lightBlue(0.5, 0.8, 1.0, 1.0);
  plant->RegisterVisualGeometry(sphereB, X_BG,
                                geometry::Sphere(radiusB),
                                "SphereB_VisualGeometry", lightBlue);
  plant->RegisterCollisionGeometry(sphereB, X_BG,
                                   geometry::Sphere(radiusB),
                                   "SphereB_CollisionGeometry",
                                   coefficient_friction_bodyB);

  // Add little spherical spokes to highlight the sphere's rotation.
  const Vector4<double> red(1.0, 0.0, 0.0, 1.0);
  const double radius_spoke = radiusB / 5;
  const math::RigidTransformd X_B_Spoke1(Vector3<double>(0, 0, radiusB));
  const math::RigidTransformd X_B_Spoke2(Vector3<double>(0, 0, -radiusB));
  const math::RigidTransformd X_B_Spoke3(Vector3<double>(radiusB, 0, 0));
  const math::RigidTransformd X_B_Spoke4(Vector3<double>(-radiusB, 0, 0));
  plant->RegisterVisualGeometry(sphereB, X_B_Spoke1,
                                geometry::Sphere(radius_spoke),
                                "Spoke1_VisualGeometry", red);
  plant->RegisterVisualGeometry(sphereB, X_B_Spoke2,
                                geometry::Sphere(radius_spoke),
                                "Spoke2_VisualGeometry", red);
  plant->RegisterVisualGeometry(sphereB, X_B_Spoke3,
                                geometry::Sphere(radius_spoke),
                                "Spoke3_VisualGeometry", red);
  plant->RegisterVisualGeometry(sphereB, X_B_Spoke4,
                                geometry::Sphere(radius_spoke),
                                "Spoke4_VisualGeometry", red);
}

}  // namespace inclined_plane
}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake
