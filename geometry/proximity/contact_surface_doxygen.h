namespace drake {
namespace geometry {
namespace proximity {

/** @addtogroup module_contact_surface

 A contact surface is the surface of pressure equilibrium between two
 compliant objects in the hydroelastic contact model.

 Rigid objects do not have pressure fields, so we construct the contact
 surface between a rigid object N and a compliant object M from the
 intersection of N's surface and M's volume. Hydroelastic contact model does
 not support contacts between two rigid objects.

 <h2> Thin rigid objects </h2>

 When a rigid object N deeply penetrates into a soft object M, their contact
 surface may have undesirable parts. For example, the following picture shows
 a rigid thin plate in transparent grey deeply penetrates into a soft ball
 shown in transparent pink. The picture shows two surface patches of the
 contact surface for each side of the thin plate. The shading shows the
 pressure distribution on the surface patches.

 @image html geometry/proximity/images/contact_surface_thin_plate_two_sides.png "Two surface patches of the contact surface between a rigid thin plate and a soft ball."

 Without applying deformation during the simulation, we cannot be certain which
 part of the intersection(surface(N) ∩ volume(M)) should belong to the
 contact surface. In the above example, if the plate is originally outside the
 ball near the top of the ball, and the plate moves down to collide deeply into
 the ball, it is reasonable to expect that only the bottom surface patch, not
 the top surface patch, should define the contact surface. On the other hand,
 consider the case that the plate is originally outside the ball near the left
 of the ball, and the plate moves right to collide into the ball so deep that
 it cuts the ball into two pieces.  In this case, both surface patches should
 define the contact surface; one for each of the two pieces of the cut-away
 ball.

 Currently we assume that the first scenario (the plate moves downward) is the
 preference, so we impose a criterion that the intersection of a triangle in N
 and a tetrahedron in M belongs to the contact surface only when **the
 triangle's outward normal vector and the tetrahedron's pressure gradient
 vector make an angle less than a certain threshold.**  In the above example,
 with the angle threshold of π/2, we get only the bottom surface patch as
 shown in the following picture. The pressure gradient in the soft ball
 points towards the center of the ball.

 @image html geometry/proximity/images/contact_surface_thin_plate_one_side.png "The bottom surface patch satisfies the angle threshold π/2."

 Unfortunately there is no single angle threshold that works for all cases.
 For example, we added a rigid bowl into the above example, as shown in the
 following picture and tested it with different values of the angle threshold.
 The soft ball makes an edge contact with the bowl that is deep enough
 to overlap both sides of the bowl (the *inside* for carrying food and
 the *outside* that carries no food). Again, the rigid thin plate penetrates
 so deep into the soft ball that both sides of the plate overlap the ball.

 @image html geometry/proximity/images/contact_bowl_plate.png "Soft ball, rigid thin plate, and rigid bowl."

 The following pictures show the zoom-in of the contact surfaces between the
 soft ball, the rigid plate, and the rigid bowl. The angle threshold varies
 from π to π/2. The threshold π allows most triangles, and the threshold π/2
 allows least triangles.

 @image html geometry/proximity/images/contact_bowl_plate_angle_1.000PI.png "Angle threshold π."

 For the angle threshold π, both the thin plate contact and the bowl edge
 contact show incorrect "backside" contact surfaces. For the plate, the
 incorrect patch is disconnected from the correct one. For the bowl, the
 incorrect patch outside the bowl is connected to the correct patch inside
 the bowl.

 @image html geometry/proximity/images/contact_bowl_plate_angle_0.750PI.png "Angle threshold 3π/4, i.e., 0.75π."

 For the angle threshold 3π/4, the plate contact surface has an area of high
 pressure removed from the backside surface, and the bowl contact surface has
 only remnants of the backside surface left.

 @image html geometry/proximity/images/contact_bowl_plate_angle_0.625PI.png "Angle threshold 5π/8, i.e., 0.625π."

 For the angle threshold 5π/8, the bowl edge contact patch is correct, but some
 (low-pressure) remnants of the thin plate backside surface are present.

 @image html geometry/proximity/images/contact_bowl_plate_angle_0.500PI.png "Angle threshold π/2."

 For the angle threshold π/2, the plate contact surface is correct, but too
 many triangles on the bowl edge contact are removed.

 Currently we use the angle threshold 5π/8 and will work on more advanced
 criteria in the future.

 @note This page is still under construction.
 */

}  // namespace proximity
}  // namespace geometry
}  // namespace drake
