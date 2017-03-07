/** @defgroup contact_model_concepts Contact Model Concepts

 Drake uses a _compliant_ contact model.  As two bodies collide, penetration is
 allowed.  The geometry of the penetration, in combination with the kinematics
 of the bodies, will produce a contact force.  This module describes the
 specific compliant contact model used in Drake and guidance on how
 to troubleshoot undesirable behavior.

 Next topic: @ref contact_model
*/

/** @defgroup contact_model
 @ingroup contact_model_concepts

 For Drake's purposes, "contact" is defined as follows;

 - Relationship between two DrakeCollision::Element instances (note: _not_
 RigidBody instances).
 - Only exists if the Element instances overlap.
 - Characterized by a single contact point and a normal direction (both the
 point and the normal vector are measured and expressed in a common frame --
 typically the world frame).

 Given such a contact, the contact model produces two equal-but-opposite forces
 which will act on the RigidBody instances to which the collision Element
 instances are assigned.  Because the forces are equal but opposite, we will
 define only one of the two forces.

 We can name the two colliding Element instances element `A` and `B`. By
 convention, we define the contact normal as pointing from `B` towards `A`.

 The computation of the contact force is most cleanly discussed in the
 _contact frame_.  We define the contact frame such that it's z-axis is aligned
 with the contact normal but place no constraints on the directions of its x-
 and y-axes.  The contact frame's origin is at the contact point.

 The contact force can be decomposed into two components: normal and tangential.
 The normal force lies in the normal direction and, therefore, in the direction
 of the contact frame's z-axis.  The tangential component lies on the contact
 frame's x-y plane.  In Drake's compliant contact model, these forces are _not_
 independent (the tangential force is a function of the normal force.)

 @section normal_force Hunt-Crossley Normal Force
 @section tangent_force Stribeck Friction Tangential Force

*/