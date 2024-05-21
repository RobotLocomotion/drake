
// From contact_model_doxygen.h This was the intro text. Even if removed
// completely (most likely), we might
// want to use the text in the links?

// Previously with an
// @addtogroup drake_contacts
/*
Drake is concerned with the simulation of _physical_ phenomena, including
contact between simulated objects.
Drake approximates real-world physical contact phenomena with a combination
of geometric techniques and response models. Here we discuss the
parameterization and idiosyncrasies of a particular contact response model,
based on point contact, non-penetration imposed with a penalty force, and a
continuous model of friction approximating Coulomb stiction and sliding friction
effects.
This document gives an overview of the state of the implementation of
contact in Drake (as of Q2 2019) with particular emphasis on how to account for
its particular quirks in a well-principled manner. What works in one simulation
scenario, may not work equally well in another scenario. This discussion will
encompass:
- @ref contact_geometry "properties of the geometric contact techniques",
- @ref contact_parameters "choosing appropriate modeling parameters",
- @ref multibody_simulation "choosing a time advancement strategy", and
- @ref stribeck_approximation "details of the friction model".
*/


// Previously right after: <h2>Rigid Approximation to Contact</h2>
/*
<h3>%Point Contact</h3>
In the limit to rigid contact, often the region of contact, or contact surface,
shrinks to a single point. Notable exceptions are planar boundaries coming into
contact (box on a plane for instance) and rigid bodies with conforming
boundaries. Even in those cases it is common to model the surface contact by
selecting a few points on the surface and generating contact forces only at
those points.

@anchor point_contact_approximation
<h3>Numerical Approximation of %Point Contact</h3>
Contact determination in the limit to rigid is not feasible with floating point
arithmetic given that rigid bodies either lie exactly on top of each other or
are not in contact. In order for a practical implementation to detect contact
the bodies must be in an overlapping configuration, even if by a negligible
small amount, see Figure 1.

In a point contact model two bodies A and B are considered to be in contact if
the geometrical intersection of their original rigid geometries is non-empty.
That is, there exists a non-empty overlap volume. In a point contact model, this
overlap region is simply characterized by a pair of witness points `Aw` and `Bw`
on bodies A and B, respectively, such that `Aw` is a point on the surface of A
that lies the farthest from the surface of B. Similarly for point `Bw`. In the
limit to rigid, bodies do not interpenetrate, the intersection volume shrinks to
zero and in this limit witness points `Aw` and `Bw` coincide with each other.

<h3>Enforcing Non-Penetration with Penalty Forces</h3>
In Drake we enforce the non-penetration constraint for rigid contact using a
penalty force. This penalty force
introduces a "numerical" compliance such that, within this approximation, rigid
bodies are allowed to overlap with a non-empty intersection. The strength of
the penalty can be adjusted so that, in the limit to a very stiff penalty force
we recover rigid contact. In this limit, for which `Aw ≡ Bw`, a contact point
`Co` is defined as `Co ≜ Aw (≡ Bw)`. In practice, with a finite numerical
stiffness of the penalty force, we define `Co = 1/2⋅(Aw + Bw)`. Notice that the
1/2 factor is arbitrary and it's chosen for symmetry.
We define the normal `n̂` as pointing outward from the surface of
`B` towards the interior of `A`. That is, if we denote with `d` the
(positive) penetration depth, we have that `Bw = d⋅n̂ + Aw`. We define a contact
frame C with origin at `Co`. The C frame's z-axis is aligned along the normal n̂
(with arbitrary x- and y-axes). Because the two forces are equal and opposite,
we limit our discussion to the force `f` acting on `A` at `Co` (such that `-f`
acts on `B` at `Co`).

@image html drake/multibody/plant/images/simple_contact.png "Figure 1: Illustration of contact between two spheres."

The computation of the contact force is most naturally discussed in the
contact frame `C` (shown in Figure 1).
The contact force, `f`,  can be decomposed into two components: normal, `fₙ`,
and tangential, `fₜ` such that `f=fₙ+fₜ`. The normal force lies in the
direction of the contact frame's z-axis.  The tangential
component lies parallel to the contact frame's x-y plane.  In Drake's
contact model, the tangential force is a function of the normal force.
The detailed discussion of the contact force computation is decomposed into
two parts: a high-level engineering discussion addressed to end users who care
most about working with the model and a further detailed discussion of the
mathematical underpinnings of the implemented model. The practical guide should
be sufficient for most users.
*/