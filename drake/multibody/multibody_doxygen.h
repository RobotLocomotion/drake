// Developers: the subsections here can be linkably referenced from your Doxygen
// comments using
//    @ref group_name
// or @ref anchor_name.
//
// For example, to link to the notation group: @ref multibody_notation
// To link to the Spatial Inertia discussion: @ref multibody_spatial_inertia.

//------------------------------------------------------------------------------
/** @defgroup multibody_concepts Multibody Dynamics Concepts

Translating from the mathematics of multibody mechanics to correct code is a
difficult process and requires careful discipline to ensure that the resulting
code is correct. In Drake we use terminology, notation, and abstractions that
have been designed specifically to reduce the possibility of errors, based on
these observations:
- Good abstractions lead to good code.
- You can't reason properly about spatial algorithms if you treat translation
and rotation separately.
- Coded algorithms should be comparable line-by-line against the typeset
literature sources from which they are derived.
- We need a shared, unambiguous notation in code that can employ programmers'
awesome pattern-matching skills to make errors visible.

In this section we provide general documentation about our terminology,
notation, and abstraction choices in a form that can be referenced from
code documentation. Much of the information here is most useful for
programmers who are writing or reading Drake multibody code. Some comments
are specifically targeted to those developers, and formatting compromises have
been made so that the information in this section is readable from the source
code used to generate it (for example, ASCII drawings instead of image files,
simple Markdown tables rather than fancy-but-unreadable html ones).
However, much of this can be useful to users of the Drake API also so
is included in the external documentation.

@warning Drake is under development and these concepts have not yet been
adopted consistently throughout the code. New code uses these concepts and
older code will be retrofitted over time. The documentation here applies to
the new `MultibodyTree` family of classes; there are some differences from the
earlier `RigidBodyTree` family.

<em><b>Developers</b>: you can link directly to specific discussion topics here
from your Doxygen comments; instructions are at the top of the source file used
to generate them.</em>
**/


//------------------------------------------------------------------------------
/** @defgroup multibody_notation Terminology and Notation
@ingroup multibody_concepts

Drake uses consistent terminology and notation for multibody mechanics that is
designed to provide clear, unambiguous documentation and a direct mapping
between typeset equations and their implementation in code.

The mathematics for multibody mechanics is complicated enough when typeset but
is particularly difficult to express in the restricted formatting permitted
in C++ variable names. We use a particular notation in code, called *Monogram
Notation*, that can be derived directly from the typeset equations that code is
implementing. Here we discuss the kinds of quantities we need to represent,
show them as typeset, and demonstrate how those are to be translated into code.

Please note that this compact notation is *not* intended as a substitute for
clear comments in code. Instead it is intended as an unambiguous specification
that can be used to compare code with the theory it implements, and to avoid
many common bugs just by rote pattern-matching of symbols.
**/

/** @defgroup multibody_frames_and_bodies Frames and Bodies
@ingroup multibody_notation

The most fundamental object in multibody mechanics is the _coordinate frame_, or
just _frame_. Unless specified otherwise, all frames we use are right-handed
Cartesian frames with orthogonal unit-vector axes x, y, and z forming a _basis_,
and an origin point O serving as the location of the frame. We use capital
letters to denote frames, such as A and B. Given a frame B, we denote its origin
as @f$B_O@f$ or `Bo` in code, and its axes as @f$B_x@f$ (`Bx`) etc. There is a
unique inertial frame commonly called the _World_ frame W, sometimes called
_Ground_ G or the _Newtonian Frame_ N. Any frame rigidly fixed to World is also
an inertial frame. (_Inertial_ here means not rotating and not accelerating.)
Drake supports the notion of a _Model_ frame, which has a fixed location with
respect to World, so that a simulation may be built up from multiple independent
models each defined with respect to its own Model frame. This corresponds to the
`<model>` tag in an `.sdf` file.

To simplify notation, we allow a frame to be specified in unambiguous contexts
where only a point or a basis is required. In those cases, either the frame
origin point, or the frame basis formed by its axes, are understood instead.

A _body_ is fundamentally a frame, so we use the same symbol for both a body
and its _body frame_. For example, a body B has an associated body frame B.
When we speak of the "location" or "pose" of a body, we really mean the location
of the body frame origin Bo or pose of the body frame B. Body properties such
as inertia and geometry are given with respect to the body frame. We denote
a body's center of mass point as Bcm; it's location on the body is specified
by a position vector from Bo to Bcm. For a rigid body, any frames attached to it
are fixed with respect to the body frame. For a flexible body, deformations are
measured with respect to the body frame.

When a user initially specifies a body, such as in a `<link>` tag of an `.sdf`
or `.urdf` file, there is a link frame L that may be distinct from the body
frame B that is used by Drake internally for computation. However, frames L and
B are always related by a constant transform that does not change during a
simulation. User-supplied information such as mass properties, visual geometry,
and collision geometry are given with respect to frame L; Drake transforms those
internally so that they are maintained with respect to B instead.
**/

/** @defgroup multibody_quantities Multibody Quantities
@ingroup multibody_notation

Quantities of interest in multibody dynamics have distinct types,
which we denote with a single letter. For example, a rotation matrix is
denoted with `R` and a position vector with `p`. Most quantities have a
_reference_ and _target_, either of which may be a frame, basis, or point, that
specify how the quantity is to be defined. In addition, for purposes of
computation, vector quantities
must be _expressed_ in a particular basis to provide numerical values for the
vector elements. For example, the velocity of a point is a vector quantity. To
define that quantity we must know the target point P, and the reference frame F
in which that point's velocity is to be measured. As a typeset symbol, we would
write that quantity @f$^F\!v^P@f$. Here v is the quantity type, the left
superscript is the reference, and the right superscript is the target. That is
the physical quantity of interest, but we have not said what basis we will
choose to give meaning to the numerical values for the vector. By default, we
always assume that the expressed-in frame is the same as the reference frame,
so in this case we would expect the vector expressed in frame F's basis. In case
we wish to use a different expressed-in frame, say W, we use bracket notation
to indicate how to write down the bracketed quantity: @f$[^F\!v^P]_W@f$.

Translating that to Monogram Notation in code we would write @f$^F\!v^P@f$ as
`v_FP`. The quantity type always comes first, then
an underscore, then left and right superscripts. That symbol implies that the
vector is expressed in the reference frame F. To express in a different frame,
we would write @f$[^F\!v^P]_W@f$ as `v_FP_W`, that is, add a final underscore
and expressed-in frame. We follow that pattern closely for all
quantities and it is quite useful once you get used to it. As a second example,
the position vector of body B's center of mass point Bcm, measured from
reference frame B and expressed in B would be @f$^B\!p^{B_{cm}}@f$ or more
explicitly, @f$[^{B_O}p^{B_{cm}}]_B @f$ where we have noted that it is the
body origin point serving as a reference, and that we will express the quantity
in the body frame. The Monogram equivalents are `p_BBcm` and `p_BoBcm_B`.

Here are some more useful multibody quantities.

<!-- Developer note: Markdown tables are nice since you can read them in the
source file. However, each row must be specified on a single line of text. You
can violate the 80-character style guide limit if you have to, but be
reasonable! Here I've used a footnote to avoid running over. -->

Quantity       |Symbol|   Typeset   | Code | Meaning
---------------|:----:|:-----------:|:----:|-------------------------
Position vector|  p   |@f$^P\!p^Q@f$|`p_PQ`|Vector from point P to Q †
Rotation Matrix|  R   |@f$^A\!R^B@f$|`R_AB`|Frame B's orientation in A
Transform      |  X   |@f$^A\!X^B@f$|`X_AB`|Frame B's pose in A

† expressed in point P's body's frame.
**/


//------------------------------------------------------------------------------
/** @defgroup multibody_spatial_algebra Spatial Algebra
@ingroup multibody_concepts

Multibody dynamics involves both rotational and translational quantities, for
motion, forces, and mass properties. It is much more effective to group
related rotational and translational quantities together than to treat them
independently. We call such groupings *spatial* quantities.

Here we describe the important spatial quantities used in Drake's multibody
mechanics implementation, the terminology and notation we use to document them,
and their physical representations in code, typically as %Eigen objects.
**/

/** @defgroup multibody_spatial_pose Spatial Pose
@ingroup multibody_spatial_algebra

TODO: Transforms, quaternions, rpy, pseudo-coordinates?
**/

/** @defgroup multibody_spatial_vectors Spatial Vectors
@ingroup multibody_spatial_algebra

Spatial vectors are 6-element quantities that are pairs of ordinary 3-vectors.
That is, Drake spatial vectors are logically elements of R³×R³, *not* R⁶; that
is, these are *not* Plücker vectors! However, we can still operate on them as
6-element column vectors so that we don't have to distinguish rotational from
translational operations. Elements 0-2 are always the rotational quantity,
elements 3-5 are translational. Here are the spatial vectors we use in Drake:

<pre>
      Spatial: Velocity  Acceleration   Force
                 ---         ---         ---    Eigen access
              0 |   |       |   |       |   |
    rotation  1 | ω |       | β |       | τ |   .head<3>()
              2 |   |       |   |       |   |
                 ---         ---         ---
              3 |   |       |   |       |   |
 translation  4 | v |       | a |       | f |   .tail<3>()
              5 |   |       |   |       |   |
                 ---         ---         ---
       Symbol:    V           A           F
</pre>
@note A spatial velocity is sometimes called a _twist_; a spatial force is
sometimes called a _wrench_. These terms have a variety of specific meanings in
the literature. For that reason, we generally avoid using them. However, if we
use them it will be in the general sense of "spatial velocity" and "spatial
force" as defined above rather than more-restrictive definitions from, for
example, screw theory.

When we need to refer to the underlying 3-vectors in a spatial vector, we use
the following symbols, with English alphabet substitutions for their Greek
equivalents:

 Code | Rotational quantity      | | Code | Translational quantity
:----:|--------------------------|-|:----:|-----------------------
  w   | ω - angular velocity     | |  v   | linear velocity
  b   | β - angular acceleration | |  a   | linear acceleration
  t   | τ - torque               | |  f   | force

While the rotational component of a spatial vector applies to a rigid body as a
whole, the translational component refers to a particular point on that same
body. When assigned numerical values for computation, both subvectors must be
expressed in the same frame, which may be that body's frame or any other
specified frame. Thus, unambiguous notation for spatial vectors must specify
both a point and an expressed-in frame. Motion quantities must also state the
reference frame with respect to which the motion is measured.

Example spatial quantity      |At |Exp|      Typeset       |   Code  | Expanded
------------------------------|---|:-:|:------------------:|:-------:|:--------:
Body B's spatial velocity in A|Bo | A |@f$^A\!V^B       @f$|`V_AB`   |`V_ABo_A`
Same, but expressed in world  |Bo | W |@f$[^AV^B]_W     @f$|`V_AB_W` |`V_ABo_W`
B's spatial acceleration in W |Bcm| W |@f$^W\!A^{B_{cm}}@f$|`A_WBcm` |`A_WBcm_W`
Spatial force applied to B    |Bcm| W |@f$[F^{Bcm}]_W   @f$|`F_Bcm_W`|` `

In the above table "At" is the point at which the translational activity occurs;
"Exp" is the expressed-in frame in which both vectors are expressed. Note that
the expressed-in frame defaults to the reference (left) frame and that the point
defaults to the target (right) frame origin. You should use fully-expanded
symbols if there is any chance of confusion.
**/

/** @defgroup multibody_spatial_inertia Spatial Mass Matrix (Spatial Inertia)
@ingroup multibody_spatial_algebra

A _Spatial Mass Matrix_ (also called _Spatial Inertia_) M represents the mass,
center of mass location, and inertia in a single 6×6 symmetric, mass-weighted
positive definite matrix that logically consists of four 3×3 submatrices.

<pre>
            Spatial mass matrix
           ---------- ----------
        0 |          |          |
        1 |  I(=mG)  |   m c×   |
        2 |          |          |
           ---------- ----------
        3 |          |          |
        4 |  -m c×   |    mE    |
        5 |          |          |
           ---------- ----------
               Symbol: M
</pre>
The symbols in the figure have the following meanings, assuming we have a
spatial inertia M for a body B, taken about a point P. For numerical
computation, the quantities must be expressed in a specified frame we'll denote
as F here.

Symbol | Meaning
:-----:|--------------------------------------------------------
   m   | mass, a nonnegative scalar
   I   | 3×3 inertia matrix, taken about P, expressed in F
   G   | 3×3 unit inertia (gyration matrix) about P, exp. in F
   c   | position vector from P to B's center of mass, exp. in F
   c×  | 3×3 cross product matrix of c
   E   | 3×3 identity matrix

In practice the 36-element spatial inertia has only 10 independent elements, so
we can represent it more compactly. Also, inertia is most effectively
represented as mass times a unit inertia, which permits numerically robust
representation of shape distribution for very small or zero masses.

Spatial inertia for a rigid body B is taken about a reference point P, and
expressed in some frame F. Often P=Bcm (B's center of mass) or P=Bo (B's
origin), and F=B (the body frame) or F=W (the world frame). We always document
these clearly and use the decorated symbol @f$ [^BM^P]_F @f$ = `M_BP_F` to mean
spatial inertia of body B about point P, expressed in frame F. For example,
`M_BBcm_W` (=@f$[^BM^{B_{cm}}]_W@f$) is the spatial inertia of body B taken
about its center of mass (the "central inertia") and expressed in the world
frame. We allow the body name and frame to be dropped if it is obvious from
the "about" point, so `M_Bcm` (@f$M^{B_{cm}}@f$) is the central spatial
inertia of body B, expressed in B. Inertia can be taken about any point. For
example `M_BWo_W` (@f$[^BM^{Wo}]_W@f$) is body B's spatial inertia taken about
the World frame origin, and expressed in World.

Given `M_BP_F` (@f$[^BM^P]_F@f$), its top left submatrix is `I_BP_F`
(@f$[^BI^P]_F@f$) and position vector c = `p_PBcm_F` (@f$[^Pp^{B_{cm}}]_F@f$),
that is, the position vector of the center of mass measured from point P and
expressed in F. Note that if the "taken about" point is `Bcm`, then c=0.
**/
