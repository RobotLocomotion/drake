// Developers: the subsections here can be linkably referenced from your Doxygen
// comments using
//    @ref group_name.
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
been made so that the information in this section is (mostly) readable from the
source code used to generate it (for example, ASCII drawings instead of image
files, simple Markdown tables rather than fancy-but-unreadable html ones).
However, much of this can be useful to users of the Drake API also so it
is included in the external documentation.

@warning Drake is under development and these concepts have not yet been
adopted consistently throughout the code. New code uses these concepts and
older code will be retrofitted over time. The documentation here applies to
the new `MultibodyTree` family of classes; there are some differences from the
earlier `RigidBodyTree` family.

<em><b>Developers</b>: you can link directly to specific discussion topics here
from your Doxygen comments; instructions are at the top of the source file used
to generate them.</em>

Next topic: @ref multibody_notation
**/


//------------------------------------------------------------------------------
/** @defgroup multibody_notation Terminology and Notation
@ingroup multibody_concepts

Drake uses consistent terminology and notation for multibody mechanics
- for clear communication among Drake programmers and users,
- to reduce the likelihood of errors in translating mathematical algorithms
  into code, and
- to facilitate verification of the code's correctness.

Where possible, we refer to published literature to supplement our code
documentation. That literature can provide clear, concise, and unambiguous
documentation of the equations and algorithms we are using, but employs typeset
mathematics to do so. We need to translate that mathematics into code, where
there is much less typographical flexibility. The Drake notation is designed to
facilitate comparison between the typeset and coded math in order to verify
correct implementation.

In addition, we need developers to provide good local documentation for our
mathematical algorithms in the form of
<a href=http://www.stack.nl/~dimitri/doxygen>Doxygen</a> comments. That
documentation will be best viewed after formatting by Doxygen, which employs
other tools like LaTeX and Markdown for beautification. However, for effective
development we need that documentation to be understandable and easily modified
by developers who are working directly in the source code. Maximizing the
post-Doxygen elegance of the formatted documentation can come at a steep cost in
source code readability and writability. And difficult-to-write documentation
often doesn't get written at all.

Perhaps not surprisingly, some notational compromises are necessary to achieve
these goals! When we do choose to use typeset mathematical notation, we avoid
typical typographical flourishes that would impede translation from math
notation to code. For example, we avoid script typefaces and putting arrows over
vectors. Also, since almost all interesting quantities in multibody dynamics are
vectors or matrices, we do not follow the common convention of using bold for
those quantities since (a) almost everything would be bold adding little
clarity, and (b) we can't do that in source code or comments. We do use a
limited set of super- and subscripts but define a rote
mapping from them to their code equivalents, as described below. We use Greek
symbols where conventional, such as ω for angular velocity, but define English
equivalents for use in variable names (`w` in this case). Where possible, we
prefer use of Unicode UTF-8 encoding in comments rather than LaTeX since UTF-8
is equally readable in the source and in the formatted output. At times we will
forego font purity where there is no chance of confusion. For example, we may
decide to format a complicated equation in LaTeX, where it will appear in
a typeset font like @f$A@f$ (which appears in the source as @c \@f\$A\@f\$),
but then refer in the text to A (source: just @c A) using the
default font that is much easier to write and to read in the source.

Next topic: @ref multibody_notation_basics
**/

// Developers: this document is somewhat of an exception to the motherhood-
// and-apple-pie goals above. We're going to show how our code-friendly
// notation maps to its typeset equivalents, and that means a lot of ugly LaTeX
// formatting here. But that doesn't mean you should write this way in your
// code comments if you can express the same thing in a more readable way!

/** @defgroup multibody_notation_basics Notation Basics
@ingroup multibody_notation

We are interested in representing physical quantities like position,
orientation, inertia, and spatial velocity. For discussion here, let @f$Q@f$
stand for an arbitrary physical quantity; we'll replace it with specific
quantity symbols later. For computation with vector quantities, we must choose a
_basis_ in which to express the individual numerical elements. A basis is a set
of independent direction vectors that spans a
space of interest; for example, the x,y,z axes of a coordinate frame span 3D
space. To _express_ a vector quantity in a particular basis means to form the
dot product of that vector with each of the basis vectors in turn, yielding
a single scalar each time. We call those scalars the _measure numbers_ of the
vector in that basis.

Any suitable basis may be chosen for computation; that choice does not change
the meaning of a physical quantity but is simply an agreed-upon convention for
writing down that quantity numerically. The chosen basis is called the
_expressed-in basis_; however, since our Cartesian bases are always
the axes of a coordinate frame, we more commonly refer to it as the
_expressed-in frame_. (See @ref multibody_frames_and_bodies for more information
about frames in Drake.)

Let the frame providing a basis be frame @f$F@f$. In typeset equations
we use bracket notation @f$[Q]_F@f$ to denote that quantity
@f$Q@f$ is being expressed in @f$F@f$'s basis @f$F_x,F_y,F_z@f$. In code or
comments, we translate this to _monogram notation_, described in detail below.
Here monogram notation would be `Q_F` (`Q` would be replaced by the actual
quantity). As a simple example, a vector @f$r@f$ expressed in
frame @f$W@f$ would be @f$[r]_W@f$ or `r_W` in code. For the remainder of this
documentation, we will generally use the
default font for single-letter variables because formatting them with
@f$\LaTeX@f$ or `code font` makes the source hard to read and write, and the
result does not add any clarity to the documentation. So in this document and in
most of Drake's Doxygen-formatted documentation, @f$F@f$≡`F`≡F. Individual Drake
programmers are _allowed_ to be fussier about single-letter typography but we
don't require it; readers should not infer any meaning to the font choice.
We are more careful about the font for more complex symbols and equations.

Physical quantities in general may be characterized by
- a symbol for the quantity type, e.g. @f$v@f$ for velocity or @f$I@f$ for
  inertia,
- a reference symbol (typically a body or frame, can be a point),
- a target symbol (can be a point, body, or frame), and
- an index for selecting a particular quantity from a collection (rarely
  needed in practice).

Quantities involving mass properties may have an additional "taken about" point;
we'll discuss that elsewhere.

(Note that a physical quantity does not have an expressed-in frame; that is
necessary only for numerical computation.)

Particular physical quantities may not have all of these characteristics. If
all are present, a generic physical quantity would look like this: @f[^RQ_i^T@f]
where R is the reference, T is the target, and i is the
index. In monogram notation, this would be `Qi_RT`. (We call this "monogram"
notation because the most important symbol--the quantity type--is listed first,
like the large last-name initial on a monogrammed shirt.) If an expressed-in
frame F must be specified we combine this with the bracket notation described
above and write @f[[^RQ_i^T]_F@f] or `Qi_RT_F` in code. For a real example,
the angular velocity of body B (the target) in body A (the reference) is
written @f$^Aω^B@f$ (monogram: `w_AB`). If that vector is to be expressed
numerically in the world frame W, we write @f$[^Aω^B]_W@f$
(monogram: `w_AB_W`). Monogram notation includes some defaults that are
commonly used to simplify symbols when the meaning is clear. In particular
if the reference symbol has an obvious frame, that basis is the default for
the expressed-in frame. For example, `w_AB` can be used instead of `w_AB_A` (you
can think of that as "no need to repeat the '_A' twice").
Other defaults will be noted as they are introduced.

Please note that this compact notation is *not* intended as a substitute for
clear comments in code. Instead it is intended as an unambiguous specification
that can be used to compare code with the theory it implements. It also
provides a means to avoid many common bugs because verification can be
achieved by rote pattern matching of symbols. The notation is often best
used in conjunction with some preliminary comments, including reminders about
the meanings of less-common symbols, and reference links to this background
documentation.

Next we discuss the kinds of quantities we need to account for and present their
typeset and code representations.

Next topic: @ref multibody_frames_and_bodies
**/

/** @defgroup multibody_frames_and_bodies Frames and Bodies
@ingroup multibody_notation

The most fundamental object in multibody mechanics is the _coordinate frame_, or
just _frame_. Unless specified otherwise, all frames we use are right-handed
Cartesian frames with orthogonal unit-vector axes x, y, and z forming a _basis_
that represents the frame's orientation, and an _origin_ point O serving as the
location of the frame. We use capital letters to denote frames, such as A and B.
Here is a typical Drake frame F:
<pre>
     Fz
     ^    Fy
     |   /
     |  /             Frame F
     | /
     o ------> Fx
     Fo
</pre>
Frame F's origin point is @f$F_O@f$ (`Fo` in code) and its basis is the set of
three mutually orthogonal unit vectors @f$F_x, F_y, F_z@f$ (`Fx`, `Fy`, `Fz`).
The basis is right-handed because @f$F_z = F_x \times F_y@f$.

There is a
unique inertial frame commonly called the _World_ frame W, sometimes called
_Ground_ G or the _Newtonian Frame_ N. (_Inertial_ here means not rotating and
not accelerating; any frame that has a fixed pose in W is also inertial.)
Drake supports the notion of a _Model_ frame, fixed in W and hence inertial,
so that a simulation may be built up from multiple independent
models each defined with respect to its own Model frame. This corresponds to the
`<model>` tag in an `.sdf` file.

To simplify notation, we allow a _frame_ to be specified in unambiguous contexts
where only a _point_ or a _basis_ is required. In those cases, either the frame
origin point, or the frame basis formed by its axes, are understood instead. You
can infer what is being used by looking at the quantity symbol. For example, for
a position vector `p_AB` we are using points `Ao` and `Bo`, while for an angular
velocity vector `w_AB` we are using the bases `Ax,Ay,Az` and `Bx,By,Bz`.

A _body_ is fundamentally a frame, so we use the same symbol for both a body
and its _body frame_. For example, a body B has an associated body frame B.
When we speak of the "location" or "pose" of a body, we really mean the location
of the body frame origin Bo or pose of the body frame B, respectively.
Body properties such
as inertia and geometry are given with respect to the body frame. We denote
a body's center of mass point as @f$B_{cm}@f$ (`Bcm`); it's location on the body
is specified by a position vector from Bo to Bcm. For a rigid body, any frames
attached to it are fixed with respect to the body frame. For a flexible body,
deformations are measured with respect to the body frame.

When a user initially specifies a body, such as in a `<link>` tag of an `.sdf`
or `.urdf` file, there is a link frame L that may be distinct from the body
frame B that is used by Drake internally for computation. However, frames L and
B are always related by a constant transform that does not change during a
simulation. User-supplied information such as mass properties, visual geometry,
and collision geometry are given with respect to frame L; Drake transforms those
internally so that they are maintained with respect to B instead.

Next topic: @ref multibody_quantities
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
write that quantity @f$^Fv^P@f$. Here v is the quantity type, the left
superscript is the reference, and the right superscript is the target. That is
the physical quantity of interest, but we have not said what basis we will
choose to give meaning to the numerical values for the vector. By default, we
always assume that the expressed-in frame is the same as the reference frame,
so in this case we would expect the vector expressed in frame F's basis. In case
we wish to use a different expressed-in frame, say W, we use bracket notation
to indicate how to write down the bracketed quantity: @f$[^Fv^P]_W@f$.

Translating that to monogram notation in code we would write @f$^Fv^P@f$ as
`v_FP`. The quantity type always comes first, then
an underscore, then left and right superscripts. That symbol implies that the
vector is expressed in the reference frame F. To express in a different frame,
we would write @f$[^Fv^P]_W@f$ as `v_FP_W`, that is, add a final underscore
and expressed-in frame. We follow that pattern closely for all
quantities and it is quite useful once you get used to it. As a second example,
the position vector of body B's center of mass point Bcm, measured from
reference frame B and expressed in B would be @f$^Bp^{B_{cm}}@f$ or more
explicitly, @f$[^{B_O}p^{B_{cm}}]_B @f$ where we have noted that it is the
body origin point serving as a reference, and that we will express the quantity
in the body frame. The monogram equivalents are `p_BBcm` and `p_BoBcm_B`,
respectively.

Here are some more useful multibody quantities.

<!-- Developer note: Markdown tables are nice since you can read them in the
source file. However, each row must be specified on a single line of text. You
can violate the 80-character style guide limit if you have to, but be
reasonable! Here I've used a footnote to avoid running over. -->

Quantity       |Symbol|  Typeset  | Code | Meaning
---------------|:----:|:---------:|:----:|-------------------------
Position vector|  p   |@f$^Pp^Q@f$|`p_PQ`|Vector from point P to Q †
Rotation Matrix|  R   |@f$^AR^B@f$|`R_AB`|Frame B's orientation in A
Transform      |  X   |@f$^AX^B@f$|`X_AB`|Frame B's pose in A

† expressed in point P's body's frame.

<!-- TODO(sherm1,mitiguy) fill out above table? -->

Next topic: @ref multibody_spatial_algebra
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

Next topic: @ref multibody_spatial_pose
**/

/** @defgroup multibody_spatial_pose Spatial Pose and Transform
@ingroup multibody_spatial_algebra

A _spatial pose_, more commonly just _pose_, provides the location and
orientation of a frame B with respect to another frame A. A _spatial transform_,
or just _transform_, is the "verb" form of a pose. It is a linear operator that
is easily constructed from the pose information as we will show below. A
transform can be used to map a point whose location
is known in frame B to that same point's location in frame A. We'll discuss
location and orientation separately and then show how they are combined to form
a convenient spatial quantity.

<h3>Location</h3>

The location of a point S in a frame A is given by a position vector
@f$^Ap^S@f$, measured from A's origin Ao. When used for computation, we assume
this vector is expressed in A's basis. When useful for clarity, the basis
can be shown explicitly as @f$[^{A_O}p^S]_A@f$. In monogram notation,
we write these symbols as `p_AS` and `p_AoS_A`, respectively. When used in a
pose, we are
interested in the location of frame B's origin Bo in A, @f$^Ap^{B_O}@f$
(`p_ABo`), or more explicitly @f$[^{A_O}p^{B_O}]_A@f$ (`p_AoBo_A`).

<h3>Orientation</h3>

A rotation matrix R, also known as a direction cosine matrix, is an orthogonal
3×3 matrix whose columns and rows are directions (that is, unit vectors) that
are mutually orthogonal. Furthermore, if the columns (or rows) are labeled x,y,z
it always holds that z = x X y (rather than -(x X y)) ensuring that this is a
right-handed rotation matrix. This is equivalent to saying
that the determinant of a rotation matrix is 1, not -1.

A rotation matrix can be considered the "verb" form of a basis that represents
the orientation of a frame F in another frame G.
The columns of a rotation matrix are simply the basis F, and the rows are the
basis G. The rotation matrix can then be used to re-express quantities from one
basis to another. Suppose we have a vector r_F expressed in terms of the
right-handed, orthogonal basis Fx, Fy, Fz and would like to express r instead
as r_G, in terms of a right-handed, orthogonal basis Gx, Gy, Gz. To calculate
r_G, we form a rotation matrix @f$^GR^F@f$ (`R_GF`) whose columns are the F
basis vectors re-expressed in G: <pre>
          ---- ---- ----
         |    |    |    |
  R_GF = |Fx_G|Fy_G|Fz_G|
         |    |    |    |
          ---- ---- ----  3×3
where
          -----           -----           -----
         |Fx⋅Gx|         |Fy⋅Gx|         |Fz⋅Gx|
  Fx_G = |Fx⋅Gy|  Fy_G = |Fy⋅Gy|  Fz_G = |Fz⋅Gy|
         |Fx⋅Gz|         |Fy⋅Gz|         |Fz⋅Gz|
          -----           -----           -----  3×1
</pre>
In the above, v⋅w=vᵀw is the dot product of two vectors v and w. (Looking at
the element definitions above, you can see that the rows are the G basis
vectors re-expressed in F.) Now we can re-express the vector r from frame F to
frame G via <pre>
     r_G = R_GF * r_F.
</pre>
Because a rotation is orthogonal, its transpose is its inverse. Hence
`R_FG = (R_GF)⁻¹ = (R_GF)ᵀ`. (In %Eigen that is `R_GF.transpose()`). This
transposed matrix can be used to re-express r_G in terms of Fx, Fy, Fz as <pre>
     r_F = R_FG * r_G  or  r_F = R_GF.transpose() * r_G
</pre>
In either direction, correct behavior can be obtained by using the
recommended notation and then matching up the frame labels pairwise left
to right (after interpreting the `transpose()` operator as reversing the
labels). Rotations are easily composed, with correctness assured by pairwise
matching of frame symbols: <pre>
    R_WC = R_WA * R_AB * R_BC.
</pre>

We generally use quaternions as a more-compact representation of a rotation
matrix, and use the Eigen::Quaternion class to represent them. Conceptually,
a quaternion `q_GF` has the same meaning and can be used in the same way as
the equivalent rotation matrix `R_GF`.

<h3>Transforms</h3>

A transform combines position and orientation so contains a pose as defined
above. We use the quantity symbol @f$X@f$ for transforms, so they appear as
@f$^AX^B@f$ when typeset and `X_AB` in code. Drake uses the `Isometry3` variant
of the Eigen::Transform class to represent transforms. ("Isometry" indicates
that the transform preserves lengths, that is, it does not scale or shear but
only translates and rotates.) Conceptually, a transform is a 4×4 matrix
structured as follows: <pre>
          --------- ----     ---- ---- ---- ----
         |         |    |   |    |    |    |    |
         |  R_GF   |p_GF|   |Fx_G|Fy_G|Fz_G|p_GF|
  X_GF = |         |    | = |    |    |    |    |
         | 0  0  0 | 1  |   |  0 |  0 |  0 | 1  |
          --------- ----     ---- ---- ---- ----  4×4
</pre>
There is a rotation matrix in the upper left 3×3 block (see above), and a
position vector in the first 3×1 elements of the rightmost column. Then the
bottom row is `[0 0 0 1]`. The rightmost column can also be viewed as the
homogenous form of the position vector, `[x y z 1]ᵀ`. See %Eigen's documentation
for Eigen::Transform for a detailed discussion.

A transform may be applied to position vectors to translate the measured-from
point to a different frame origin, and to re-express the vector in that frame's
basis. For example, if we know the location of a point P
measured in and expressed in frame A, we write that `p_AP` (or `p_AoP_A`) to
mean the vector from A's origin Ao to the point P, expressed in A. If we want
to know the location of that same point P, but measured in and expressed in
frame B, we can write: <pre>
    p_BP = X_BA * p_AP.
</pre> The inverse of a transform reverses the superscripts so <pre>
    X_FG = (X_GF)⁻¹
</pre> The inverse has a particularly simple form. Given `X_GF` as depicted
above, `X_FG` is <pre>
          --------- -------------     --------- ----
         |         |             |   |         |    |
         | (R_GF)ᵀ |−(R_GF)ᵀ*p_GF|   |  R_FG   |p_FG|
  X_FG = |         |             | = |         |    |
         | 0  0  0 |      1      |   | 0  0  0 | 1  |
          --------- --------------    --------- ----
</pre>
Transforms are easily composed, with correctness assured by pairwise matching of
frame symbols: <pre>
    X_WC = X_WA * X_AB * X_BC.
</pre>

Next topic: @ref multibody_spatial_vectors
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
    rotation  1 | ω |       | α |       | τ |   .head<3>()
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

 Code | Rotational quantity      || Code | Translational quantity
:----:|--------------------------||:----:|-----------------------
  w   | ω - angular velocity     ||  v   | linear velocity
alpha | α - angular acceleration ||  a   | linear acceleration
  t   | τ - torque               ||  f   | force

While the rotational component of a spatial vector applies to a rigid body as a
whole, the translational component refers to a particular point on that same
body. When assigned numerical values for computation, both subvectors must be
expressed in the same frame, which may be that body's frame or any other
specified frame. Thus, unambiguous notation for spatial vectors must specify
both a point and an expressed-in frame. Motion quantities must also state the
reference frame with respect to which the motion is measured.

Example spatial quantity      |At |Exp|     Typeset        |   Code
------------------------------|---|:-:|:------------------:|:-------:
Body B's spatial velocity in A|Bo | A |@f$^AV^B         @f$|`V_AB`
Same, but expressed in world  |Bo | W |@f$[^AV^B]_W     @f$|`V_AB_W`
B's spatial acceleration in W |Bcm| W |@f$^WA^{B_{cm}}  @f$|`A_WBcm`
Spatial force applied to B    |Bcm| W |@f$[F^{B_{cm}}]_W@f$|`F_Bcm_W`

In the above table "At" is the point at which the translational activity occurs;
"Exp" is the expressed-in frame in which both vectors are expressed. Note that
the expressed-in frame defaults to the reference (left) frame and that the point
defaults to the target (right) frame origin. You should use fully-expanded
symbols if there is any chance of confusion.

Next topic: @ref multibody_spatial_inertia
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

Spatial inertia for a rigid body B is taken about a point P, and
expressed in some frame F. Often P=Bcm (B's center of mass) or P=Bo (B's
origin), and F=B (the body frame) or F=W (the world frame). We always document
these clearly and use the decorated symbol @f$ [M^{B/P}]_F @f$ = `M_BP_F` to
mean spatial inertia of body B about point P, expressed in frame F. For example,
`M_BBcm_W` (=@f$[M^{B/B_{cm}}]_W@f$) is the spatial inertia of body B taken
about its center of mass (the "central inertia") and expressed in the world
frame. We allow the body name and frame to be dropped if it is obvious from
the "about" point, so `M_Bcm` (@f$M^{B_{cm}}@f$) is the central spatial
inertia of body B, expressed in B. Inertia can be taken about any point. For
example `M_BWo_W` (@f$[M^{B/Wo}]_W@f$) is body B's spatial inertia taken about
the World frame origin, and expressed in World.

Given `M_BP_F` (@f$[M^{B/P}]_F@f$), its top left submatrix is `I_BP_F`
(@f$[I^{B/P}]_F@f$) and position vector c = `p_PBcm_F` (@f$[^Pp^{B_{cm}}]_F@f$),
that is, the position vector of the center of mass measured from point P and
expressed in F. Note that if the "taken about" point is `Bcm`, then c=0.
**/
