/** @file
 Doxygen-only documentation for @ref multibody_notation,
 @ref multibody_spatial_inertia.  */

// Developers: the subsections here can be linkably referenced from your Doxygen
// comments using
//    @ref group_name.
//
// For example, to link to the notation group: @ref multibody_notation
// To link to the Spatial Inertia discussion: @ref multibody_spatial_inertia.


//------------------------------------------------------------------------------
/** @addtogroup multibody_notation

Translating from the mathematics of multibody mechanics to correct code is a
difficult process and requires careful discipline to ensure that the resulting
code is correct. In Drake we use terminology, notation, and abstractions that
have been designed specifically to reduce the possibility of errors, based on
these observations:
- Good abstractions lead to good code.
- You can't reason properly about spatial algorithms if you treat translation
and rotation separately.
- Disciplined notation is essential to prevent coordinate frame errors.
- Coded algorithms should be comparable equation-by-equation against the typeset
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

<em><b>Developers</b>: you can link directly to specific discussion topics here
from your Doxygen comments; instructions are at the top of the source file used
to generate them.</em>

Next topic: @ref multibody_notation_basics
*/

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
*/

/** @defgroup multibody_frames_and_bodies Frames and Bodies
@ingroup multibody_notation

The _frame_ and _body_ are fundamental to multibody mechanics.
Unless specified otherwise, each _frame_ (also called a _coordinate frame_)
contains a right-handed orthogonal unitary _basis_ and an _origin_ point.  Its
name is usually one capital letter (e.g., A, B, or C). Shown below is a frame F.
Frame F's origin point Fo locates the frame and its basis orients the frame.
<pre>
     Fz
     ^    Fy          Frame F has origin point Fo and a
     |   /            right-handed orthogonal basis having
     |  /             unit vectors Fx, Fy, Fz.  The basis
     | /              is right-handed because Fz = Fx x Fy.
     o ------> Fx
     Fo
</pre>
Newton's laws of motion are valid in a non-rotating, non-accelerating "inertial
frame", herein called the _World_ frame W (also called _Ground_ frame G or
_Newtonian Frame_ N).  Any frame with fixed pose in W is also an inertial frame.
Drake supports _Model_ frames (inertial frames fixed in W) so a simulation can
be built from multiple independent models, each defined with respect to its own
Model frame. This corresponds to the `<model>` tag in an `.sdf` file.

In unambiguous situations, abbreviated notation uses the _frame_ name to also
designate the frame's _origin_ or the frame's _basis_.  For example, if `A` and
`B` are frames, `p_AB` denotes the position vector from point `Ao` (A's origin)
to point `Bo` (B's origin), expressed in frame A (i.e., expressed in terms of
unit vectors `Ax, Ay, Az`). Similarly, `w_AB` denotes frame B's angular velocity
in frame A, expressed in frame A.  `v_AB` denotes the translational velocity of
point Bo (B's origin) in frame A, expressed in frame A.  `V_AB` denotes frame
B's spatial velocity in frame A, expressed in frame A and is defined as the
combination of `w_AB` and `v_AB`.
See @ref multibody_quantities for more information about notation.

Each _body_ contains a _body frame_  and we use the same symbol `B` for both a
body `B` and its body frame. Body B's location is defined via `Bo` (the
origin of the body frame) and body B's pose is defined via the pose of B's
body frame.  Body properties (e.g., inertia and geometry) are measured with
respect to the body frame.  Body B's center of mass is denoted `Bcm`
(in typeset as @f$B_{cm}@f$) and its location is specified by a position vector
from Bo to Bcm.  Bcm is not necessarily coincident with Bo and body B's
translational and spatial properties (e.g., position, velocity, acceleration)
are measured using Bo (not Bcm).  If an additional frame is fixed to a rigid
body, its position is located from the body frame.
For a flexible body, deformations are measured with respect to the body frame.

When a user initially specifies a body, such as in a `<link>` tag of an `.sdf`
or `.urdf` file, there is a link frame L that may differ from Drake's body frame
B.  Since frames L and B are always related by a <b>constant</b> transform,
parameters (mass properties, visual geometry, collision geometry, etc.) given
with respect to frame L are transformed and stored internally with respect to
B's body frame.

<h3>Notation for offset frame</h3>
Sometimes we need a frame that is rigidly attached to a frame F with its basis
rigidly aligned to F's basis but with its origin shifted from Fo to a point R.
We call that an _offset frame_ and denote this offset frame in typeset notation
as @f$ F_R @f$. Since code lacks subscripts, we lowercase the point name to
make it look more like a subscript as `Fr`.  Recall that we permit frame names
and body names to also serve as points (by using their origins).  Suppose you
would like a frame that is regarded as rigidly attached to frame F but whose
origin is coincident with some body B. In this case, create an offset frame `Fb`
whose basis rigidly aligns with F's basis but whose origin is coincident with
Bo (B's origin).

Notation example: V_WB @f$(^WV^B)@f$ denotes the spatial velocity of a frame B
in World W. V_WBp @f$(^WV^{Bp)}@f$ denotes the spatial velocity of a frame
whose orientation is the same as B but whose origin is offset from Bo to be
coincident with a point P.  V_WBcm @f$(^WV^{Bcm})@f$ denotes the spatial
velocity of a frame whose orientation is the same as B but whose origin is
located at Bcm (B's center of mass).

If this notation is not sufficient for your purposes, please name the offset
frame and use comments to precisely describe the orientation of its basis and
the location of its origin.

Next topic: @ref multibody_quantities
*/

/** @defgroup multibody_quantities Multibody Quantities
@ingroup multibody_notation

Quantities of interest in multibody dynamics have distinct types. For example,
a rotation matrix is denoted with `R` and a position vector with `p`. New
quantities can be created by differentiation of an existing quantity (see
@ref Dt_multibody_quantities).

Most quantities have a
_reference_ and _target_, either of which may be a frame, basis, or point, that
specify how the quantity is defined. In computation, vectors are _expressed_ in
a specified basis (or frame) that associates a direction with each
vector element. For example, the velocity of a point P moving in a reference
frame F is a vector quantity with _target_ point P and _reference_ frame F.  In
typeset this symbol is written as @f$^Fv^P@f$. Here v is the quantity type, the
left superscript F is the reference, and the right superscript P is the target.
In computation, this vector is _expressed_ in a particular basis. By default,
the assumed expressed-in frame is the same as the _reference_ frame, so in this
case, the assumed expressed-in frame is frame F's basis. Alternately, to use a
different expressed-in frame, say W, typeset with the bracket notation:
@f$[^Fv^P]_W@f$.

Kane/monogram notation was developed decades ago for kinematics and dynamics.
It explicitly communicates the quantity being measured and how it is expressed.
The typeset symbol @f$^Fv^P@f$ is translated to monogram notation as `v_FP`.
The quantity type always comes first, then an underscore, then left and right
superscripts.  By default, the symbol `v_FP` implies the vector is expressed in
frame F. Alternately, to express in frame W, use the typeset @f$[^Fv^P]_W@f$ and
monogram notation `v_FP_W` (add a final underscore and expressed-in frame W).
We adhere to this pattern for all quantities and it is quite useful once you get
familiar with it. As a second example, consider the position vector of point Bcm
(body B's center of mass) from point Bo (the origin of frame B), expressed in B.
In explicit typeset, this is @f$[^{B_o}p^{B_{cm}}]_B @f$ whereas in implicit
typeset this is abbreviated @f$^Bp^{B_{cm}}@f$ (where the left-superscript B
denotes Bo and the expressed-in frame is implied to be B). The corresponding
monogram equivalents are `p_BoBcm_B` and `p_BBcm`, respectively.

Here are some more useful multibody quantities.

<!-- Developer note: Markdown tables are nice since you can read them in the
source file. However, each row must be specified on a single line of text. You
can violate the 80-character style guide limit if you have to, but be
reasonable! Alternately, use a footnote to avoid running over. -->

Quantity             |Symbol|     Typeset              | Monogram   | Meaningᵃ
---------------------|:----:|:------------------------:|:----------:|----------------------------
Rotation matrix      |  R   |@f$^BR^C@f$               |`R_BC`      |Frame C's orientation in frame B
Position vector      |  p   |@f$^Pp^Q@f$               |`p_PQ`      |Position vector from point P to point Q
Transform/pose       |  X   |@f$^BX^C@f$               |`X_BC`      |Frame C's *rigid* transform (pose) in frame B
General Transform    |  T   |@f$^BT^C@f$               |`T_BC`      |The relationship between two spaces -- it may be affine, projective, isometric, etc. Every X_AB can be written as T_AB, but not every T_AB can be written as X_AB.
Angular velocity     |  w   |@f$^B\omega^C@f$          |`w_BC`      |Frame C's angular velocity in frame Bᵃ
Velocity             |  v   |@f$^Bv^Q@f$               |`v_BQ`      |%Point Q's translational velocity in frame B
Spatial velocity     |  V   |@f$^BV^{C}@f$             |`V_BC`      |Frame C's spatial velocity in frame B (for point Co)ᵇ
Spatial velocity     |  V   |@f$^BV^{Cp}@f$            |`V_BCp`     |Frame C's spatial velocity in frame B (for point Cp)ᵇ
Angular acceleration |alpha |@f$^B\alpha^C@f$          |`alpha_BC`  |Frame C's angular acceleration in frame B
Acceleration         |  a   |@f$^Ba^Q@f$               |`a_BQ`      |%Point Q's translational acceleration in B
Spatial acceleration |  A   |@f$^BA^{C}@f$             |`A_BC`      |Frame C's spatial acceleration in frame B (for point Co)ᵇ
Spatial acceleration |  A   |@f$^BA^{Cp}@f$            |`A_BCp`     |Frame C's spatial acceleration in frame B (for point Cp)ᵇ
Torque               |  t   |@f$t^{B}@f$               |`t_B`       |Torque on a body (or frame) B
Force                |  f   |@f$f^{P}@f$               |`f_P`       |Force on a point P
Spatial force        |  F   |@f$F^{P}@f$               |`F_P`       |Spatial force (torque/force)ᶜ
Inertia matrix       |  I   |@f$I^{B/Bo}@f$            |`I_BBo`     |Body B's inertia matrix about Bo
Spatial inertia      |  M   |@f$M^{B/Bo}@f$            |`M_BBo`     |Body B's spatial inertia about Boᵃ
Spatial momentum     |  L   |@f$^AL^{S/P}@f$           |`L_ASP`     |System S's spatial momentum about point P in frame A
Spatial momentum     |  L   |@f$^AL^{S/Scm}@f$         |`L_AScm`    |System S's spatial momentum about point Scm in frame A
Jacobian wrt qᵈ      | Jq   |@f$[J_{q}^{{}^Pp^Q}]_E@f$ |`Jq_p_PQ_E` |Q's position Jacobian from P <b>in</b> E wrt q
Jacobian wrt q̇       | Jqdot|@f$J_{q̇}^{{}^Bv^Q}@f$     |`Jqdot_v_BQ`|Q's translational velocity Jacobian in B wrt q̇
Jacobian wrt v       | Jv   |@f$J_{v}^{{}^Bv^Q}@f$     |`Jv_v_BQ`   |Q's translational velocity Jacobian in B wrt v
Jacobian wrt v       | Jv   |@f$J_{v}^{{}^B\omega^C}@f$|`Jv_w_BC`   |C's angular velocity Jacobian in B wrt v

ᵃ In code, a vector has an expressed-in-frame which appears after the quantity.
<br>Example: `w_BC_E` is C's angular velocity in B, expressed in frame E, typeset
as @f$[^B\omega^C]_E @f$.
<br>Similarly, an inertia matrix or spatial inertia has an expressed-in-frame.
<br>Example: `I_BBo_E` is body B's inertia matrix about Bo,
expressed in frame E, typeset as @f$[I^{B/Bo}]_E@f$.
<br>For more information, see @ref multibody_spatial_inertia

ᵇ In code, spatial velocity (or spatial acceleration) has an expressed-in-frame
which appears after the quantity.
<br>Example: `V_BC_E` is frame C's spatial velocity in frame B, expressed in
frame E and contains both `w_BC_E` (described aboveᵃ) and v_BC_E (point Co's
translational velocity in frame B, expressed in frame E).  Reminder, a rigid
body D's translational and spatial velocity are for point Do (the origin of
D's body frame), not for Dcm (D's center of mass).
See @ref multibody_frames_and_bodies for more information.

ᶜ It is often useful to <b>replace</b> a set of forces by an equivalent set
with a force @f$f^{P}@f$ (equal to the set's resultant) placed at an arbitrary
point P, together with a torque @f$t@f$ equal to the moment of the set about
P.  A spatial force Fᴾ containing @f$t@f$ and @f$f^P@f$ can represent this
replacement.

ᵈ The Jacobian contains partial derivatives wrt (with respect to) scalars
e.g., wrt q (generalized positions), or q̇, or v (generalized velocities).
The example below shows the simplicity of Jacobian monogram:
first is the Jacobian symbol (Jv), next is the kinematic quantity (v_BQ),
last is an expressed-in frame (E).
<br>Example: `Jv_v_BQ_E` is `Jv` (Jacobian wrt v),
for `v_BQ` (velocity in frame B of point Q), expressed in frame E.
<br> <b>Advanced:</b> Due to rules of vector differentiation, explicit Jacobian
monogram notation for `Jq` (Jacobian wrt generalized positions q) requires an
extra frame (e.g., `JBq` is partial differentiation in frame B wrt q).
Frequently, the partial-differentiation-in-frame B is identical to the
expressed-in-frame E and a shorthand notation can be used.
<br>Example: `Jq_p_PQ_E` is `Jq` (Jacobian <b>in</b> frame E wrt q),
for `p_PQ` (position vector from point P to point Q),
expressed <b>in</b> frame E.
<br> <b>Special relationship between position and velocity Jacobians:</b>
When a point Q's position vector originates at a point Bo <b>fixed</b> in frame
B and when there are no motion constraints (no relationships between q̇₁ ... q̇ₙ),
@f$\;[J_{q}^{{}^{Bo}p^Q}]_B = [J_{q̇}^{{}^Bv^Q}]_B\;@f$ i.e.,
`(Jq_p_BoQ_B = Jqdot_v_BQ_B)`.

Monogram notation addresses frequently used kinematics.  In a context-sensible
way (and in collaboration with reviewers), extend notation to related concepts.
Some examples are shown below.
Extended notation         || Description (herein E is the vectors' expressed-in-frame)
:------------------------:||---------------------------------------------
p_PQset_E  | p_PQlist_E   |  Set of position vectors from a point P to each of the points in the set Q = {Q₀, Q₁, Q₂, ...}
v_BQset_E  | v_BQlist_E   |  Set of translational velocities in frame B for the set of points Q = {Q₀, Q₁, Q₂, ...}
w_BCset_E  | w_BClist_E   |  Set of angular velocities in frame B for the frames in the set C = {C₀, C₁, C₂, ...}
vset_E     | vlist_E      |  Set of generic vectors v = {v₀,  v₁,  v₂} expressed in frame E
mesh_B                    || A mesh whose underlying vertices' positions are from Bo (frame B's origin), expressed in frame B
point_cloud_B             || A point cloud whose underlying points' positions are from Bo (frame B's origin), expressed in frame B

 Next topic: @ref Dt_multibody_quantities
*/

//------------------------------------------------------------------------------
/** @defgroup Dt_multibody_quantities Time Derivatives of Multibody Quantities
@ingroup multibody_notation

<b>Scalar quantities</b>: The ordinary first time-derivative of the scalar x is
 denoted xdot or xDt whereas the ordinary second time-derivative of x is denoted
 xddot or xDDt.

<b>Vector quantities</b> (Advanced topic): The ordinary time-derivative of a
vector v (such as position or velocity) is different than the derivative of a
scalar.  A vector has direction whereas a scalar does not.  The derivative of
a vector requires a frame in which the derivative is being taken.
The typeset notation for the ordinary time-derivative in frame @f$ G @f$ of a
vector @f$ v @f$ is @f$ \frac{^Gd}{dt}\,v @f$ and its monogram notation is
`DtG_v`.  Since the derivative of a vector is a vector, we need to specify an
expressed-in frame E. The typeset notation is @f$ [\frac{^Gd}{dt}\,v]_E @f$
whereas the monogram notation is `DtG_v_E`. In unicode comments (e.g., in a
header or source file), use `[ᴳd/dt v]_E` or `DtG(v)_E` (see below).†

Important note: The derivative operator applies to the vector, _not_ the
computational representation of the vector. It is misleading to include
an expressed-in frame in the symbol name for the vector v.  The expressed-in
frame applies only to the final derived quantity. For example, consider
`V_BC`, frame C's spatial velocity in frame B (a spatial velocity contains
two vectors, namely angular velocity and velocity).  In code, you may express
`V_BC` in frame E as `V_BC_E`. The definition of the spatial acceleration `A_BC`
is the derivative in frame B of `V_BC` (the derivative <b>must</b> be in B).
However, the resulting expressed-in frame is arbitrary, e.g., a frame F.
The expressed-in frame E for `V_BC` does not impact the final result. The
monogram notation for this derivative is `DtB_V_BC_F` which is interpreted as
@f$ [\frac{^Bd}{dt}\,^BV^C]_F @f$; the `_F` goes with the result,
not the quantity being differentiated. The resulting vector happens to
be `A_BC_F`, but that is only because the derivative was taken in frame B.
If the derivative was taken in F (or C or E or any frame other than B), there
is <b>no</b> conventional spatial acceleration name or notation for the result
`DtF_V_BC`.

When using this DtFrame derivative notation in code, the expressed-in frame is
_always_ specified at the end of the symbol.  However there is _never_ an
expressed-in frame specified for the quantity being differentiated. For
example, given a vector v, the expression `DtG(v_E)_F` is misleading.
Instead, use `DtG(v)_F`.

† In unicode comments for the derivative in frame A of a vector v, use `ᴬd/dt v`
(preferred if available) or `DtA(v)`.  Although the former is preferred, not all
uppercase letters are available as superscripts in unicode.  Consider choosing
frame names to accommodate this strange quirk in unicode.

Next topic: @ref multibody_spatial_algebra
*/

//------------------------------------------------------------------------------
/** @defgroup multibody_spatial_algebra Spatial Algebra
@ingroup multibody_notation

Multibody dynamics involves both rotational and translational quantities, for
motion, forces, and mass properties. It is much more effective to group
related rotational and translational quantities together than to treat them
independently. We call such groupings *spatial* quantities.

Here we describe the important spatial quantities used in Drake's multibody
mechanics implementation, the terminology and notation we use to document them,
and their physical representations in code, typically as %Eigen objects.

Next topic: @ref multibody_spatial_pose
*/

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

@anchor orientation_discussion
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

<h3>Rigid transforms</h3>

A rigid transform combines position and orientation as defined above. We use the
quantity symbol @f$X@f$ for rigid transforms, so they appear as @f$^AX^B@f$ when
typeset and as `X_AB` in code using our monogram notation. We often say that
`X_AB` is the "pose" of frame B in frame A. In Drake this concept is provided by
the RigidTransform class. Conceptually, a transform is a 4×4 matrix structured
as follows: <pre>
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
homogeneous form of the position vector, `[x y z 1]ᵀ`. See Drake's documentation
for RigidTransform for a detailed discussion.

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
*/

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

While the rotational component of a spatial vector applies to a rigid body or
frame as a whole, the translational component refers to a particular point
rigidly fixed to that same body or frame. When assigned numerical values
for computation, both subvectors must be expressed in the same frame, which may
be that body's frame or any other specified frame. Thus, unambiguous notation
for spatial vectors must specify both a point and an expressed-in frame. Motion
quantities must also state the reference frame with respect to which the motion
is measured.

Example spatial quantity      |At |Exp|     Typeset        |   Code  |  Full
------------------------------|---|:-:|:------------------:|:-------:|:--------:
Body B's spatial velocity in A|Bo | A |@f$^AV^B         @f$|`V_AB`   |`V_ABo_A`
Same, but expressed in world  |Bo | W |@f$[^AV^B]_W     @f$|`V_AB_W` |`V_ABo_W`
B's spatial acceleration in W |Bcm| W |@f$^WA^{B_{cm}}  @f$|`A_WBcm` |`A_WBcm_W`
Spatial force acting on body B|Bcm| W |@f$[F^{B_{cm}}]_W@f$|`F_Bcm_W`|`F_BBcm_W`
Spatial force acting on body A| Q | W |@f$[F^{A/Q}]_W   @f$|`F_AQ_W` |    —

In the above table "At" is the point at which the translational activity occurs;
"Exp" is the expressed-in frame in which both vectors are expressed. The
expressed-in frame defaults to the reference (left) frame and the point
defaults to the target (right) frame origin. The "Code" column shows the
notation to use in code, using the available defaults; "Full" shows the code
notation with the defaults shown explicitly.

For spatial forces we need to identify the body (actually a frame) on which the
force is acting, as well as a point rigidly fixed to that body (or frame). When
the body is obvious from the point name (such as Bo or Bcm above), the body does
not need to be specified again. However, when the body is not clear it should be
listed before the point as in the last line of the table above. There it can be
read as "the point of body A coincident in space with point Q", where point Q
might be identified with a different body. You should use fully-expanded
symbols, and helpful comments, if there is any chance of confusion.

Next topic: @ref multibody_spatial_inertia
*/

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
*/
