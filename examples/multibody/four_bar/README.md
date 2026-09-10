# Four-Bar Linkage Examples

These two examples model the same kind of mechanism -- a planar four-bar
linkage -- and illustrate two different ways to deal with the closed kinematic
loop that a four-bar forms:

* `four_bar` loads the loop from an SDF model file and lets MultibodyPlant model
  it automatically, closing it with a constraint that `Finalize()` adds. This is
  what ordinary use of that feature looks like.
  Note: this uses a Drake feature that is currently experimental.
* `four_bar_with_bushing` leaves one of the four joints out and stands a
  compliant bushing in for it, so the remaining joints form a tree.

# Automatic loop modeling: `four_bar`

This example loads a four-bar linkage from an SDF model file that describes it
the way you would draw it: as a plain loop of four revolute joints, with no
attempt to specify a spanning tree and no stand-in for the "extra" joint. Beyond
opting in with `MultibodyPlant::SetEnableLoopTopology()` there is nothing to do
about the loop, and the rest of the program is an ordinary passive simulation.

`Finalize()` is where the modeling happens: it breaks the loop by adding a
"shadow" copy of one link, retargeting one joint onto the shadow, and adding a
weld constraint that holds the shadow to the link it was split from. Nothing
else has to know that any of that happened -- this example never mentions the
shadow link, and nothing draws it.

Closing the loop takes a constraint, so this model needs a solver that can
enforce one, and it does not care which: it runs discrete under SAP, which is
the default here, and continuous under CENIC.

The model file, `four_bar.sdf`, describes the linkage in an **assembled**
configuration, as it must. SDF and URDF give a joint a single "joint frame",
rather than Drake's more general model in which a joint connects an independent
parent frame to an independent child frame, and both of Drake's frames then have
to be derived from that one. So a parsed model is loop-consistent at q = 0 no
matter what poses its links are given: give the links poses that do not close
the loop and the discrepancy does not show up as a loop waiting to be closed,
but is absorbed into the joint offsets instead, silently building a mechanism
other than the one drawn. (The USD file format can express Drake's joint model,
so this is a limitation of these two formats rather than of loop modeling.) See
the comment at the top of `four_bar.sdf` for a figure of the assembled linkage
along with its dimensions and masses.

## Running four_bar

To run with default flags:

```
bazel run //examples/multibody/four_bar:four_bar
```
You'll see output like this:
```
[console] [info] Meshcat listening for connections at http://localhost:7000

The linkage is shown as the model file defines it.
Press Enter to simulate . . .
```
Open that URL in a browser to see the linkage, then press Enter and watch it
swing passively under gravity. Note what the example never mentions: the split
link, the constraint that closes the loop, or how well the loop is holding
together. Keeping it closed is the solver's job, and there is nothing here to
check up on or intervene in.

To run the same model continuously with the CENIC integrator instead of the
default discrete SAP solver:

```
bazel run //examples/multibody/four_bar:four_bar -- \
    --time_step=0 --simulator_integration_scheme=cenic
```

To drive the linkage with a constant torque at the `world_driver` joint, which
is the only actuated one, rather than letting it swing passively, and to run
start to finish without stopping for Enter:

```
bazel run //examples/multibody/four_bar:four_bar -- \
    --applied_torque=5 --nointeractive
```

`--help` lists all the options, along with the standard Drake simulator flags
such as `--simulator_target_realtime_rate`:

```
bazel run //examples/multibody/four_bar:four_bar -- --help
```

# Closing the loop with a bushing: `four_bar_with_bushing`

A compliant bushing is another way to close a four-bar's kinematic loop: leave
one of the four revolute joints out of the model, so that the remaining three
form a tree, and stand a bushing in for the joint that is missing. Unlike the
loop constraint in `four_bar`, which the solver enforces, a bushing is a force
element whose stiffness and damping you choose yourself, and the rest of this
file is largely about how to choose them.

Note that the figure and dimensions below describe *this* example; the
automatically modeled one above has its own geometry.

This planar four-bar linkage demonstrates how to use a bushing to
approximate a closed kinematic chain. It loads an SDF model from the
file "four_bar_with_bushing.sdf" into MultibodyPlant. It handles the closed
kinematic chain by replacing one of the four-bar's revolute (pin) joints with a
bushing ([drake::multibody::LinearBushingRollPitchYaw](https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_linear_bushing_roll_pitch_yaw.html))
whose force stiffness and damping values were approximated as discussed below.
An alternative way to close this four-bar's kinematic chain is to "cut"
one of the four-bar's rigid links in half and join those halves with a
bushing that has both force and torque stiffness/damping. Note: the links
in this example are constrained to rigid motion in the world X-Z
plane (bushing X-Y plane) by the 3 revolute joints specified in the
SDF. Therefore it is not necessary for the bushing to have force
stiffness/damping along the joint axis.

To run with default flags:

```
bazel run //examples/multibody/four_bar:four_bar_with_bushing
```

You should see the four-bar model oscillating passively with a small initial
velocity.

To change the initial velocity of `joint_WA`, q̇A in radians/second :
```
bazel run //examples/multibody/four_bar:four_bar_with_bushing -- --initial_velocity=<desired_velocity>
```

You can also apply a constant torque, 𝐓ᴀ, to `joint_WA` with a command line
argument:
```
bazel run //examples/multibody/four_bar:four_bar_with_bushing -- --applied_torque=<desired_torque>
```
The torque is applied constantly to the joint actuator with no feedback. Thus,
 if set high enough, you will see the system become unstable. 

You can change the bushing parameters from the command line to observe their
effect on
the modeled joint. For instance, change `force_stiffness` to 300:
 ```
bazel run //examples/multibody/four_bar:four_bar_with_bushing -- --force_stiffness=300
```
And observe a gradual displacement between link *B* and link *C*.

Change `force_damping` to 0:
 ```
bazel run //examples/multibody/four_bar:four_bar_with_bushing -- --force_damping=0
```
And observe the joint oscillating.

Try setting `applied_torque` to 1000 and watch how the large forces interact
with the bushing stiffness.


## Four-bar linkage model

The figure below shows a planar four-bar linkage consisting of 
frictionless-pin-connected uniform rigid links *A, B, C* and ground-link *W*.
- Link *A* connects to *W* and *B* at points *A*ₒ and *B*ₒ
- Link *B* connects to *A* and *C* at points *B*ₒ and *B*c
- Link *C* connects to *W* and *B* at points *C*ₒ and *C*ʙ

Right-handed orthogonal unit vectors **Âᵢ B̂ᵢ Ĉᵢ Ŵᵢ** 
(*i = x,y,z*) are fixed in *A, B, C, W,* with:
- **Â**𝐱 directed from *A*ₒ to *B*ₒ
- **B̂**𝐱 directed from *B*ₒ to *B*c
- **Ĉ**𝐱 directed from *C*ₒ to *C*ʙ
- **Ŵ**𝐳 vertically-upward.
- **Â**𝐲 = **B̂**𝐲 = **Ĉ**𝐲 = **Ŵ**𝐲 parallel to pin axes

| Diagram of the four bar model described above. |
| :---: |
| ![FourBarLinkageSchematic](images/FourBarLinkageSchematic.png)    |
|  |

|                 Quantity                   |       Symbol      |   Value   |
|--------------------------------------------|-------------------|-----------|
| Distance between *W*ₒ and *C*ₒ             |         𝐋ᴡ        |    2 m    |
| Lengths of links *A, B, C*                 |        *L*        |    4 m    |
| Masses of *A, B, C*                        |        *m*        |   20 kg   |
| Earth’s gravitational acceleration         |        *g*        | 9.8 m/s²  |
|                                            |                   |           |
| **Ŵ**𝐲 measure of motor torque on *A*      |         𝐓ᴀ        | Specified |
| Angle from **Ŵ**𝐱 to **Â**𝐱 with a +**Ŵ**𝐲 sense |    𝐪ᴀ    | Variable |
| Angle from **Â**𝐱 to **B̂**𝐱 with a +**Â**𝐲 sense |    𝐪ʙ    | Variable |
| Angle from **Ŵ**𝐱 to **Ĉ**𝐱 with a +**Ŵ**𝐲 sense |    𝐪ᴄ    | Variable |
|                                            |               |           |
| "Coupler-point" *P*'s position from *B*ₒ   |     2 **B̂**𝐱 - 2 **B̂**𝐳 |

With 𝐓ᴀ = 0, the equilibrium values for the angles are:
𝐪ᴀ ≈ 75.52°, 𝐪ʙ ≈ 104.48°, 𝐪ᴄ ≈ 104.48°.

## Starting Configuration

The SDF defines all of the links with their x axes parallel to the world x
axis for convenience of measuring the angles in the state of the system
with respect to a fixed axis. Below we derive a valid initial configuration
of the three angles 𝐪ᴀ, 𝐪ʙ, and 𝐪ᴄ.

| Derivation of starting configuration |
| :---: |
| ![FourBarLinkageSchematic](images/FourBarLinkageGeometry.png)    |
| |

Due to equal link lengths, the initial condition (static equilibrium) 
forms an isosceles trapezoid and initial values can be determined from
trigonometry. 𝐪ᴀ is one angle of a right triangle with its adjacent
side measuring 1 m and its hypotenuse measuring 4 m.  Hence, initially
𝐪ᴀ = tan⁻¹(√15) ≈ 1.318 radians ≈ 75.52°.

Because link *B* is parallel to **Ŵ**𝐱, 𝐪ᴀ and 𝐪ʙ are supplementary,
hence the initial value is 𝐪ʙ = π - 𝐪ᴀ ≈ 1.823 radians ≈  104.48°.
Similarly, 𝐪ᴀ and 𝐪ᴄ are supplementary, so initially 𝐪ᴄ = 𝐪ʙ. 

# Modeling the revolute joint between links B and C with a bushing

In this example, we replace the pin joint at point **Bc** (see diagram)
that connects links *B* and *C* with a
[drake::multibody::LinearBushingRollPitchYaw](https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_linear_bushing_roll_pitch_yaw.html)
(there are many other uses of a bushing).  We model a z-axis revolute joint by
setting torque stiffness constant k₂ = 0 and  torque damping constant d₂ = 0.
We chose the z-axis (Yaw) to avoid a singularity associated with "gimbal lock".
Two frames (one attached to *B* called `Bc_Bushing` with origin at point
**Bc** and one attached to *C* called `Cb_Bushing` with origin at point
**Cb**) are oriented so their z-axes are perpedicular to the planar
four-bar linkage.

## Estimating bushing parameters
Joints are normally modeled with hard constraints except in their motion
direction, and three of the four revolute joints here are indeed modeled
that way. However, in order to close the kinematic loop we have to use a
bushing as a "penalty method" substitute for hard constraints. That is, because
the bushing is compliant it will violate the constraint to some degree. The
stiffer we make it, the more precisely it will enforce the constraint but
the more difficult the problem will be to solve numerically. We want to
choose stiffness k and damping d for the bushing to balance those
considerations. First, consider your tolerance for constraint errors -- if
the joint allows deviations of 1mm (say) would that be OK for your application?
Similarly, would angular errors of 1 degree (say) be tolerable? We will give
a procedure below for estimating a reasonable value of k to achieve a
specified translational and rotational tolerance. Also, we need to choose
d to damp out oscillations caused by the stiff spring in a "reasonable" time.
Consider a time scale you would consider negligible. Perhaps a settling time
of 1ms (say) would be ignorable for your robot arm, which presumably has
much larger time constants for important behaviors. We will give a
procedure here for obtaining a reasonable d from k and your settling
time tolerance. For a more detailed discussion on choosing bushing parameters
for a variety of its uses, see [drake::multibody::LinearBushingRollPitchYaw](https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_linear_bushing_roll_pitch_yaw.html).

### Estimate force stiffness [kx ky kz] from loading/displacement
The bushing's force stiffness constants [kx ky kz] can be
approximated via various methods (or a combination thereof).
For example, one could specify a maximum bushing displacement in a
direction (e.g.,  xₘₐₓ), estimate a maximum directional load (Fx) that
combines gravity forces, applied forces, inertia forces (centripetal,
Coriolus, gyroscopic), and then calculate kx ≈ Fx /  xₘₐₓ.

### Estimate force stiffness [kx ky kz] constants from mass and ωₙ
The bushing's force stiffness constants [kx ky kz] can be
approximated via a related linear constant-coefficient 2ⁿᵈ-order ODE:

|  |  |
| ----- | ---- |
|  m ẍ +     dₓ ẋ +  kₓ x = 0  |  or alternatively as |
|    ẍ + 2 ζ ωₙ ẋ + ωₙ² x = 0  |  where ωₙ = √(kₓ/m),  ζ = dx / (2 √(m kₓ)) |

Values for kₓ can be determined by choosing a characteristic mass m
(which may be directionally dependent) and then choosing ωₙ > 0
(speed of response). Rearranging ωₙ = √(kₓ/m) produces kₓ = m ωₙ².
One way to choose ωₙ is to choose a settling time tₛ which
approximates the desired time for stretch x to settle to within 1% (0.01)
of an equilibrium solution, and choose a damping ratio ζ (e.g., ζ = 1,
critical damping), then calculate ωₙ = -ln(0.01) / (ζ tₛ) ≈ 4.6 / (ζ tₛ).
For the included example code, a characteristic mass m = 20 kg was chosen
with tₛ = 0.12 and ζ = 1 (critical damping). Thus
ωₙ = -ln(0.01) / 0.12 ≈ 38.38 and kₓ = (20)*(38.38)² ≈ 30000.

### Estimate force damping [dx dy dz] from mass and stiffness 
Once m and kₓ have been chosen, damping dₓ can be estimated by picking a
damping ratio ζ (e.g., ζ ≈ 1, critical damping), then dₓ ≈ 2 ζ √(m kx).
For our example dₓ ≈ 2·√(20·30000) ≈ 1500.

### Estimating torque stiffness [k₀ k₁ k₂] and damping [d₀ d₁ d₂]
The bushing in this planar example replaces a revolute joint. The links are
constrained to planar motion by the existing joints, hence no
torque stiffness nor torque damping is needed.  An alternative way to
deal with this four-bar's closed kinematic loop is to "cut" one of the
four-bar's rigid links in half and join those halves with a bushing
that has both force and torque stiffness/damping.  If this technique
is used, torque stiffness is needed.  One way to approximate torque
stiffness is with concepts similar to the force stiffness above.
For example, the bushing's torque stiffness k₀ could be calculated
by specifying a maximum bushing angular displacement θₘₐₓ, estimating
a maximum moment load Mx and calculating k₀ = Mx / θₘₐₓ.
Alternatively, a value for k₀ can be determined by choosing a
characteristic moment of inertia I₀ (which is directionally dependent)
and then choosing ωₙ (e.g., from setting time), then using k₀ ≈ I₀ ωₙ².
With k₀ available and a damping ratio ζ chosen, d₀ ≈ 2 ζ √(I₀ k₀).
