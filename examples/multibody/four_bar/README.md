# Four-Bar Linkage Example
> A four-bar linkage example that demonstrates the use of a linear bushing as
> a way to model a kinematic loop. The file
> `FourBarLinkageValentineHeartStaticsDynamics.pdf` contains a problem
> statement for a four bar linkage which this example shows how to model with
> Drake's MultibodyPlant. This example demonstrates loading a single SDF model
> into a generic `MultiBodyPlant`. The model is a simple four bar linkage
>. This example also demonstrates modeling a closed loop kinematic chain by
> replacing one of the revolute joints in the four bar with a
> `LinearBushingRollPitchYaw` `ForceElement`.

## Four-bar linkage statics

The figure below shows a planar four-bar linkage consisting of frictionless
-pin-connected uniform rigic links *A, B, C* and ground-link *W*.
- Link *A* connects to *W* and *B* at points *A*ₒ and *B*ₒ
- Link *B* connects to *A* and *C* at points *B*ₒ and *B*c
- Link *C* connects to *W* and *B* at points *C*ₒ and *C*ʙ

Right-handed orthogonal unit vectors **âᵢ, b̂ᵢ, ĉᵢ, ŵᵢ** 
(*i = x,y,z*) are fixed in *A, B, C, W,* with:
- **â**𝐱 directed from *A*ₒ to *B*ₒ
- **b̂**𝐱 directed from *B*ₒ to *B*c
- **ĉ**𝐱 directed from *C*ₒ to *C*ʙ
- **ŵ**𝐳 vertically-upward.
- **â**𝐲 = **b̂**𝐲 = **ĉ**𝐲 = **ŵ**𝐲 parallel to pin axes

| Diagram of the four bar model described above. |
| :---: |
| ![FourBarLinkageSchematic](images/FourBarLinkageSchematic.png)    |
|  |

|                     Quantity                     |        Symbol       |   Value   |
| ------------------------------------------------ | ------------------- | --------- |
| Distance between *W*ₒ and *C*ₒ                   |          𝐋ᴡ        |    2 m    |
| Lengths of links *A, B, C*                       |         *L*         |    4 m    |
| Masses of *A, B, C*                              |         *m*         |   20 kg   |
| Earth’s gravitational acceleration               |         *g*         | 9.8 m/s²  |
|                                                  |                     |           |
| **ŵ**𝐲 measure of motor torque on *A*            |          𝐓ᴀ         | Specified |
| Angle from **ŵ**𝐱 to **â**𝐱 with a +**ŵ**𝐲 sense |          𝐪ᴀ         |  Variable |
| Angle from **â**𝐱 to **b̂**𝐱 with a +**â**𝐲       |          𝐪ʙ         |  Variable |
| Angle from **ŵ**𝐱 to **ĉ**𝐱 with a +**ŵ**𝐲 sense |          𝐪ᴄ         | Variable  |
|                                                  |                     |           |
| "Coupler-point" *P*'s position from *B*ₒ         |         2 **b̂**𝐱 - 2 **b̂**𝐳     |

With 𝐓ᴀ = 0, the equilibrium values for the angles are: 
𝐪ᴀ ≈ 104.48°, 𝐪ʙ ≈ 75.52°, 𝐪ᴄ ≈ 75.52°.

## Starting Configuration

> The SDF defines all of the links with their x axes parallel to the world x
> axis for convenience of measuring the angles in the state of the system
> with respect to a fixed axis. Below we derive a valid initial configuration
> of the three angles `q_A`, `q_B`, and `q_C`.

| Derivation of starting configuration |
| :---: |
| ![FourBarLinkageSchematic](images/FourBarLinkageGeometry.png)    |
| |

Because of the congruity of the link lengths, this initial condition (static
equilibrium) is very symmetric, forms an isosceles trapezoid, and can be
solved just with trigonometry. `q_A` is one angle of a right triangle with
its adjacent side measuring 1m and hypotenuse measuring 4m. We
can immediately calculate `q_A` as `atan2(sqrt(15), 1)`.

Because link B is parallel with the `W_x` axis, `q_A` and `q_B` are
 supplementary, and thus we can set `q_B = M_PI - q_A`.

For a similar symmetric argument, the same triangle is formed on the right
 side of the trapezoid. Making `q_A` and `q_C` supplementary and thus `q_C
  = M_PI - q_A`.

## Modelling a revolute joint

In this example, we remove the joint at point **Bc** in the diagram that
would usually connect link **B** to link **C**. To accomplish this we use a
`LinearBushingRollPitchYaw` `ForceElement` placed at the joint. One of the
many uses of this bushing element is to model a revolute joint. We model a z
-axis revolute joint by setting the torque stiffness (k₂) and torque damping
(d₂) constants to 0. We choose the z axis (Yaw) to avoid gimbal lock. Two
frames (one attached to **B** called `BC_Bushing` with origin at point 
**Bc** and one attached to **C** called `CB_Bushing` with origin at point 
**Cb**) are placed such that their z axes point out of the paper in the above
 diagram.