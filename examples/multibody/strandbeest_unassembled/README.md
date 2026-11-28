# Unassembled Strandbeest

This example simulates Theo Jansen's
[Strandbeest](https://www.strandbeest.com/), a walking machine whose legs are
linkages full of kinematic loops. The model files describe the machine as
*unassembled* parts, straight bars and rigid triangles joined by pins, with
each part laid out along a coordinate axis, so that describing it takes no
trigonometry. Drake does the rest:

* At `Finalize()`, MultibodyPlant finds the kinematic loops that the pins form,
  three per leg, and breaks each one by splitting one of its parts into two
  links, the part and a "shadow" of it, joined by a weld constraint.
* `AssembleNearestConfiguration()`, in `assemble_nearest_configuration.h`,
  assembles the legs by optimization: it finds the configuration nearest the
  unassembled one that satisfies every weld. Nothing in it is specific to this
  example, so you can copy it for your own models.
* Motors that the program adds turn each leg's crank to its phase on the
  crankshaft, in simulation, and the cranks are locked there.
* Finally the crankshaft turns, and the legs walk the machine along the ground
  if there are enough of them to hold it up.

The bar lengths are Jansen's "holy numbers." See
[Jansen's linkage](https://en.wikipedia.org/wiki/Jansen%27s_linkage) for the
usual drawing and the letters a to m that name the bars; the model files use
the same letters.

## Running the example

To run with default flags:

```
bazel run //examples/multibody/strandbeest_unassembled:strandbeest_unassembled
```

You'll see output like this:

```
[console] [info] Meshcat listening for connections at http://localhost:7000
Modeled 36 kinematic loop(s) with 36 constraint(s).
Loop closure error as defined: 0.5984 m

The legs are unassembled. Drake broke each kinematic loop by
...
Press Enter to assemble the legs . . .
```

Open the URL it prints in a browser to watch. The program stops three times,
waiting for you to press Enter each time:

1. With the legs as the model file describes them, unassembled: every leg is
   drawn the same way (or as its mirror image), with its crank lined up with the
   others, and some parts' holes are away from the pins they belong on. The
   shadow links are not drawn.
2. Once the legs are assembled, with all of their cranks lined up. The legs
   take three seconds to move from where they were drawn to where they are
   assembled, so that you can watch it happen.
3. Once each crank has been turned to its phase on the crankshaft and locked
   there.

Then the crankshaft turns for ten seconds of simulated time, and the program
reports how far the machine walked:

```
Press Enter to turn the crankshaft and walk . . .
Loop closure error after simulating 10 s: 0.00346 m
The strandbeest walked 3.38 m, and its axle is 0.920 m up.
```

The loop closure error is how far the shadow links are from the parts they
were split from, as the largest distance between where a part and its shadow
put any of the part's holes. It is zero for a perfectly assembled machine.

## Choosing a model

The default model has twelve legs. `--model` picks another one from the
`model` directory:

| `--model=`             | Legs | Kinematic loops | Walks            |
| ---------------------- | ---: | --------------: | ---------------- |
| `JansenLeg`            |    1 |               3 | in the air       |
| `TwoLegs`              |    2 |               6 | in the air       |
| `FourLegs`             |    4 |              12 | in the air       |
| `EightLegs`            |    8 |              24 | along the ground |
| `TwelveLegs` (default) |   12 |              36 | along the ground |
| `SixteenLegs`          |   16 |              48 | along the ground |
| `TwentyLegs`           |   20 |              60 | along the ground |

From four legs up, the legs come in mirror-image pairs, as Jansen builds them.
Models with fewer than eight legs can't hold the machine up, so the program
keeps the chassis locked where it starts, with the feet just clear of the
ground, and the legs walk in the air.

A single leg is the easiest way to see what the assembly does:

```
bazel run //examples/multibody/strandbeest_unassembled:strandbeest_unassembled \
    -- --model=JansenLeg
```

## Other options

To run start to finish without stopping for Enter:

```
bazel run //examples/multibody/strandbeest_unassembled:strandbeest_unassembled \
    -- --nointeractive
```

To walk for longer, and to stretch the assembly out over ten seconds of wall
clock time so that it is easier to follow (or skip it with
`--assembly_playback_time=0`):

```
bazel run //examples/multibody/strandbeest_unassembled:strandbeest_unassembled \
    -- --simulation_time=30 --assembly_playback_time=10
```

To turn the crankshaft twice as fast. The faster the machine moves, the farther
its loops drift apart with the default 50 ms time step, so take smaller steps
too:

```
bazel run //examples/multibody/strandbeest_unassembled:strandbeest_unassembled \
    -- --crank_speed=4 --time_step=0.01
```

The program runs at real time by default, so that the walking looks realistic.
To run as fast as it can instead:

```
bazel run //examples/multibody/strandbeest_unassembled:strandbeest_unassembled \
    -- --simulator_target_realtime_rate=0
```

The example runs a discrete plant, which uses the SAP solver to enforce the
weld constraints. A continuous plant integrated with CENIC can enforce them
too, but on the larger models it is far slower; see
[#25293](https://github.com/RobotLocomotion/drake/issues/25293).

## Getting help

To list this example's own options:

```
bazel run //examples/multibody/strandbeest_unassembled:strandbeest_unassembled \
    -- --helpshort
```

`--help` lists those along with the standard Drake simulator flags, such as
`--simulator_target_realtime_rate`.

## The model files

* `model/JansenParts.xacro` has Jansen's lengths and the macros that build the
  machine from parts: `bar`, `triangle`, and `pin` for the parts and the joints
  between them, `jansen_leg` for one leg, `positioned_leg` and `leg_pair` for
  placing legs on the crankshaft, and `chassis_and_crankshaft` and `ground`.
* Each `model/<Name>.sdf.xacro` is one model, `--model=<Name>`. Each is only a
  few lines long, placing the legs along the crankshaft, each with its phase.

Bazel runs xacro to turn each `<Name>.sdf.xacro` into `<Name>.sdf` when it
builds the example, so after you edit one, just run the example again. To make
a model of your own, add a new `.sdf.xacro` file, add it to the `models` list
in `BUILD.bazel`, and run with `--model` set to its name.

Every pin is a revolute joint that uses Drake's `<drake:independent_frames/>`
SDFormat extension to name the two holes it pins together, one on each part.
Here is a leg's pin `U1` (simplified from what the `pin` macro produces):

```xml
<joint name="leg1_bde_j" type="revolute">
  <drake:independent_frames/>
  <parent>leg1_bde_U1</parent>  <!-- The hole U1 on triangle bde. -->
  <child>leg1_j_U1</child>      <!-- The hole U1 on bar j. -->
  <axis><xyz>0 -1 0</xyz></axis>
</joint>
```

The joint is all that relates the two parts, so each part can be laid out
wherever it is convenient. A standard SDFormat joint has only a single frame,
which forces a model to be described already assembled. See
[the four-bar examples](../four_bar/README.md#describing-an-unassembled-loop-in-sdformat)
for more about the extension.

## How the machine is assembled

A linkage with loops can usually be assembled in more than one way, and
`AssembleNearestConfiguration()` finds the way nearest where it starts. So the
model file draws each leg as it looks with its crank pointing straight away
from it, and the program assembles every leg with its crank there, at zero.
That puts every leg on the intended branch at once.

One drawing fits only one crank angle, though, and each leg's crank has its
own phase, marked on the crankshaft by the frame `<prefix>crank_phase`. So the
program next turns each crank to its phase with a motor, in a simulation in
which the weld constraints keep the legs assembled, and locks it there as if it
were bolted to the crankshaft. That is much faster than reassembling the legs
over and over with the optimizer. Meanwhile, the chassis and the crankshaft
stay locked.

`Run()`, at the top of `strandbeest_unassembled.cc`, tells this story in
order. `assemble_nearest_configuration.h` and `.cc` are the files to copy.
Note that `AssembleNearestConfiguration()` uses
`MultibodyPlant::get_weld_constraint_specs()`, which is marked "(Internal use
only)", because the plant doesn't yet offer a public way to ask for the frames
of its weld constraints.

`test/jansen_leg_assembly_test.cc` checks that the one-leg model assembles the
way Jansen designed it, rather than in one of the other ways its parts can fit
together. Every leg is drawn and assembled the same way (some mirrored), so
that covers them all.

To see the shadow links that Drake adds, and the welds that join them to their
parts, run the simpler
[`four_bar_unassembled`](../four_bar/README.md#watching-a-loop-be-modeled-four_bar_unassembled)
example, which draws them.
