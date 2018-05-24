#!/usr/bin/env bash

# Case 1, gripper_force=0:
# With implicit integrator and the parameters chosen and
# v_stiction_tolerance=1e-5, this simulation runs at 26X.

# Case 2, gripper_force = 10 N (~ 1 kg):
# With implicit integrator and the parameters chosen and
# v_stiction_tolerance=2e-4 this simulation runs at X / 1.5 (50% slower than
# real time).
# Number of time steps and consequently the simulation wall time roughly scale
# with the inverse of v_stiction_tolerance. That is, to reduce the observed
# sliding to a fifth will imply a simulation that runs five times slower than
# with the current configuration.

# We need to get into the .runfiles directory in order for sdformat to find root.sdf. See #7874.
cd bazel-bin/examples/simple_gripper/simple_gripper.runfiles/drake

./examples/simple_gripper/simple_gripper \
    --simulation_time=10 --target_realtime_rate=0.0 \
    --integration_scheme=implicit_euler --max_time_step=1e-2 --accuracy=1e-2 \
    --gripper_force=10.0 \
    --amplitude=0.05 --frequency=1.0 \
    --ring_static_friction=1.0 --ring_dynamic_friction=0.5 \
    --penetration_allowance=0.01 --v_stiction_tolerance=1e-2 \
    --ring_samples=8 --ring_orient=0 \
    --grip_width=0.095 \
    --rx=0 --ry=0 --rz=0

# Note on grip width:
# With --finger_width=0.09415, the gripper just barely touches the mug and it starts falling

cd -
