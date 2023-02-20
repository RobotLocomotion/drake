# Checking MbP Quaternion + Free Body Dynamics

See <https://github.com/RobotLocomotion/drake/issues/18816>

## Running

```sh
cd drake
bazel build //examples/so3_integ_bug:py/spatial_trajectories_test

bazel-bin/examples/so3_integ_bug/py/spatial_trajectories_test
```

### Testing against MuJoCo

Not really necessary, but it's cute. Set up virtualenv:

```sh
cd drake
./examples/so3_integ_bug/venv.sh

# Then rebuild and run.
```
