# Strandbeest
This example runs a simulation of Theo Jansen's 
[Strandbeest](https://www.strandbeest.com/) mechanism. 

## Change the model
Modify any relevant values in `model/Macros.xacro`. The Strandbeest model is 
a dependency of the executable, and `xacro` will be run automatically.

## Run the example
Run the example with the default available solver:
```
bazel run //examples/multibody/strandbeest:run_with_motor
```
Run the example with the [SNOPT](https://drake.mit.edu/bazel.html#snopt) 
solver for faster IK solves:
```
bazel run --config snopt //examples/multibody/strandbeest:run_with_motor
```
