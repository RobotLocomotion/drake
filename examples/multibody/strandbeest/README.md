# Strandbeest

## Change the model
Modfiy any relevant values in `model/Macros.xacro`. The Strandbeest model is 
a dependancy of the executable, and `xacro` will be run automatically.

## Run the example
```
bazel run --config snopt //examples/multibody/strandbeest:run_with_motor
```
