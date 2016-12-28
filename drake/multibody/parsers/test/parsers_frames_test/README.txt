To load these models into Gazebo, first update the `GAZEBO_MODEL_PATH`
environment variable:

    cd drake-distro/drake/multibody/parsers/test/parsers_frames_test
    export GAZEBO_MODEL_PATH=`pwd`:$GAZEBO_MODEL_PATH

Then, launch `gazebo` and then import the models by clicking and dragging on the
model's name into the simulation.
