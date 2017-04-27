To load these models into Gazebo, first update environment variable
`GAZEBO_MODEL_PATH` by executing the following command:

    export GAZEBO_MODEL_PATH=`pwd`:$GAZEBO_MODEL_PATH

Launch `gazebo`, then delete the ground plane and pause the simulation. This is
necessary since the objective is to evaluate the model's frames and not all of
its kinematics and dynamics. Switch to the "Insert" tab on the left hand side of
Gazebo's GUI and import the desired model by clicking and dragging it into the
simulation.
