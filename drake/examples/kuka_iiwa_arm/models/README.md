# How To Load Models into Gazebo

The models in this folder contain `model.config` files that allow you to easily
load the them into Gazebo. This is useful for viewing and debugging purposes. To
do this, first add the directory containing this README to environment variable
`GAZEBO_MODEL_PATH`.

    $ cd drake-distro/drake/examples/kuka_iiwa_arm/models
    $ export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:`pwd`

Next start Gazebo:

    $ gazebo

Finally, in the left panel of Gazebo's GUI, click on the "Insert" tab, then
left-click on the model you want to add, then left-click on the location in the
simulated world where you would like the model to be placed.