This directory contains unit tests for the code in ROS package
`drake_ros_common`. To run the unit tests, first install and build Drake by
following [these instructions](http://drake.mit.edu/from_source_ros.html). Then
execute:

    $ rostest drake_ros_common_test parameter_server_test.test
    $ rostest drake_ros_common_test simulation_abort_function_test.test
