This directory contains unit tests for the code in ROS package
`drake_ros_common`. It is configured as a separate package to test whether
`drake_ros_common` is correctly exporting its libraries and header files.

To run the unit tests, first install and build Drake by following
[these instructions](http://drake.mit.edu/from_source_ros.html). Then execute:

    $ rostest drake_ros_common_test parameter_server_test.test
