To run the single_car_in_stata_garage unit test:

   $ rostest drake_cars_examples single_car_in_stata_garage_test.test

The above will store output in a log file and use a roscore on a random port
making it difficult to debug.

To debug the unit test:

  $ roscore
  $ rostest drake_cars_examples single_car_in_stata_garage_test.test --text --reuse-master
