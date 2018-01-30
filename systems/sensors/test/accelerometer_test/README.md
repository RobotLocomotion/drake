To run a demo of an accelerometer attached to a pendulum:

# When building Drake using Bazel

    cd drake
    bazel build //tools:drake_visualizer
    bazel-bin/tools/drake_visualizer &
    bazel run -- //systems/sensors:accelerometer_example --initial_q=1.57 --initial_v=0
