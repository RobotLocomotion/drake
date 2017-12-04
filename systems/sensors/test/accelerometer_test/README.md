To run a demo of an accelerometer attached to a pendulum:

# When building Drake using CMake

    cd drake-distro
    ./build/install/bin/drake-visualizer &
    ./build/drake/systems/sensors/test/accelerometer_test/accelerometer_example --initial_q=1.57 --initial_v=0

# When building Drake using Bazel

    cd drake-distro
    bazel build //tools:drake_visualizer
    bazel-bin/tools/drake_visualizer &
    bazel run -- //drake/systems/sensors:accelerometer_example --initial_q=1.57 --initial_v=0
