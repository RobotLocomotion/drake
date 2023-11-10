"""
$ bazel build //tmp:repro

$ bazel-bin/tmp/repro
Time / iter: 0.2897 ms

$ bazel-bin/tmp/repro --disable_scipy
Time / iter: 0.0267 ms
"""

import argparse
import time


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--disable_scipy", action="store_true")
    args = parser.parse_args()

    if args.disable_scipy:
        import sys
        sys.modules["scipy"] = None

    import numpy as np
    from numpy import array

    from pydrake.solvers import MathematicalProgram

    prog = MathematicalProgram()

    vd_vars = np.concatenate([
        prog.NewContinuousVariables(6, "spatial.scale"),
        prog.NewContinuousVariables(7, "joint.scale"),
    ])
    # ^ this is np.array(dtype=object) of Variable

    Avd = array([[ 2.65718997e+01, -5.60733765e-14,  1.79933696e-16,
          2.93309075e-04, -7.32090388e-11, -3.97322068e-02,
          0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
          0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
          0.00000000e+00],
        [-7.14810902e-01, -4.20804285e-12,  5.53401959e-19,
          -1.61281948e-01, -7.49863660e-13,  5.93973871e+00,
          0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
          0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
          0.00000000e+00],
        [-2.07657434e+01,  1.63474531e-13, -7.11641900e-16,
          -1.18829747e-03, -5.34017976e-11,  1.36295039e-01,
          0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
          0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
          0.00000000e+00],
        [ 4.78482153e-01, -4.82733413e-11,  5.48477048e-19,
          -1.81288172e-01, -8.50964416e-13, -1.46958506e+01,
          0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
          0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
          0.00000000e+00],
        [-1.40378837e+02, -1.92382976e-15,  5.10901382e-16,
          5.75425272e-04,  3.53620989e-11, -3.91765323e-02,
          0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
          0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
          0.00000000e+00],
        [-1.22612281e+00,  5.72600659e-10,  2.38563650e-19,
          -6.79529439e-04,  9.61744602e-14,  1.98614632e+01,
          0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
          0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
          0.00000000e+00],
        [ 1.17683501e+01,  7.21314493e-14,  9.65329560e-15,
          -5.49135713e-04, -1.09862856e-10,  5.68293103e-02,
          0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
          0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
          0.00000000e+00]])
    bvd = array([0., 0., 0., 0., 0., 0., 0.])
    b_v_min = array([-51.65625, -51.65625, -51.65625, -51.65625, -61.9875 , -61.9875 ,
        -61.9875 ])
    b_v_max = array([-51.65625, -51.65625, -51.65625, -51.65625, -61.9875 , -61.9875 ,
        -61.9875 ])

    count = 100
    t_start = time.time()
    for i in range(count):
        prog.AddLinearConstraint(
            Avd,
            b_v_min - bvd,
            -b_v_max - bvd,
            vd_vars,
        )
    dt_mean = (time.time() - t_start) / count
    print(f"Time / iter: {dt_mean * 1000:.4f} ms")


if __name__ == "__main__":
    main()
