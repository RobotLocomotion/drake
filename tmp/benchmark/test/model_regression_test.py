"""
Provides regression testing of models.

To update (overwrite) existing regression tests:

    ./run //common:model_regression_test --regenerate

"""

import os
import sys
import unittest

from pydrake.systems.analysis import Simulator
from model_benchmark import (
    make_plant, Expression, Benchmark, create_benchmark, Comparison, FrameNameMismatch)


class Check(object):
    def __init__(self, file, expr):
        self.file = file
        self.expr = expr


iiwa14_no_collision = "drake/manipulation/models/iiwa_description/sdf/iiwa14_no_collision.sdf"


checks = {
    "MyModel: Regression for SDF changes": Check(
        file="tmp/benchmark/data/iiwa14_no_collision.sdf.yaml",
        expr=Expression("make_plant(iiwa14_no_collision)", __name__),
    ),
}


def regenerate():
    if not os.path.exists("WORKSPACE"):
        raise RuntimeError("Run from project root directory only")
    for check in checks.values():
        new = create_benchmark(check.expr)
        new.save_yaml(check.file)


class TestModelRegression(unittest.TestCase):
    def test_mbp_vs_saved(self):
        for note, check in checks.items():
            old = Benchmark.load_yaml(check.file)
            new = create_benchmark(check.expr, old)
            cmp_ = Comparison(tol=2e-5)  # Yuck!
            new.compare(old, cmp_, all_configurations=True)
            self.assertEqual(len(cmp_.errors), 0, note)


if __name__ == "__main__":
    if "--regenerate" in sys.argv:
        regenerate()
    else:
        sys.stdout = sys.stderr
        unittest.main()
