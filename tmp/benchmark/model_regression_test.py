"""
Provides regression testing of models.

To update (overwrite) existing regression tests:

    ./run //common:model_regression_test --regenerate

"""

import os
import sys
import unittest

from pydrake.systems.analysis import Simulator
from .model_benchmark import (
    Benchmark, create_benchmark, Comparison, FrameNameMismatch)


class Check(object):
    def __init__(self, file, expr):
        self.file = file
        self.expr = expr


checks = {
    "MyModel: Regression for SDF changes": Check(
        file="stuff.yaml",
        expr="load_model(biscuit)",  # noqa
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
            cmp_ = new.compare(old)
            self.assertEqual(len(cmp_.errors), 0, note)


if __name__ == "__main__":
    if "--regenerate" in sys.argv:
        regenerate()
    else:
        sys.stdout = sys.stderr
        unittest.main()
