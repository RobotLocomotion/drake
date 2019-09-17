"""Provides simple testing of `model_benchmark` features."""

from os import environ
from os.path import join
import re
from subprocess import check_call
import sys
import unittest

import numpy as np

from pydrake.common.eigen_geometry import Isometry3, AngleAxis
from .model_benchmark import (
    Benchmark, SemanticQ, create_benchmark,
    KinematicError, PositionValueMismatch, PositionNameMismatch)

simple_expr = "simple_mbp()"


def corrupt_frame(frame):
    X_err = Isometry3(
        rotation=AngleAxis(np.pi / 2, [1, 0, 0]).rotation(),
        translation=[0, 0, 0])
    frame.X_WF = frame.X_WF.multiply(X_err)


def change_q_order(b):
    for config in b.configurations:
        q_old = config.q
        names = list(q_old.names)
        names[0], names[-1] = names[-1], names[0]
        config.q = SemanticQ(names, q_old.get_values(names))


class TestModelBenchmark(unittest.TestCase):
    """Tests benchmark code and some simple stuff."""
    def check_error(self, err, path, type_):
        self.assertIsInstance(err, type_)
        self.assertEqual(err.path, path)

    def assert_good(self, cmp_):
        self.assertEqual(len(cmp_.errors), 0)

    def test_idempotent(self):
        # Idempotent test.
        baseline = create_benchmark(expr=simple_expr)
        self.assert_good(baseline.compare(baseline))
        # - Recreate.
        recreate = create_benchmark(expr=simple_expr)
        self.assert_good(baseline.compare(recreate))
        # - Recreate for comparison.
        recomp = create_benchmark(simple_expr, baseline)
        self.assert_good(baseline.compare(recomp))
        # - Direct.
        saved_file = join(environ["TEST_TMPDIR"], "saved.yaml")
        baseline.save_yaml(saved_file)
        saved = Benchmark.load_yaml(saved_file)
        self.assert_good(baseline.compare(saved))
        # - Commandline.
        cmdline_file = join(environ["TEST_TMPDIR"], "cmdline.yaml")
        check_call([
            script, "create", "--expr", simple_expr,
            "--output", cmdline_file])
        cmdline = Benchmark.load_yaml(cmdline_file)
        self.assert_good(baseline.compare(cmdline))

    def test_corruption(self):
        baseline = create_benchmark(expr=simple_expr)
        # Change q order.
        reorder = create_benchmark(expr=simple_expr)
        change_q_order(reorder)
        self.assertNotEqual(
            baseline.configurations[0].q.names,
            reorder.configurations[0].q.names)
        error, = baseline.compare(reorder).errors
        self.check_error(
            error, "/configuration[@note='Zeros']/q",
            PositionNameMismatch)
        # Corruption test
        bad = create_benchmark(expr=simple_expr)
        # - Configuration
        q_old = bad.configurations[0].q
        values_bad = q_old.get_values(None)
        values_bad[0] = 10
        bad.configurations[0].q = SemanticQ(q_old.names, values_bad)
        errors = baseline.compare(bad).errors
        self.assertEqual(len(errors), 1)
        self.check_error(
            errors[0], "/configuration[@note='Zeros']/q",
            PositionValueMismatch)
        # - Rotation of first config's first frame.
        # - - Ignored since config is bad.
        bad = create_benchmark(expr=simple_expr)
        corrupt_frame(bad.configurations[0].frames[0])
        corrupt_frame(bad.configurations[1].frames[0])
        errors = baseline.compare(bad).errors
        self.assertEqual(len(errors), 1)
        self.check_error(
            errors[0],
            "/configuration[@note='Zeros']/frame[@name='WorldBody']",
            KinematicError)


if __name__ == "__main__":
    sys.stdout = sys.stderr
    unittest.main()
