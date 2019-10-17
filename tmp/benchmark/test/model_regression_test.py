"""
Provides regression testing of models.

To update (overwrite) existing regression tests:

    ./run //common:model_regression_test --regenerate

"""

import os
from os.path import dirname, join
import subprocess
import sys
import unittest

from pydrake.common import FindResourceOrThrow
from pydrake.systems.analysis import Simulator
from model_benchmark import (
    make_plant, Expression, Benchmark, create_benchmark, Comparison, FrameNameMismatch)


class Check(object):
    def __init__(self, sdf_file):
        self.sdf_file = sdf_file
        self.yaml_file = join("tmp/benchmark/data", f"{sdf_file}.yaml")
        sdf_file_drake = join("drake", sdf_file)
        self.expr = expr = Expression(f"make_plant('{sdf_file_drake}')", __name__)

# iiwa14_no_collision = "drake/manipulation/models/iiwa_description/sdf/iiwa14_no_collision.sdf"

def list_checks():
    drake_dir = dirname(FindResourceOrThrow("drake/.drake-find_resource-sentinel"))
    files = subprocess.check_output(
        ["find", ".", "-name", "*.sdf"], cwd=drake_dir).strip().decode("utf8").split("\n")
    for file in list(files):
        if file.startswith("./external/"):
            files.remove(file)
        elif file.startswith("./"):
            files.remove(file)
            files.append(file[2:])
    files.sort()
    return [Check(f) for f in files]


def regenerate():
    if not os.path.exists("WORKSPACE"):
        raise RuntimeError("Run from project root directory only")
    for check in list_checks():
        print(f"Generate: {check.sdf_file}")
        new = create_benchmark(check.expr)
        os.makedirs(dirname(check.yaml_file), exist_ok=True)
        new.save_yaml(check.yaml_file)


class TestModelRegression(unittest.TestCase):
    def test_mbp_vs_saved(self):
        missing = []
        for check in list_checks():
            print(f"Check: {check.yaml_file}")
            if not os.path.exists(check.yaml_file):
                missing.append(check.yaml_file)
                continue
            old = Benchmark.load_yaml(check.yaml_file)
            new = create_benchmark(check.expr, old)
            cmp_ = Comparison(5e-6)  # Yuck!
            new.compare(old, cmp_, all_configurations=True)
            self.assertEqual(len(cmp_.errors), 0, check.sdf_file)
        self.assertEqual(len(missing), 0, "\n".join(missing))


if __name__ == "__main__":
    if "--regenerate" in sys.argv:
        regenerate()
    else:
        sys.stdout = sys.stderr
        unittest.main()
