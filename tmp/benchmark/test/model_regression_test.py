"""
Provides regression testing of models.

To update (overwrite) existing regression tests:

    cd drake
    bazel build //tmp/benchmark:py/model_regression_test
    ./bazel-bin/tmp/benchmark/py/model_regression_test --regenerate

"""

import os
from os.path import dirname, isfile, join, realpath
import subprocess
import sys
import unittest

from drake import __file__ as root_module_file
from pydrake.common import FindResourceOrThrow
from model_benchmark import (
    make_plant, Expression, Benchmark, create_benchmark, Comparison, FrameNameMismatch)


def drake_source_tree_root():
    return dirname(realpath(root_module_file))


def try_find_file(sdf_file):
    """
    Try first to find with Bazel (so we get neighboring generated files, etc.).
    If that fails, return path of file in source tree.
    """
    try:
        return FindResourceOrThrow(f"drake/{sdf_file}")
    except RuntimeError as e:
        # TODO(eric.cousineau): This shouldn't really be here. Ideally, all
        # files could be found via Bazel.
        assert ".runfiles" in str(e), e
        sdf_file_source = join(drake_source_tree_root(), sdf_file)
        assert isfile(sdf_file_source), sdf_file_source
        return sdf_file_source


class Check(object):
    def __init__(self, sdf_file):
        self.sdf_file = sdf_file
        self.yaml_file = join("tmp/benchmark/data", f"{sdf_file}.yaml")
        self.expr = Expression(
            f"make_plant(try_find_file({repr(sdf_file)}), find_resource=False)",
            __name__,
        )


# File: Error text substring
KNOWN_REQUIRED_EXCEPTIONS = {
    "examples/atlas/sdf/block_angle_steps_1/model.sdf": (
        "Error: Unable to find uri[model://cinder_block_wide]"
    ),
    "examples/atlas/sdf/block_angle_steps_2/model.sdf": (
        "Error: Unable to find uri[model://cinder_block_wide]"
    ),
    "examples/atlas/sdf/block_level_steps_1/model.sdf": (
        "Error: Unable to find uri[model://cinder_block_wide]"
    ),
    "examples/atlas/sdf/block_level_steps_2/model.sdf": (
        "Error: Unable to find uri[model://cinder_block_wide]"
    ),
    "examples/atlas/sdf/sun/model.sdf": (
        "File must have a single <model> element."
    ),
    "examples/simple_four_bar/FourBar.sdf": (
        "This mobilizer is creating a closed loop since"
    ),
    "multibody/parsing/test/box_package/sdfs/box.sdf": (
        # Needs to be added as a data deps.
        "provided uri \"package://box_model/meshes/box.obj\""
    ),
    "multibody/parsing/test/sdf_parser_test/include_models.sdf": (
        # Needs to be added as a data deps.
        "Error: Unable to find uri[model://simple_robot1]"
    ),
    "multibody/parsing/test/sdf_parser_test/joint_parsing_test.sdf": (
        # Needs to be supported in configuration stuff (BallRpyJoin).
        "BallRpyJoint_"
    ),
    "multibody/parsing/test/sdf_parser_test/negative_damping_joint.sdf": (
        # Negative test case.
        "Joint damping is negative for joint 'joint'"
    ),
    "multibody/parsing/test/sdf_parser_test/two_models.sdf": (
        # Negative test case.
        "File must have a single <model> element."
    ),
}

# TODO(eric): How to easily check merge-base vs. feature branch? This is kinda
# messy.
KNOWN_POSSIBLE_EXCEPTIONS = {
    "multibody/parsing/test/sdf_parser_test/world_with_directly_nested_models.sdf": (
        # Negative test case.
        "File must have a single <model> element."
    ),
    "multibody/parsing/test/sdf_parser_test/model_with_directly_nested_models.sdf": {
        "Newly added"
    },
}


def list_checks(*, use_source_tree=True):
    # Find available data from disk / Bazel target.
    if use_source_tree:
        drake_dir = drake_source_tree_root()
    else:
        # Use Bazel declared stuff.
        drake_dir = dirname(FindResourceOrThrow("drake/.drake-find_resource-sentinel"))
    files_raw = subprocess.run(
        ["find", ".", "-name", "*.sdf"],
        cwd=drake_dir,
        encoding="utf8",
        stdout=subprocess.PIPE,
        check=True,
    ).stdout.strip().split("\n")

    files = []
    for file_raw in files_raw:
        if file_raw.startswith("./external/"):
            continue
        elif file_raw.startswith("./"):
            file = file_raw[2:]
            files.append(file)
        else:
            assert False, file_raw
    files.sort()
    # files = ["multibody/parsing/test/sdf_parser_test/model_with_directly_nested_models.sdf"]
    return [Check(f) for f in files]


def regenerate():
    if not os.path.exists("WORKSPACE"):
        raise RuntimeError("Run from project root directory only")
    new_error_messages = []
    new_error_files = []
    known_error_files = set()
    known_option_error_files = set()

    for check in list_checks():
        print()
        print(f"Generate: {check.sdf_file}")
        try:
            new = create_benchmark(check.expr)
            os.makedirs(dirname(check.yaml_file), exist_ok=True)
            new.save_yaml(check.yaml_file)
        except (RuntimeError, AssertionError) as e:
            expected_substr = KNOWN_REQUIRED_EXCEPTIONS.get(check.sdf_file)
            possible_substr = KNOWN_POSSIBLE_EXCEPTIONS.get(check.sdf_file)
            if expected_substr is not None and expected_substr in str(e):
                print(f"  Known error - ignoring: {expected_substr}")
                known_error_files.add(check.sdf_file)
            elif possible_substr is not None and possible_substr in str(e):
                print(f"  Known optional error - ignoring: {possible_substr}")
                known_option_error_files.add(check.sdf_file)
            else:
                new_error_files.append(check.sdf_file)
                new_error_messages.append(
                    f"{check.sdf_file}:\n{str(e)}"
                )

    # Freak out if there are new exceptions.
    if len(new_error_messages) > 0:
        error_file = "/tmp/model_regression_test_errors.txt"
        with open(error_file, "w") as f:
            f.write("\n\n".join(new_error_messages))
        raise RuntimeError(
            f"Unexpected errors, more info in {error_file}:\n" +
            "\n".join(sorted(new_error_files))
        )

    # Freak out if exceptions change.
    expected_error_files = set(KNOWN_REQUIRED_EXCEPTIONS.keys())
    expected_not_found = expected_error_files - known_error_files
    unexpected = known_error_files - expected_error_files
    # Should've been caught above.
    assert len(unexpected) == 0, unexpected
    if len(expected_not_found) > 0:
        raise RuntimeError(
            f"Expected errors, but no longer has errors:\n" +
            "\n".join(sorted(expected_not_found)))

    print()
    print(f"All regression files generated.")
    print(f"  Expected (required) unparseable files: {len(KNOWN_REQUIRED_EXCEPTIONS)}")
    print(f"  Expected (optional) unparseable files: {len(known_option_error_files)}")
    print()


class TestModelRegression(unittest.TestCase):
    def test_mbp_vs_saved(self):
        missing = []
        actual_skipped = set()
        for check in list_checks():
            print(f"Check: {check.yaml_file}")
            if not os.path.exists(check.yaml_file):
                if check.sdf_file in KNOWN_REQUIRED_EXCEPTIONS:
                    actual_skipped.add(check.sdf_file)
                elif check.sdf_file in KNOWN_POSSIBLE_EXCEPTIONS:
                    pass
                else:
                    missing.append(check.yaml_file)
                print("  Skipping")
                continue
            old = Benchmark.load_yaml(check.yaml_file)
            new = create_benchmark(check.expr, old)
            cmp_ = Comparison(5e-6)  # Yuck!
            new.compare(old, cmp_, all_configurations=True)
            self.assertEqual(len(cmp_.errors), 0, check.sdf_file)
        self.assertEqual(len(missing), 0, "\n".join(missing))
        expected_skipped = set(KNOWN_REQUIRED_EXCEPTIONS.keys())
        self.assertEqual(actual_skipped, expected_skipped)


def main():
    if "--regenerate" in sys.argv:
        regenerate()
    else:
        sys.stdout = sys.stderr
        unittest.main()


if __name__ == "__main__":
    main()
