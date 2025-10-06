from pathlib import Path
from typing import Iterator
import unittest

from python import runfiles

from pydrake.common.test_utilities.meta import (
    ValueParameterizedTest,
    run_with_multiple_values,
)
from pydrake.planning import RobotDiagramBuilder


def _runfiles_inventory() -> Iterator[tuple[str, Path]]:
    """Generates (resource_path, filesystem_path) tuples for all runfiles in
    @drake_models. For example, resource_path="drake_models/veggies/pepper.png"
    and filesystem_path="/dir/to/foo.runfiles/drake_models/veggies/pepper.png".
    """
    manifest = runfiles.Create()
    inventory = Path(
        manifest.Rlocation("drake/tools/workspace/drake_models/inventory.txt")
    )
    repo_name = "+drake_dep_repositories+drake_models/"
    for line in inventory.read_text(encoding="utf-8").splitlines():
        assert line.startswith(repo_name), line
        filename = line[len(repo_name) :].strip()
        resource_path = f"drake_models/{filename}"
        filesystem_path = Path(manifest.Rlocation(resource_path))
        assert filesystem_path.exists(), filesystem_path
        yield resource_path, filesystem_path


def _models_inventory() -> Iterator[str]:
    """Generates package URLs for all top-level model files in @drake_models.
    For example, url="package://drake_models/veggies/pepper.sdf".
    """
    for resource_path, filesystem_path in _runfiles_inventory():
        url = f"package://{resource_path}"
        if (
            resource_path.endswith(".dmd.yaml")
            or resource_path.endswith(".sdf")
            or resource_path.endswith(".urdf")
        ):
            yield url
        elif resource_path.endswith(".xml"):
            # We can't tell a MuJoCo file just by its suffix.
            with filesystem_path.open(encoding="utf-8") as xml:
                if "<mujoco" in xml.read(1024):
                    yield url


def _get_all_test_cases() -> Iterator[dict]:
    """Generates test case specifications, each one a dictionary consisting of:
    - url: str
    - expect_warnings: bool
    - expect_errors: bool
    The test specifications are is used by TestDrakeModels.test_model below to
    run Drake's parser against all models in @drake_models.
    """
    # Package URLs for all top-level model files.
    all_models = set(_models_inventory())

    # Allow warnings on these models until they are repaired.
    models_with_warnings = {
        # TODO(#19992) for tracking these warnings.
        "package://drake_models/atlas/atlas_convex_hull.urdf",
        "package://drake_models/atlas/atlas_minimal_contact.urdf",
        # We don't have any tracking issue for fixing the allegro models,
        # because we don't use them for anything we care about. If someone
        # wants to fix the warnings, be our guest.
        "package://drake_models/allegro_hand_description/urdf/allegro_hand_description_left.urdf",  # noqa
        "package://drake_models/allegro_hand_description/urdf/allegro_hand_description_right.urdf",  # noqa
        # TODO(jwnimmer-tri) Fix these warnings.
        "package://drake_models/jaco_description/urdf/j2n6s300_col.urdf",
    }
    stragglers = models_with_warnings - all_models
    assert not stragglers, repr(stragglers)

    # Allow errors on these models until they are repaired.
    models_with_errors = {
        # This file is not designed to be loaded independently from its
        # parent file `homecart.dmd.yaml`; it will always error out.
        "package://drake_models/tri_homecart/homecart_grippers.dmd.yaml",
    }
    stragglers = models_with_errors - all_models
    assert not stragglers, repr(stragglers)
    overlap = models_with_warnings & models_with_errors
    assert not overlap, repr(overlap)

    for url in all_models:
        yield dict(
            url=url,
            expect_warnings=(url in models_with_warnings),
            expect_errors=(url in models_with_errors),
        )


class TestDrakeModels(unittest.TestCase, metaclass=ValueParameterizedTest):
    @run_with_multiple_values(_get_all_test_cases())
    def test_model(
        self, *, url: str, expect_warnings: bool, expect_errors: bool
    ):
        """Checks that when parsed, the given package url contains warnings
        and/or errors consistent with the given conditions. This test case is
        run repeatedly (via 'run_with_multiple_values') in order to cover all
        models in @drake_models.
        """
        if expect_errors:
            with self.assertRaises(Exception):
                self._parse(url=url, strict=False)
            return
        if expect_warnings:
            self._parse(url=url, strict=False)
            with self.assertRaises(Exception):
                self._parse(url=url, strict=True)
            return
        self._parse(url=url, strict=True)

    def _parse(self, *, url: str, strict: bool):
        builder = RobotDiagramBuilder()
        parser = builder.parser()
        if strict:
            parser.SetStrictParsing()
        parser.AddModels(url=url)
