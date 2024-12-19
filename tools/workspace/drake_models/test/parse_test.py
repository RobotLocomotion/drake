from pathlib import Path
from typing import Iterator
import unittest

from python import runfiles

from pydrake.common.test_utilities.meta import (
    ValueParameterizedTest,
    run_with_multiple_values,
)
from pydrake.planning import RobotDiagramBuilder


def _inventory() -> Iterator[str]:
    """Generates the list of all the runfiles of @drake_models. The result
    is strings like "drake_models/veggies/pepper.png".
    """
    manifest = runfiles.Create()
    inventory = Path(manifest.Rlocation(
        "drake/tools/workspace/drake_models/inventory.txt"))
    repo_name = "+drake_dep_repositories+drake_models/"
    with open(inventory, encoding="utf-8") as f:
        for line in f.readlines():
            assert line.startswith(repo_name), line
            filename = line[len(repo_name):].strip()
            resource_path = f"drake_models/{filename}"
            yield resource_path, Path(manifest.Rlocation(resource_path))


def _get_all_models() -> Iterator[str]:
    """Generates the Paths of all top-level model files in @drake_models.
    The result is URLs like "package://drake_models/veggies/pepper.sdf".
    """
    for resource_path, filesystem_path in _inventory():
        url = f"package://{resource_path}"
        if any([resource_path.endswith(".dmd.yaml"),
                resource_path.endswith(".sdf"),
                resource_path.endswith(".urdf")]):
            yield url
        elif resource_path.endswith(".xml"):
            # We can't tell a MuJoCo file just by its suffix.
            if "<mujoco" in filesystem_path.read_text(encoding="utf-8"):
                yield url
    return


def _get_all_test_cases():
    """Checks that all model files in @drake_models parse without any
    errors nor warnings.
    """
    all_models = set(_get_all_models())

    # Allow warnings on these models until they are repaired.
    models_with_warnings = set([
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
    ])
    stragglers = models_with_warnings - all_models
    assert not stragglers, repr(stragglers)

    # Allow errors on these models until they are repaired.
    models_with_errors = set([
        # This file is not designed to be loaded independently from its
        # parent file `homecart.dmd.yaml`; it will always error out.
        "package://drake_models/tri_homecart/homecart_grippers.dmd.yaml",
    ])
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
    def test_model(self, *,
                   url: str,
                   expect_warnings: bool,
                   expect_errors: bool):
        """Checks that when parsed, the given url contains warnings and/or
        errors consistent with the expected conditions.
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
