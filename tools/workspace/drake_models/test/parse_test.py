from pathlib import Path
from typing import Iterator
import unittest

from python import runfiles

from pydrake.planning import RobotDiagramBuilder


class TestDrakeModels(unittest.TestCase):

    @staticmethod
    def package_xml() -> Path:
        """Returns the path to drake_models/package.xml in runfiles.
        """
        manifest = runfiles.Create()
        result = Path(manifest.Rlocation("drake_models/package.xml"))
        assert result.exists(), result
        return result

    @staticmethod
    def inventory() -> Iterator[str]:
        """Generates the list of all the runfiles of @drake_models. The result
        is strings like "drake_models/veggies/pepper.png".
        """
        manifest = runfiles.Create()
        inventory = Path(manifest.Rlocation(
            "drake/tools/workspace/drake_models/inventory.txt"))
        with open(inventory, encoding="utf-8") as f:
            for line in f.readlines():
                assert line.startswith("drake_models/")
                yield line.strip()

    @staticmethod
    def get_all_models() -> Iterator[str]:
        """Generates the Paths of all top-level model files in @drake_models.
        The result is URLs like "package://drake_models/veggies/pepper.sdf".
        """
        for path in TestDrakeModels.inventory():
            url = f"package://{path}"
            if any([path.endswith(".dmd.yaml"),
                    path.endswith(".sdf"),
                    path.endswith(".urdf")]):
                yield url
            elif path.endswith(".xml"):
                # We can't tell a MuJoCo file just by its suffix.
                runfiles = TestDrakeModels.package_xml().parent.parent
                if "<mujoco" in (runfiles / path).open().read():
                    yield url
        return

    def test_all_models(self):
        """Checks that all model files in @drake_models parse without any
        errors nor warnings.
        """
        all_models = list(TestDrakeModels.get_all_models())

        # Allow warnings on these models until they are repaired.
        models_with_warnings = [
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
        ]
        self.assertFalse(set(models_with_warnings) - set(all_models))

        # Allow errors on these models until they are repaired.
        models_with_errors = [
            # This file is not designed to be loaded independently from its
            # parent file `homecart.dmd.yaml`; it will always error out.
            "package://drake_models/tri_homecart/homecart_grippers.dmd.yaml",
        ]
        self.assertFalse(set(models_with_errors) - set(all_models))
        self.assertFalse(set(models_with_warnings) & set(models_with_errors))

        # Run all tests.
        for url in all_models:
            with self.subTest(url=url):
                expect_warnings = url in models_with_warnings
                expect_errors = url in models_with_errors
                self._check_model(
                    url=url,
                    expect_warnings=expect_warnings,
                    expect_errors=expect_errors)

    def _check_model(self, *,
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
        parser.package_map().AddPackageXml(str(self.package_xml()))
        if strict:
            parser.SetStrictParsing()
        parser.AddModels(url=url)
