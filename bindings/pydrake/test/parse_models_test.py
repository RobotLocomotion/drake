"""
Ensures that all models available via pydrake are parseable by MultibodyPlant
and SceneGraph.
"""

import glob
import os
import unittest

from pydrake.common import FindResourceOrThrow
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.framework import DiagramBuilder


def find_model_files(base_dir):
    return (
        glob.glob(os.path.join(base_dir, "**/*.sdf"), recursive=True)
        + glob.glob(os.path.join(base_dir, "**/*.urdf"), recursive=True))


def parse_model_and_create_context(file):
    """Tests a model by loading parsing it with a SceneGraph connected,
    building the relevant diagram, and allocating its default context."""
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder)
    Parser(plant).AddModelFromFile(file)
    plant.Finalize()
    diagram = builder.Build()
    diagram.CreateDefaultContext()


class TestParseModels(unittest.TestCase):
    def test_available_runfiles(self):
        # Find all available Drake models from runfiles.
        drake_dir = os.path.dirname(FindResourceOrThrow(
            "drake/.drake-find_resource-sentinel"))
        model_files = find_model_files(drake_dir)
        # Filter model files.
        check_count = 0
        # Load each file.
        for model_file in model_files:
            model_relpath = os.path.relpath(model_file, drake_dir)
            if model_relpath.startswith("external/sdformat/sdf"):
                # These are schema files; do not load.
                continue
            print(f"Test model: {model_relpath}")
            parse_model_and_create_context(model_file)
            check_count += 1
        # Simple sanity check.
        self.assertGreater(check_count, 10)
