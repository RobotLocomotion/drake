"""
Ensures that all models available via pydrake are parseable by MultibodyPlant
and SceneGraph.
"""

import copy
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
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.0)
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
        for model_file in copy.copy(model_files):
            model_relpath = os.path.relpath(model_file, drake_dir)
            if model_relpath.startswith("external/sdformat/sdf"):
                # These are schema files; do not load.
                model_files.remove(model_file)
        # We expect there to be dozens of files that should be available for
        # parsing. Do a quick check to make sure we're in the right order of
        # magnitude in order to prevent false test success based on changes
        # that might lead to missing files.
        self.assertGreater(len(model_files), 40)
        # Parse each model file for testing.
        for model_file in model_files:
            model_relpath = os.path.relpath(model_file, drake_dir)
            print(f"Test model: {model_relpath}")
            parse_model_and_create_context(model_file)
