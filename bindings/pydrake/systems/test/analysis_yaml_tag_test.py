"""Checks that SimulatorConfig YAML works with minimal imports.

N.B. This is deliberately its own test file: it must NOT import
pydrake.multibody.cenic (as analysis_test.py does), so that we can confirm the
CenicIntegratorParameters YAML type tag resolves without it.
"""

import sys
import unittest

from pydrake.common.yaml import yaml_dump_typed, yaml_load_typed
from pydrake.systems.analysis import SimulatorConfig


class TestAnalysisYamlTag(unittest.TestCase):
    def test_cenic_yaml_tag_without_cenic_import(self):
        # The YAML type tag must resolve whenever SimulatorConfig is available,
        # even if pydrake.multibody.cenic is never imported.
        self.assertNotIn("pydrake.multibody.cenic", sys.modules)
        config = yaml_load_typed(
            schema=SimulatorConfig,
            data="""
integration_scheme: cenic
integrator_params: !CenicIntegratorParameters
  explicit_external_inputs: true
""",
        )
        self.assertTrue(config.integrator_params.explicit_external_inputs)
        self.assertIn("!CenicIntegratorParameters", yaml_dump_typed(config))
