# -*- coding: utf-8 -*-

import pydrake.manipulation as mut

import unittest

from pydrake.common import FindResourceOrThrow
from pydrake.multibody.parsing import (
    LoadModelDirectives,
    Parser,
    ProcessModelDirectives,
)
from pydrake.manipulation import ApplyDriverConfigs
from pydrake.multibody.plant import MultibodyPlant
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.lcm import LcmBuses


class TestUtil(unittest.TestCase):
    def test_driver(self):
        dut = mut.ZeroForceDriver()
        self.assertIn("ZeroForceDriver", repr(dut))

        builder = DiagramBuilder()
        plant = builder.AddSystem(MultibodyPlant(1.))
        parser = Parser(plant)
        directives = LoadModelDirectives(FindResourceOrThrow(
            "drake/manipulation/util/test/iiwa7_wsg.dmd.yaml"))
        models_from_directives = ProcessModelDirectives(directives, parser)
        plant.Finalize()
        model_dict = dict()
        for model in models_from_directives:
            model_dict[model.model_name] = model

        self.assertEqual(len(builder.GetSystems()), 1)
        mut.ApplyDriverConfig(
            driver_config=dut, model_instance_name="iiwa7", sim_plant=plant,
            models_from_directives=model_dict, lcms=LcmBuses(),
            builder=builder)
        self.assertEqual(len(builder.GetSystems()), 2)

    def test_apply_driver_configs(self):
        """Checks the ApplyDriverConfigs from our parent module.
        """
        builder = DiagramBuilder()
        plant = builder.AddSystem(MultibodyPlant(1.))
        parser = Parser(plant)
        directives = LoadModelDirectives(FindResourceOrThrow(
            "drake/manipulation/util/test/iiwa7_wsg.dmd.yaml"))
        models_from_directives = ProcessModelDirectives(directives, parser)
        plant.Finalize()

        driver_configs = {
            "iiwa7": mut.ZeroForceDriver(),
            "schunk_wsg": mut.ZeroForceDriver(),
        }
        lcm_buses = LcmBuses()

        self.assertEqual(len(builder.GetSystems()), 1)
        ApplyDriverConfigs(
            driver_configs=driver_configs,
            sim_plant=plant,
            models_from_directives=models_from_directives,
            lcm_buses=lcm_buses,
            builder=builder)
        self.assertEqual(len(builder.GetSystems()), 3)
