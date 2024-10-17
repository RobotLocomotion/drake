"""Eventually this program might grow up to be an actual regression test for
memory leaks, but for now it merely serves to demonstrate such leaks.

Currently, it neither asserts the absence of leaks (i.e., a real test) nor the
presence of leaks (i.e., an expect-fail test) -- instead, it's a demonstration
that we can instrument and observe by hand, to gain traction on the problem.
"""

import dataclasses
import functools
import gc
import sys
import textwrap
import unittest
import weakref

from pydrake.planning import RobotDiagramBuilder
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.primitives import ConstantVectorSource

from pydrake.common import RandomGenerator
from pydrake.common.schema import Rotation, Transform
from pydrake.lcm import DrakeLcmParams
from pydrake.geometry import Meshcat
from pydrake.manipulation import ApplyDriverConfigs, IiwaDriver
from pydrake.geometry import SceneGraphConfig
from pydrake.multibody.plant import AddMultibodyPlant, MultibodyPlantConfig
from pydrake.multibody.parsing import (
    LoadModelDirectivesFromString,
    ProcessModelDirectives,
)
from pydrake.systems.analysis import (
    ApplySimulatorConfig,
    Simulator,
    SimulatorConfig,
)
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.lcm import ApplyLcmBusConfig
from pydrake.systems.sensors import ApplyCameraConfig, CameraConfig
from pydrake.visualization import ApplyVisualizationConfig, VisualizationConfig


@functools.cache
def _get_meshcat_singleton():
    return Meshcat()


@dataclasses.dataclass(frozen=True)
class Sentinel:
    finalizer: weakref.finalize
    name: str


def _object_generation(o):
    for gen in range(3):
        gen_list = gc.get_objects(generation=gen)
        gen_id_list = [id(x) for x in gen_list]
        if id(o) in gen_id_list:
            return gen
    return -99


def _report_sentinels(sentinels, message):
    print(message)
    for sentinel in sentinels:
        print(f"sentinel for {sentinel.name}")
        finalizer = sentinel.finalizer
        print(f"sentinel alive? {finalizer.alive}")
        if finalizer.alive:
            o = finalizer.peek()[0]
            print(f"generation: {_object_generation(o)}")
            print(f"referrers: {gc.get_referrers(o)}")
            print(f"referents: {gc.get_referents(o)}")
            print(f"is_tracked: {gc.is_tracked(o)}")


def _counts_for_cycle_parts(o, name):
    o_count = sys.getrefcount(o)
    if hasattr(o, "__dict__"):
        dict_count = sys.getrefcount(o.__dict__)
    else:
        dict_count = 0
    if hasattr(o, "_pydrake_ref_cycle_peers"):
        set_count = sys.getrefcount(o._pydrake_ref_cycle_peers)
    else:
        set_count = 0
    print(f"{name}: o count {o_count} dict count {dict_count}"
          f" set count {set_count}")


class _DutRepeater:
    def __init__(self, *, verbose=False):
        self._verbose = verbose
        self._leaks = 0

    def leaks(self):
        return self._leaks

    def make_sentinel(self, obj, name):
        if self._verbose:
            print(f"sentinel: tracked {name} {hex(id(obj))}")

        def done(oid):
            if self._verbose:
                print(f"sentinel: unmade {name} {hex(oid)}")
        return Sentinel(finalizer=weakref.finalize(obj, done, id(obj)),
                        name=name)

    def make_sentinels_from_locals(self, dut_name, locals_dict):
        # Skip `self`; it is not part of the test setup. Skip specific types as
        # needed.
        return {self.make_sentinel(value, f"{dut_name}::{key}")
                for key, value in locals_dict.items()
                if key != 'self'
                and not any(isinstance(value, typ) for typ in [list, str])}

    def dut_simple_source(self):
        """A device under test that creates and destroys a leaf system."""
        source = ConstantVectorSource([1.0])
        return {self.make_sentinel(source, "simple source")}

    def dut_trivial_simulator(self):
        """A device under test that creates and destroys a simulator that
        contains only a single, simple subsystem.
        """
        builder = DiagramBuilder()
        source = builder.AddSystem(ConstantVectorSource([1.0]))
        source2 = builder.AddSystem(ConstantVectorSource([1.0]))
        diagram = builder.Build()
        simulator = Simulator(system=diagram)
        simulator.AdvanceTo(1.0)
        if self._verbose:
            for key, value in locals().items():
                _counts_for_cycle_parts(value, key)
        return self.make_sentinels_from_locals("trivial_simulator", locals())

    def dut_mixed_language_simulator(self):
        """A device under test that creates and destroys a simulator that
        contains subsystems written in both C++ and Python.
        """
        builder = RobotDiagramBuilder()
        builder.builder().AddSystem(ConstantVectorSource([1.0]))
        diagram = builder.Build()
        simulator = Simulator(system=diagram)
        simulator.AdvanceTo(1.0)
        context = simulator.get_context()
        plant = diagram.plant()
        plant_context = plant.GetMyContextFromRoot(context)
        plant.EvalSceneGraphInspector(plant_context)
        return self.make_sentinels_from_locals("mixed_language_simulator",
                                               locals())

    def dut_full_example(self):
        """A device under test that creates and destroys a simulator that
        contains everything a full-stack simulation would ever use.
        """
        builder = DiagramBuilder()
        plant, scene_graph = AddMultibodyPlant(
            plant_config=MultibodyPlantConfig(
                time_step=0.01,
            ),
            scene_graph_config=SceneGraphConfig(),
            builder=builder,
        )
        directives = LoadModelDirectivesFromString(textwrap.dedent("""  # noqa
        directives:
        - add_model:
            name: amazon_table
            file: package://drake_models/manipulation_station/amazon_table_simplified.sdf
        - add_weld:
            parent: world
            child: amazon_table::amazon_table
        - add_model:
            name: iiwa
            file: package://drake_models/iiwa_description/urdf/iiwa14_primitive_collision.urdf
            default_joint_positions:
              iiwa_joint_1: [-0.2]
              iiwa_joint_2: [0.79]
              iiwa_joint_3: [0.32]
              iiwa_joint_4: [-1.76]
              iiwa_joint_5: [-0.36]
              iiwa_joint_6: [0.64]
              iiwa_joint_7: [-0.73]
        - add_frame:
            name: iiwa_on_world
            X_PF:
              base_frame: world
              translation: [0, -0.7, 0.1]
              rotation: !Rpy { deg: [0, 0, 90] }
        - add_weld:
            parent: iiwa_on_world
            child: iiwa::base
        - add_model:
            name: wsg
            file: package://drake_models/wsg_50_description/sdf/schunk_wsg_50_with_tip.sdf
            default_joint_positions:
              left_finger_sliding_joint: [-0.02]
              right_finger_sliding_joint: [0.02]
        - add_frame:
            name: wsg_on_iiwa
            X_PF:
              base_frame: iiwa_link_7
              translation: [0, 0, 0.114]
              rotation: !Rpy { deg: [90, 0, 90] }
        - add_weld:
            parent: wsg_on_iiwa
            child: wsg::body
        - add_model:
            name: bell_pepper
            file: package://drake_models/veggies/yellow_bell_pepper_no_stem_low.sdf
            default_free_body_pose:
              flush_bottom_center__z_up:
                base_frame: amazon_table::amazon_table
                translation: [0, 0.10, 0.20]
        """))
        added_models = ProcessModelDirectives(
            plant=plant,
            directives=directives,
        )
        plant.Finalize()
        lcm_buses = ApplyLcmBusConfig(
            builder=builder,
            lcm_buses={
                "default": DrakeLcmParams(),
            },
        )
        ApplyDriverConfigs(
            builder=builder,
            sim_plant=plant,
            models_from_directives=added_models,
            lcm_buses=lcm_buses,
            driver_configs={
                "iiwa": IiwaDriver(
                    hand_model_name="wsg",
                ),
            },
        )
        ApplyCameraConfig(
            builder=builder,
            lcm_buses=lcm_buses,
            config=CameraConfig(
                name="camera_0",
                X_PB=Transform(
                    translation=[1.5, 0.8, 1.25],
                    rotation=Rotation(value=Rotation.Rpy(deg=[-120, 5, 125])),
                ),
            ),
        )
        ApplyVisualizationConfig(
            builder=builder,
            lcm_buses=lcm_buses,
            config=VisualizationConfig(),
            meshcat=_get_meshcat_singleton(),
        )
        diagram = builder.Build()
        simulator = Simulator(system=diagram)
        ApplySimulatorConfig(
            simulator=simulator,
            config=SimulatorConfig(),
        )
        random = RandomGenerator(22)
        diagram.SetRandomContext(simulator.get_mutable_context(), random)
        simulator.AdvanceTo(0.5)
        return self.make_sentinels_from_locals("full_simulator", locals())

    def repeat(self, *, dut: callable, count: int):
        """Returns the details of calling dut() for count times in a row."""
        # Pre-allocate all of our return values.
        gc.collect()
        # Call the dut repeatedly, observing tracked blocks.
        for i in range(count):
            sentinels = dut()
            if self._verbose:
                _report_sentinels(sentinels, "before collect")
            gc.collect()
            if self._verbose:
                _report_sentinels(sentinels, "after collect")
            self._leaks += any(
                [sentinel.finalizer.alive for sentinel in sentinels])


class TestMemoryLeaks(unittest.TestCase):
    def setUp(self):
        self._repeater = _DutRepeater(verbose=False)

    def do_test(self, *, dut, count, leaks_allowed=0):
        self._repeater.repeat(dut=dut, count=count)
        leaks = self._repeater.leaks()
        self.assertLessEqual(leaks, leaks_allowed)
        if leaks < leaks_allowed:
            # We succeeded in some sense, but the leaks allowed count is now
            # stale.
            self.assertEqual(
                leaks, leaks_allowed,
                f"leaks={leaks} leaks_allowed={leaks_allowed};"
                f" leaks_allowed count is stale, please update the test.")


    def test_simple_source(self):
        self.do_test(
            dut=self._repeater.dut_simple_source,
            count=10)

    def test_trivial_simulator(self):
        self.do_test(
            dut=self._repeater.dut_trivial_simulator,
            count=5,
            # TODO(rpoyner-tri): Allow 0 leaks.
            leaks_allowed=5)

    def test_mixed_language_simulator(self):
        self.do_test(
            dut=self._repeater.dut_mixed_language_simulator,
            count=5,
            # TODO(rpoyner-tri): Allow 0 leaks.
            leaks_allowed=1)

    def test_full_example(self):
        # Note: this test doesn't invoke the #14355 deliberate cycle.
        self.do_test(
            dut=self._repeater.dut_full_example,
            count=2)
