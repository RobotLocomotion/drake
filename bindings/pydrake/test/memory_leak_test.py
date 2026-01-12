"""Regression test for memory leaks.

The test contains examples of pydrake code that may leak (DUTs),
instrumentation to detect leaks, and optional additional debug printing under
an internal verbose option.
"""

import dataclasses
import functools
import gc
import sys
import textwrap
import unittest
import weakref

from pydrake.common import RandomGenerator
from pydrake.common.schema import Rotation, Transform
from pydrake.geometry import Meshcat, SceneGraphConfig
from pydrake.lcm import DrakeLcmParams
from pydrake.manipulation import ApplyDriverConfigs, IiwaDriver
from pydrake.multibody.parsing import (
    LoadModelDirectivesFromString,
    ProcessModelDirectives,
)
from pydrake.multibody.plant import AddMultibodyPlant, MultibodyPlantConfig
from pydrake.planning import RobotDiagramBuilder
from pydrake.systems.analysis import (
    ApplySimulatorConfig,
    Simulator,
    SimulatorConfig,
)
from pydrake.systems.framework import DiagramBuilder, LeafSystem
from pydrake.systems.lcm import ApplyLcmBusConfig
from pydrake.systems.primitives import ConstantVectorSource
from pydrake.systems.sensors import ApplyCameraConfig, CameraConfig
from pydrake.visualization import ApplyVisualizationConfig, VisualizationConfig

# Developer-only configuration.
VERBOSE = False


@functools.cache
def _get_meshcat_singleton():
    return Meshcat()


@dataclasses.dataclass(frozen=True)
class _Sentinel:
    """_Sentinel uses `weakref.finalize` to spy on the end of an object's
    lifetime. The test will use this information to determine whether objects
    of interest were properly garbage collected or not, and to provide logging
    of exactly when objects are finalized.

    See also: https://docs.python.org/3/library/weakref.html#weakref.finalize
    """

    finalizer: weakref.finalize
    name: str


def _make_sentinel(obj, name):
    """Makes a _Sentinel for `obj` using `name` for debugging messages.  If
    this instance was created with `VERBOSE=False` (the default), no
    messages will be printed, but the _Sentinel will still track the object.
    """
    if VERBOSE:
        print(f"sentinel: tracked {name} {hex(id(obj))}")

    def done(oid):
        if VERBOSE:
            print(f"sentinel: unmade {name} {hex(oid)}")

    return _Sentinel(finalizer=weakref.finalize(obj, done, id(obj)), name=name)


def _make_sentinels_from_locals(dut_name, locals_dict):
    """Makes _Sentinels for all local variables of interest."""
    # Skip specific types not supported by weakref, as needed.
    return {
        _make_sentinel(value, f"{dut_name}::{key}")
        for key, value in locals_dict.items()
        if not any(isinstance(value, typ) for typ in [list, str])
    }


def _report_sentinels(sentinels, message: str):
    """Prints extensive debug information for a sequence of _Sentinels.
    The message string can provide additional context that may be available at
    the call site.
    """
    print(message)

    for sentinel in sentinels:
        print(f"sentinel for {sentinel.name}")
        finalizer = sentinel.finalizer
        print(f"sentinel alive? {finalizer.alive}")
        if finalizer.alive:
            print(f"ref_count: {sys.getrefcount(finalizer.peek()[0])}")
            o = finalizer.peek()[0]
            is_tracked = gc.is_tracked(o)
            print(f"is_tracked: {is_tracked}")
            if is_tracked:
                print(f"generation: {_object_generation(o)}")
                print(f"referrers: {gc.get_referrers(o)}")
                print(f"referents: {gc.get_referents(o)}")


def _object_generation(o) -> int | None:
    """Returns the garbage collection generation of the passed object, or None
    if the object is not tracked by garbage collection.

    See also: https://github.com/python/cpython/blob/main/InternalDocs/garbage_collector.md#optimization-generations  # noqa
    """
    for gen in range(3):
        gen_list = gc.get_objects(generation=gen)
        if any([x is o for x in gen_list]):
            return gen
    return None


def _dut_simple_source():
    """A device under test that creates and destroys a leaf system."""
    source = ConstantVectorSource([1.0])
    return {_make_sentinel(source, "simple source")}


def _dut_trivial_simulator():
    """A device under test that creates and destroys a simulator that
    contains only a single, simple subsystem.
    """
    builder = DiagramBuilder()
    source = builder.AddSystem(ConstantVectorSource([1.0]))
    source2 = builder.AddSystem(ConstantVectorSource([1.0]))
    diagram = builder.Build()
    simulator = Simulator(system=diagram)
    simulator.AdvanceTo(1.0)
    return _make_sentinels_from_locals("trivial_simulator", locals())


def _dut_mixed_language_simulator():
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
    return _make_sentinels_from_locals("mixed_language_simulator", locals())


def _dut_full_example():
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
    directives = LoadModelDirectivesFromString(
        textwrap.dedent("""
    directives:
    - add_model:
        name: amazon_table
        file: |-
          package://drake_models/manipulation_station/amazon_table_simplified.sdf
    - add_weld:
        parent: world
        child: amazon_table::amazon_table
    - add_model:
        name: iiwa
        file: |-
          package://drake_models/iiwa_description/urdf/iiwa14_primitive_collision.urdf
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
        file: |-
          package://drake_models/wsg_50_description/sdf/schunk_wsg_50_with_tip.sdf
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
        file: |-
          package://drake_models/veggies/yellow_bell_pepper_no_stem_low.sdf
        default_free_body_pose:
          flush_bottom_center__z_up:
            base_frame: amazon_table::amazon_table
            translation: [0, 0.10, 0.20]
    """)
    )
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
    return _make_sentinels_from_locals("full_example", locals())


class MyPyLeafSystem(LeafSystem):
    def __init__(self):
        super().__init__()  # Don't forget to initialize the base class.
        # Comment/uncomment this one line to turn leak off or on.
        self._a_port = self.DeclareVectorInputPort(name="a", size=2)


def _dut_mypyleafsystem():
    mypyleafsystem = MyPyLeafSystem()
    return _make_sentinels_from_locals("mypyleafsystem", locals())


def _repeat(*, dut: callable, count: int):
    """Calls dut() for count times in a row, performing a full garbage
    collection before and after each call. Tracks memory leaks of interest; the
    count of repetitions with leaks is returned. If `VERBOSE=True`, additional
    debug information will be printed.

    Args:
        dut() -> Sequence[_Sentinel]: a callable function containing code to
                                     test for leaks, and returning _Sentinels
                                     for data of interest.
        count: the number of times to invoke `dut`.

    Returns:
        int: the total number of repetitions that leaked objects, as detected
             by examining returned _Sentinels.
    """
    gc.collect()
    # Call the dut repeatedly, observing tracked blocks.
    leaks = 0
    for i in range(count):
        sentinels = dut()
        if VERBOSE:
            _report_sentinels(sentinels, "before collect")
        gc.collect()
        if VERBOSE:
            _report_sentinels(sentinels, "after collect")
        leaks += any([sentinel.finalizer.alive for sentinel in sentinels])
    return leaks


class TestMemoryLeaks(unittest.TestCase):
    def do_test(self, *, dut, count, leaks_allowed=0, leaks_required=0):
        """Runs the requested `dut` (see _repeat() above) for `count`
        iterations. Check that leaks detected <= leaks allowed. In addition,
        check if the leaks required <= the actual leaks measured. Using a non-0
        leaks_required will cause the test to fail if fixes get implemented. In
        that case, the test can likely be updated to be more strict.
        """
        leaks = _repeat(dut=dut, count=count)
        self.assertLessEqual(leaks, leaks_allowed)
        self.assertGreaterEqual(leaks, leaks_required)

    def test_simple_source(self):
        self.do_test(dut=_dut_simple_source, count=10)

    def test_trivial_simulator(self):
        self.do_test(dut=_dut_trivial_simulator, count=5)

    def test_mixed_language_simulator(self):
        self.do_test(dut=_dut_mixed_language_simulator, count=5)

    def test_full_example(self):
        # Note: this test doesn't invoke the #14355 deliberate cycle.
        self.do_test(dut=_dut_full_example, count=1)

    def test_mypyleafsystem(self):
        self.do_test(dut=_dut_mypyleafsystem, count=1)
