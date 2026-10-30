"""Regression test for memory leaks.

The test contains examples of pydrake code that may leak (DUTs),
instrumentation to detect leaks, and optional additional debug printing under
an internal verbose option.

Two test classes provide instrumentation for the likely categories of leaks:

* TestPythonMemoryLeaks detects leaks of python objects after garbage
  collection.

* TestNativeHeapLeaks detects leaks of memory in the native code heap, using
  instrumentation only supported on Linux.
"""

import ctypes
import dataclasses
import functools
import gc
import sys
import textwrap
import unittest
import weakref

import numpy as np

from pydrake.common import RandomGenerator
from pydrake.common.schema import Rotation, Transform
from pydrake.common.value import Value
from pydrake.examples import PendulumPlant
from pydrake.geometry import Meshcat, SceneGraphConfig
from pydrake.lcm import DrakeLcm, DrakeLcmParams
from pydrake.manipulation import ApplyDriverConfigs, IiwaDriver
from pydrake.multibody.inverse_kinematics import (
    DifferentialInverseKinematicsSystem,
)
from pydrake.multibody.parsing import (
    LoadModelDirectivesFromString,
    ProcessModelDirectives,
)
from pydrake.multibody.plant import (
    AddMultibodyPlant,
    AddMultibodyPlantSceneGraph,
    MultibodyPlantConfig,
)
from pydrake.planning import (
    CollisionCheckerParams,
    DirectCollocation,
    IrisParameterizationFunction,
    RobotDiagramBuilder,
    SceneGraphCollisionChecker,
)
from pydrake.solvers import (
    GurobiSolver,
    MathematicalProgram,
    MinimumValueLowerBoundConstraint,
    MinimumValueUpperBoundConstraint,
    ProjectedGradientDescentSolver,
)
from pydrake.systems.analysis import (
    ApplySimulatorConfig,
    Simulator,
    SimulatorConfig,
)
from pydrake.systems.framework import (
    DiagramBuilder,
    InputPortSelection,
    LeafSystem,
    ValueProducer,
)
from pydrake.systems.lcm import ApplyLcmBusConfig
from pydrake.systems.primitives import ConstantVectorSource
from pydrake.systems.sensors import ApplyCameraConfig, CameraConfig
from pydrake.trajectories import FunctionHandleTrajectory
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
        if any(x is o for x in gen_list):
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


def _global_callback(*args):
    """A callback that refers to nothing in particular."""
    del args


def _make_callback(owner, cyclic: bool, result=None):
    """Returns a callback for the #24162 devices under test below, which
    returns `result` when called. When `cyclic` is true, the callback refers
    back to `owner`, so storing it in C++ within `owner` forms a reference
    cycle that garbage collection must be able to see. Otherwise, the callback
    does not refer to `owner`.
    """
    if not cyclic:
        return lambda *args: result

    def callback(*args):
        del args
        assert owner is not None  # Refers to `owner`, to capture it.
        return result

    return callback


def _sentinels_for(dut_name, owner):
    return {_make_sentinel(owner, dut_name)}


# The following devices under test each store a callback inside some pydrake
# object, using one of the bindings identified in #24162.


def _dut_abstract_output_port(cyclic):
    system = LeafSystem()
    system.DeclareAbstractOutputPort(
        "y",
        lambda: Value[object](None),
        _make_callback(system, cyclic),
    )
    return _sentinels_for("abstract_output_port", system)


def _dut_cache_entry(cyclic):
    system = LeafSystem()
    system.DeclareCacheEntry(
        description="cache",
        value_producer=ValueProducer(
            allocate=lambda: Value[object](None),
            calc=_make_callback(system, cyclic),
        ),
    )
    return _sentinels_for("cache_entry", system)


def _dut_simulator_monitor(cyclic):
    simulator = Simulator(ConstantVectorSource([1.0]))
    simulator.set_monitor(_make_callback(simulator, cyclic))
    return _sentinels_for("simulator_monitor", simulator)


def _make_program():
    prog = MathematicalProgram()
    return prog, prog.NewContinuousVariables(2)


def _dut_program_cost(cyclic):
    prog, x = _make_program()
    prog.AddCost(_make_callback(prog, cyclic), x)
    return _sentinels_for("program_cost", prog)


def _dut_program_constraint(cyclic):
    prog, x = _make_program()
    prog.AddConstraint(_make_callback(prog, cyclic), [0.0], [1.0], x)
    return _sentinels_for("program_constraint", prog)


def _dut_program_visualization_callback(cyclic):
    prog, x = _make_program()
    prog.AddVisualizationCallback(_make_callback(prog, cyclic), x)
    return _sentinels_for("program_visualization_callback", prog)


def _dut_lcm_subscribe(cyclic):
    lcm = DrakeLcm("memq://")
    lcm.Subscribe("CHANNEL", _make_callback(lcm, cyclic))
    return _sentinels_for("lcm_subscribe", lcm)


def _dut_lcm_subscribe_multichannel(cyclic):
    lcm = DrakeLcm("memq://")
    lcm.SubscribeMultichannel("CHAN.*", _make_callback(lcm, cyclic))
    return _sentinels_for("lcm_subscribe_multichannel", lcm)


def _dut_lcm_subscribe_all_channels(cyclic):
    lcm = DrakeLcm("memq://")
    lcm.SubscribeAllChannels(_make_callback(lcm, cyclic))
    return _sentinels_for("lcm_subscribe_all_channels", lcm)


def _dut_function_handle_trajectory(cyclic):
    # The trajectory doesn't store a reference to anything that could store
    # it in turn, so we need a mutable holder to form the cycle.
    holder = []
    trajectory = FunctionHandleTrajectory(
        _make_callback(holder, cyclic, result=np.zeros((1, 1))), rows=1
    )
    holder.append(trajectory)
    return _sentinels_for("function_handle_trajectory", trajectory)


def _dut_minimum_value_constraint(cyclic):
    holder = []
    constraint = MinimumValueLowerBoundConstraint(
        num_vars=1,
        minimum_value_lower=0.1,
        influence_value_offset=0.1,
        max_num_values=1,
        value_function=_make_callback(holder, cyclic),
    )
    holder.append(constraint)
    return _sentinels_for("minimum_value_constraint", constraint)


def _dut_minimum_value_penalty_function(cyclic):
    constraint = MinimumValueUpperBoundConstraint(
        num_vars=1,
        minimum_value_upper=0.1,
        influence_value_offset=0.1,
        max_num_values=1,
        value_function=_global_callback,
    )
    constraint.set_penalty_function(
        _make_callback(constraint, cyclic, result=(0.0, 0.0))
    )
    return _sentinels_for("minimum_value_penalty_function", constraint)


def _dut_iris_parameterization_function(cyclic):
    holder = []
    parameterization = IrisParameterizationFunction(
        _make_callback(holder, cyclic), dimension=1
    )
    holder.append(parameterization)
    return _sentinels_for("iris_parameterization_function", parameterization)


def _dut_projected_gradient_descent_projection(cyclic):
    solver = ProjectedGradientDescentSolver()
    solver.SetCustomProjectionFunction(_make_callback(solver, cyclic))
    return _sentinels_for("projected_gradient_descent_projection", solver)


def _dut_projected_gradient_descent_gradient(cyclic):
    solver = ProjectedGradientDescentSolver()
    solver.SetCustomGradientFunction(_make_callback(solver, cyclic))
    return _sentinels_for("projected_gradient_descent_gradient", solver)


def _make_collision_checker(distance_function=lambda q1, q2: 0.0):
    builder = RobotDiagramBuilder()
    (model_instance,) = builder.parser().AddModelsFromString(
        textwrap.dedent("""
    <?xml version="1.0"?>
    <sdf version="1.7">
      <model name="robot">
        <link name="base"/>
        <link name="arm"/>
        <joint name="shoulder" type="revolute">
          <parent>base</parent>
          <child>arm</child>
          <axis><xyz>0 0 1</xyz></axis>
        </joint>
      </model>
    </sdf>
    """).strip(),
        "sdf",
    )
    plant = builder.plant()
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base"))
    params = CollisionCheckerParams()
    params.model = builder.Build()
    params.robot_model_instances = [model_instance]
    params.edge_step_size = 0.1
    params.configuration_distance_function = distance_function
    return SceneGraphCollisionChecker(params)


def _dut_collision_checker_distance_function(cyclic):
    checker = _make_collision_checker()
    checker.SetConfigurationDistanceFunction(
        _make_callback(checker, cyclic, result=0.0)
    )
    return _sentinels_for("collision_checker_distance_function", checker)


def _dut_collision_checker_interpolation_function(cyclic):
    checker = _make_collision_checker()
    checker.SetConfigurationInterpolationFunction(
        _make_callback(checker, cyclic, result=np.zeros(1))
    )
    return _sentinels_for("collision_checker_interpolation_function", checker)


def _dut_collision_checker_params_distance_function(cyclic):
    # Here the callback is stored in the params struct (and copied from there
    # into the checker), instead of being set on the checker directly.
    owner_box = []
    checker = _make_collision_checker(
        _make_callback(owner_box, cyclic, result=0.0)
    )
    owner_box.append(checker)
    return _sentinels_for("collision_checker_params_distance_function", checker)


def _make_direct_collocation():
    plant = PendulumPlant()
    return DirectCollocation(
        plant,
        plant.CreateDefaultContext(),
        num_time_samples=3,
        minimum_time_step=0.2,
        maximum_time_step=0.5,
        input_port_index=InputPortSelection.kUseFirstInputIfItExists,
        assume_non_continuous_states_are_fixed=False,
    )


def _dut_dircol_input_callback(cyclic):
    dircol = _make_direct_collocation()
    dircol.AddInputTrajectoryCallback(_make_callback(dircol, cyclic))
    return _sentinels_for("dircol_input_callback", dircol)


def _dut_dircol_state_callback(cyclic):
    dircol = _make_direct_collocation()
    dircol.AddStateTrajectoryCallback(_make_callback(dircol, cyclic))
    return _sentinels_for("dircol_state_callback", dircol)


def _dut_dircol_complete_callback(cyclic):
    dircol = _make_direct_collocation()
    dircol.NewSequentialVariable(rows=1, name="test")
    dircol.AddCompleteTrajectoryCallback(
        _make_callback(dircol, cyclic), names=["test"]
    )
    return _sentinels_for("dircol_complete_callback", dircol)


def _dut_gurobi_mip_node_callback(cyclic):
    solver = GurobiSolver()
    solver.AddMipNodeCallback(_make_callback(solver, cyclic))
    return _sentinels_for("gurobi_mip_node_callback", solver)


def _dut_gurobi_mip_sol_callback(cyclic):
    solver = GurobiSolver()
    solver.AddMipSolCallback(_make_callback(solver, cyclic))
    return _sentinels_for("gurobi_mip_sol_callback", solver)


def _dut_diff_ik_select_data(cyclic):
    Ingredient = DifferentialInverseKinematicsSystem.CollisionConstraint
    constraint = Ingredient(Ingredient.Config())
    constraint.SetSelectDataForCollisionConstraintFunction(
        _make_callback(constraint, cyclic)
    )
    return _sentinels_for("diff_ik_select_data", constraint)


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
        leaks += any(sentinel.finalizer.alive for sentinel in sentinels)
    return leaks


class TestPythonMemoryLeaks(unittest.TestCase):
    """Detects leaks of Python objects."""

    def do_test(self, *, dut, count, leaks_allowed=0, leaks_required=0):
        """Runs the requested `dut` (see _repeat() above) for `count`
        iterations. Checks that leaks detected <= leaks allowed. In addition,
        checks if the leaks required <= the actual leaks measured. Using a
        non-0 leaks_required will cause the test to fail if fixes get
        implemented. In that case, the test can likely be updated to be more
        strict.
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

    def test_stored_callbacks(self):
        """Checks bindings that store Python callbacks in C++ (#24162). With
        a non-cyclic callback, nothing may leak. With a cyclic callback, each
        of these currently leaks; when one gets fixed, its `leaks_required`
        here will fail, and should be changed to 0.
        """
        duts = [
            _dut_abstract_output_port,
            _dut_cache_entry,
            _dut_simulator_monitor,
            _dut_program_cost,
            _dut_program_constraint,
            _dut_program_visualization_callback,
            _dut_lcm_subscribe,
            _dut_lcm_subscribe_multichannel,
            _dut_lcm_subscribe_all_channels,
            _dut_minimum_value_constraint,
            _dut_minimum_value_penalty_function,
            _dut_projected_gradient_descent_projection,
            _dut_projected_gradient_descent_gradient,
            _dut_collision_checker_distance_function,
            _dut_collision_checker_interpolation_function,
            _dut_collision_checker_params_distance_function,
            _dut_dircol_input_callback,
            _dut_dircol_state_callback,
            _dut_dircol_complete_callback,
            _dut_gurobi_mip_node_callback,
            _dut_gurobi_mip_sol_callback,
            _dut_diff_ik_select_data,
        ]
        for dut in duts:
            with self.subTest(dut=dut.__name__, cyclic=False):
                self.do_test(dut=functools.partial(dut, cyclic=False), count=1)
            with self.subTest(dut=dut.__name__, cyclic=True):
                # TODO(#24162) These should not leak.
                self.do_test(
                    dut=functools.partial(dut, cyclic=True),
                    count=1,
                    leaks_allowed=1,
                    leaks_required=1,
                )

        # FunctionHandleTrajectory is a copyable value type, and many C++
        # classes (TrajectorySource, CompositeTrajectory, etc.) copy it. We
        # can't borrow its callables without protecting every one of those
        # copies, so it deliberately owns them, and cyclic callbacks leak.
        #
        # IrisParameterizationFunction is the same, because it is copied into
        # IrisZoOptions and IrisNp2Options.
        for dut in [
            _dut_function_handle_trajectory,
            _dut_iris_parameterization_function,
        ]:
            with self.subTest(dut=dut.__name__, cyclic=False):
                self.do_test(dut=functools.partial(dut, cyclic=False), count=1)
            with self.subTest(dut=dut.__name__, cyclic=True):
                self.do_test(
                    dut=functools.partial(dut, cyclic=True),
                    count=1,
                    leaks_allowed=1,
                    leaks_required=1,
                )


class Mallinfo2(ctypes.Structure):
    """Structure returned by libc `mallinfo2()`. See `man 3 mallinfo`."""

    _fields_ = [
        ("arena", ctypes.c_size_t),
        ("ordlbks", ctypes.c_size_t),
        ("smblks", ctypes.c_size_t),
        ("hblks", ctypes.c_size_t),
        ("hblkhd", ctypes.c_size_t),
        ("usmblks", ctypes.c_size_t),
        ("fsmblks", ctypes.c_size_t),
        ("uordblks", ctypes.c_size_t),
        ("fordblks", ctypes.c_size_t),
        ("keepcost", ctypes.c_size_t),
    ]


def _dut_bug24529():
    """A device under test that uses The `time_step` signature of
    AddMultibodyPlantSceneGraph; see issue #24529.
    """
    builder = DiagramBuilder()
    plant, _ = AddMultibodyPlantSceneGraph(builder, time_step=0.001)
    plant.Finalize()
    builder.Build()


@unittest.skipUnless(sys.platform == "linux", "native heap tests require Linux")
class TestNativeHeapLeaks(unittest.TestCase):
    """Detects leaks in the native code heap. The detection method depends on
    Linux libc features."""

    def setUp(self):
        libc = ctypes.CDLL(None)
        self._mallinfo2 = libc.mallinfo2
        self._mallinfo2.restype = Mallinfo2

    def do_test(self, *, dut):
        """Runs the requested `dut` for a fixed number of iterations doing a
        Python garbage collection before and after each. Checks that native
        heap growth per iteration reaches 0.
        """
        iterations = 20
        for _ in range(iterations):
            gc.collect()
            before = self._mallinfo2()
            dut()
            gc.collect()
            after = self._mallinfo2()
            growth = after.uordblks - before.uordblks
            if growth == 0:
                break
        self.assertEqual(
            growth,
            0,
            f"heap still growing by {growth} bytes"
            f" after {iterations} iterations",
        )

    def test_bug24529(self):
        self.do_test(dut=_dut_bug24529)
