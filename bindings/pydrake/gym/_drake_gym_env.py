import sys
from typing import Callable, Optional, Union
import warnings

import gymnasium as gym
import numpy as np

from pydrake.common import RandomGenerator
from pydrake.systems.analysis import Simulator, SimulatorStatus
from pydrake.systems.framework import (
    Context,
    InputPort,
    InputPortIndex,
    OutputPortIndex,
    PortDataType,
    System,
)
from pydrake.systems.sensors import ImageRgba8U


assert sys.version_info >= (3, 10), "pydrake.gym requires Python 3.10 or newer"


class DrakeGymEnv(gym.Env):
    """
    DrakeGymEnv provides a gym.Env interface for a Drake System (often a
    Diagram) using a Simulator.
    """

    def __init__(self,
                 simulator: Union[Simulator,
                                  Callable[[RandomGenerator], Simulator]],
                 time_step: float,
                 action_space: gym.spaces.Space,
                 observation_space: gym.spaces.Space,
                 reward: Union[Callable[[System, Context], float],
                               OutputPortIndex, str],
                 action_port_id: Union[InputPort, InputPortIndex, str] = None,
                 observation_port_id: Union[OutputPortIndex, str] = None,
                 render_rgb_port_id: Union[OutputPortIndex, str] = None,
                 render_mode: str = 'human',
                 reset_handler: Callable[[Simulator, Context], None] = None,
                 hardware: bool = False):
        """
        Args:
            simulator: Either a ``drake.systems.analysis.Simulator``, or
                a function that produces a (randomized) Simulator.
            time_step: Each call to ``step()`` will advance the simulator by
                ``time_step`` seconds.
            action_space: Defines the ``gym.spaces.Space`` for the actions.  If
                the action port is vector-valued, then passing ``None``
                defaults to a ``gym.spaces.Box`` of the correct dimension with
                bounds at negative and positive infinity.  Note: Stable
                Baselines 3 strongly encourages normalizing the
                ``action_space`` to [-1, 1].
            observation_space: Defines the ``gym.spaces.Space`` for the
                observations.  If the observation port is vector-valued, then
                passing ``None`` defaults to a ``gym.spaces.Box`` of the
                correct dimension with bounds at negative and positive
                infinity.
            reward: The reward can be specified in one of two
                ways: (1) by passing a callable with the signature
                ``value = reward(system, context)`` or (2) by passing a scalar
                vector-valued output port of ``simulator``'s system.
            action_port_id: The ID of an input port of ``simulator``'s system
                compatible with the ``action_space``.  Each Env *must* have an
                action port; passing ``None`` defaults to using the *first*
                input port (inspired by
                ``InputPortSelection.kUseFirstInputIfItExists``).
            observation_port_id: An output port of ``simulator``'s system
                compatible with the ``observation_space``. Each Env *must* have
                an observation port (it seems that gym doesn't support empty
                observation spaces / open-loop policies); passing ``None``
                defaults to using the *first* input port (inspired by
                ``OutputPortSelection.kUseFirstOutputIfItExists``).
            render_rgb_port_id: An optional output port of ``simulator``'s
                system that returns  an ``ImageRgba8U``; often the
                ``color_image`` port of a Drake ``RgbdSensor``.
            render_mode: The render mode of the environment determined at
                initialization. Defaults to ``human`` which uses visualizers
                inside the System (e.g. MeshcatVisualizer,
                PlanarSceneGraphVisualizer, etc.). ``render_mode`` equal to
                ``rgb_array`` evaluates the ``render_rgb_port`` and ``ansi``
                calls ``__repr__`` on the system Context.
            reset_handler: A function that sets the home state
                (plant, and/or env.) at ``reset()``.
                The reset state can be specified in one of
                the two ways:
                (if ``reset_handler`` is None) setting random context using
                a Drake random_generator
                (e.g. ``joint.set_random_pose_distribution()``
                using the ``reset()`` seed), (otherwise) using
                ``reset_handler()``.
            hardware: If True, it prevents from setting random context at
                ``reset()`` when using ``random_generator``, but it does
                execute ``reset_handler()`` if given.

        Notes (using ``env`` as an instance of this class):

        - You may set simulator/integrator preferences by using
          ``env.simulator`` directly.
        - The ``done`` condition returned by ``step()`` is always False by
          default.  Use ``env.simulator.set_monitor()`` to use Drake's monitor
          functionality for specifying termination conditions.
        - You may additionally wish to directly set ``env.reward_range`` and/or
          ``env.spec``.  See the docs for ``gym.Env`` for more details.
        """
        super().__init__()
        if isinstance(simulator, Simulator):
            self.simulator = simulator
            self.make_simulator = None
        elif callable(simulator):
            self.simulator = None
            self.make_simulator = simulator
        else:
            raise ValueError("Invalid simulator argument")

        assert time_step > 0
        self.time_step = time_step

        assert isinstance(action_space, gym.spaces.Space)
        self.action_space = action_space

        assert isinstance(observation_space, gym.spaces.Space)
        self.observation_space = observation_space

        if isinstance(reward, (OutputPortIndex, str)):
            self.reward_port_id = reward
            self.reward = None
        elif callable(reward):
            self.reward_port_id = None
            self.reward = reward
        else:
            raise ValueError("Invalid reward argument")

        if action_port_id:
            assert isinstance(action_port_id, (InputPortIndex, str))
            self.action_port_id = action_port_id
        else:
            self.action_port_id = InputPortIndex(0)

        if observation_port_id:
            assert isinstance(observation_port_id, (OutputPortIndex, str))
            self.observation_port_id = observation_port_id
        else:
            self.observation_port_id = OutputPortIndex(0)

        self.metadata['render_modes'] = ['human', 'ascii']

        # Setup rendering.
        self.render_mode = render_mode
        if render_rgb_port_id:
            assert isinstance(render_rgb_port_id, (OutputPortIndex, str))
            self.render_mode = 'rgb_array'
            self.metadata['render_modes'].append('rgb_array')
        self.render_rgb_port_id = render_rgb_port_id

        self.generator = RandomGenerator()

        if reset_handler is None or callable(reset_handler):
            self.reset_handler = reset_handler
        else:
            raise ValueError("reset_handler is not callable.")

        self.hardware = hardware

        if self.simulator:
            self._setup()

    def _setup(self):
        """Completes the setup once we have a self.simulator."""
        system = self.simulator.get_system()

        # Setup action port
        if self.action_port_id:
            if isinstance(self.action_port_id, InputPortIndex):
                self.action_port = system.get_input_port(self.action_port_id)
            else:
                self.action_port = system.GetInputPort(self.action_port_id)
        if self.action_port.get_data_type() == PortDataType.kVectorValued:
            assert np.array_equal(self.action_space.shape,
                                  [self.action_port.size()])

        def get_output_port(id):
            if isinstance(id, OutputPortIndex):
                return system.get_output_port(id)
            return system.GetOutputPort(id)

        # Setup observation port
        if self.observation_port_id:
            self.observation_port = get_output_port(self.observation_port_id)
        if self.observation_port.get_data_type() == PortDataType.kVectorValued:
            assert np.array_equal(self.observation_space.shape,
                                  [self.observation_port.size()])

        # Note: We require that there is no direct feedthrough action_port to
        # observation_port.  Unfortunately, HasDirectFeedthrough returns false
        # positives, and would produce noisy warnings.

        # Setup reward.
        if self.reward_port_id:
            reward_port = get_output_port(self.reward_port_id)
            self.reward = lambda system, context: reward_port.Eval(context)[0]

        # Setup rendering port.
        if self.render_rgb_port_id:
            self.render_rgb_port = get_output_port(self.render_rgb_port_id)
            assert self.render_rgb_port.get_data_type() == \
                PortDataType.kAbstractValued
            assert isinstance(self.render_rgb_port.Allocate().get_value(),
                              ImageRgba8U)

    def step(self, action):
        """
        Implements ``gym.Env.step`` to advance the simulation forward by one
        ``self.time_step``.

        Args:
            action: an element from ``self.action_space``.
        """
        assert self.simulator, "You must call reset() first"

        context = self.simulator.get_context()
        time = context.get_time()

        self.action_port.FixValue(context, action)
        truncated = False
        # Observation prior to advancing the simulation.
        prev_observation = self.observation_port.Eval(context)
        info = dict()
        try:
            status = self.simulator.AdvanceTo(time + self.time_step)
        except RuntimeError as e:
            # TODO(JoseBarreiros-TRI) We don't currently check for the
            # error coming from the solver failing to converge.
            warnings.warn("Calling Done after catching RuntimeError:")
            warnings.warn(e.args[0])
            # Truncated is used when the solver failed to converge.
            # Note: this is different than the official use of truncated
            # in Gymnasium:
            # "Whether the truncation condition outside the scope of the MDP
            # is satisfied. Typically, this is a timelimit, but
            # could also be used to indicate an agent physically going out
            # of bounds."
            # We handle the solver failure to converge by returning
            # zero reward and the previous observation since the action
            # was not successfully applied. This comes at a cost of
            # an extra evaluation of the observation port.
            truncated = True
            terminated = False
            reward = 0
            return prev_observation, reward, terminated, truncated, info

        observation = self.observation_port.Eval(context)
        reward = self.reward(self.simulator.get_system(), context)
        terminated = (
            not truncated
            and (status.reason()
                 == SimulatorStatus.ReturnReason.kReachedTerminationCondition))

        return observation, reward, terminated, truncated, info

    def reset(self, *,
              seed: Optional[int] = None,
              options: Optional[dict] = None):
        """
        If a callable "simulator factory" was passed to the constructor, then a
        new simulator is created.  Otherwise this method simply resets the
        ``simulator`` and its Context.
        """
        super().reset(seed=seed)
        assert options is None or options == dict(), (
            "Options are not supported in env.reset() method.")

        if (seed is not None):
            # TODO(ggould) This should not reset the generator if it was
            # already explicitly seeded (see API spec), but we have no way to
            # check that at the moment.
            self.generator = RandomGenerator(seed)

        if self.make_simulator:
            self.simulator = self.make_simulator(self.generator)
            self._setup()

        context = self.simulator.get_mutable_context()
        context.SetTime(0)
        self.simulator.Initialize()
        if self.reset_handler is not None:
            # The initial state is set by reset_handler().
            self.simulator.get_system().SetDefaultContext(context)
            self.reset_handler(self.simulator, context, seed)
        else:
            if not self.hardware:
                self.simulator.get_system().SetRandomContext(context,
                                                             self.generator)

        # Note: The output port will be evaluated without fixing the input
        # port.
        observations = self.observation_port.Eval(context)
        return observations, dict()

    def render(self):
        """
        Rendering in ``human`` mode is accomplished by calling ForcedPublish on
        ``system``.  This should cause visualizers inside the System (e.g.
        MeshcatVisualizer, PlanarSceneGraphVisualizer, etc.) to draw their
        outputs.  To be fully compliant, those visualizers should set their
        default publishing period to ``np.inf`` (do not publish periodically).

        Rendering in ``ascii`` mode calls ``__repr__`` on the system Context.

        Rendering in ``rgb_array`` mode is enabled by passing a compatible
        ``render_rgb_port`` to the class constructor.
        """
        assert self.simulator, "You must call reset() first"

        if self.render_mode == 'human':
            self.simulator.get_system().ForcedPublish(
                self.simulator.get_context())
            return
        elif self.render_mode == 'ansi':
            return __repr__(self.simulator.get_context())
        elif self.render_mode == 'rgb_array':
            assert self.render_rgb_port, \
                "You must set render_rgb_port in the constructor"
            return self.render_rgb_port.Eval(
                self.simulator.get_context()).data[:, :, :3]
        else:
            super(DrakeGymEnv).render()
