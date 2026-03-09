import copy
import dataclasses
import gc
import unittest
import weakref

from pydrake.systems.framework import DiagramBuilder, System


@dataclasses.dataclass
class PortMonitor:
    """Mock-up of some bespoke user code that would monitor ports during a
    simulation."""

    system: System | None = None
    ports: list | None = None


def _create_port_monitor_diagram(system, ports):
    """Prepares a diagram to provoke a lifetime hazard like #22515."""
    port_monitor = PortMonitor(system, ports)
    builder = DiagramBuilder()
    builder.AddSystem(system)
    diagram = builder.Build()
    diagram.port_monitor = port_monitor
    return diagram


def check_ports_lifetime_hazard(
    test: unittest.TestCase, dut: list[System], ports: list
):
    """Checks that the list of 'ports', when stored as metadata associated with
    a Diagram, does not interfere with garbage collection.

    The `test` is used to report pass/fail.

    The `dut` must be *list* containing a single System. (We need to pass it as
    a list so that we can clear the caller's local variable reference to it.)

    The `ports` is a list of one or more input or output ports from the `dut`.
    """
    # Take ownership of `dut` (now named `system`).
    assert isinstance(dut, list)
    (system,) = dut
    assert isinstance(system, System)
    dut[:] = []

    # Take ownership of `ports` (now named `my_ports`).
    assert isinstance(ports, list)
    assert len(ports) > 0
    my_ports = copy.copy(ports)
    ports[:] = []

    # Spy on the lifetime of `system`.
    system_spy = weakref.finalize(system, lambda: None)

    # Add `system` to a diagram, pushing ownership of the list of `system` and
    # `my_ports` down into the diagram.
    diagram = _create_port_monitor_diagram(system, my_ports)
    del system, my_ports

    # Sanity check that the system is still alive.
    gc.collect()
    test.assertTrue(system_spy.alive)

    # Releasing the diagram should be enough to release the system and ports.
    del diagram
    gc.collect()
    test.assertFalse(system_spy.alive)
