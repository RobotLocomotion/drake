"""
Runs the manipulation_station example with a meshcat joint slider ui for
directly tele-operating the joints.  To have the meshcat server automatically
open in your browser, supply the --open-window flag; the joint sliders will be
accessible by clicking on "Open Controls" in the top right corner.
"""

import argparse
import sys
import webbrowser

import numpy as np

from drake.examples.manipulation_station.schunk_wsg_buttons import \
    SchunkWsgButtons
from pydrake.examples import (
    CreateClutterClearingYcbObjectList, ManipulationStation,
    ManipulationStationHardwareInterface)
from pydrake.geometry import DrakeVisualizer
from pydrake.multibody.meshcat import JointSliders
from pydrake.math import RigidTransform, RotationMatrix
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.analysis import Simulator
from pydrake.geometry import Meshcat, MeshcatVisualizer
from pydrake.systems.primitives import FirstOrderLowPassFilter, VectorLogSink


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--target_realtime_rate", type=float, default=1.0,
        help="Desired rate relative to real time.  See documentation for "
             "Simulator::set_target_realtime_rate() for details.")
    parser.add_argument(
        "--duration", type=float, default=np.inf,
        help="Desired duration of the simulation in seconds.")
    parser.add_argument(
        "--hardware", action='store_true',
        help="Use the ManipulationStationHardwareInterface instead of an "
             "in-process simulation.")
    parser.add_argument(
        "--test", action='store_true',
        help="Disable opening the gui window for testing.")
    parser.add_argument(
        '--setup', type=str, default='manipulation_class',
        help="The manipulation station setup to simulate. ",
        choices=['manipulation_class', 'clutter_clearing', 'planar'])
    parser.add_argument(
        "-w", "--open-window", dest="browser_new",
        action="store_const", const=1, default=None,
        help="Open the MeshCat display in a new browser window.")
    args = parser.parse_args()

    builder = DiagramBuilder()

    # NOTE: the meshcat instance is always created in order to create the
    # teleop controls (joint sliders and open/close gripper button).  When
    # args.hardware is True, the meshcat server will *not* display robot
    # geometry, but it will contain the joint sliders and open/close gripper
    # button in the "Open Controls" tab in the top-right of the viewing server.
    meshcat = Meshcat()

    if args.hardware:
        # TODO(russt): Replace this hard-coded camera serial number with a
        # config file.
        camera_ids = ["805212060544"]
        station = builder.AddSystem(ManipulationStationHardwareInterface(
            camera_ids))
        station.Connect(wait_for_cameras=False)
    else:
        station = builder.AddSystem(ManipulationStation())

        # Initializes the chosen station type.
        if args.setup == 'manipulation_class':
            station.SetupManipulationClassStation()
            station.AddManipulandFromFile(
                "drake/examples/manipulation_station/models/"
                + "061_foam_brick.sdf",
                RigidTransform(RotationMatrix.Identity(), [0.6, 0, 0]))
        elif args.setup == 'clutter_clearing':
            station.SetupClutterClearingStation()
            ycb_objects = CreateClutterClearingYcbObjectList()
            for model_file, X_WObject in ycb_objects:
                station.AddManipulandFromFile(model_file, X_WObject)
        elif args.setup == 'planar':
            station.SetupPlanarIiwaStation()
            station.AddManipulandFromFile(
                "drake/examples/manipulation_station/models/"
                + "061_foam_brick.sdf",
                RigidTransform(RotationMatrix.Identity(), [0.6, 0, 0]))

        station.Finalize()

        geometry_query_port = station.GetOutputPort("geometry_query")
        DrakeVisualizer.AddToBuilder(builder, geometry_query_port)
        meshcat_visualizer = MeshcatVisualizer.AddToBuilder(
            builder=builder,
            query_object_port=geometry_query_port,
            meshcat=meshcat)

        if args.setup == 'planar':
            meshcat.Set2dRenderMode()

    if args.browser_new is not None:
        url = meshcat.web_url()
        webbrowser.open(url=url, new=args.browser_new)

    teleop = builder.AddSystem(JointSliders(
        meshcat=meshcat, plant=station.get_controller_plant()))

    num_iiwa_joints = station.num_iiwa_joints()
    filter = builder.AddSystem(FirstOrderLowPassFilter(
        time_constant=2.0, size=num_iiwa_joints))
    builder.Connect(teleop.get_output_port(0), filter.get_input_port(0))
    builder.Connect(filter.get_output_port(0),
                    station.GetInputPort("iiwa_position"))

    wsg_buttons = builder.AddSystem(SchunkWsgButtons(meshcat=meshcat))
    builder.Connect(wsg_buttons.GetOutputPort("position"),
                    station.GetInputPort("wsg_position"))
    builder.Connect(wsg_buttons.GetOutputPort("force_limit"),
                    station.GetInputPort("wsg_force_limit"))

    # When in regression test mode, log our joint velocities to later check
    # that they were sufficiently quiet.
    if args.test:
        iiwa_velocities = builder.AddSystem(VectorLogSink(num_iiwa_joints))
        builder.Connect(station.GetOutputPort("iiwa_velocity_estimated"),
                        iiwa_velocities.get_input_port(0))
    else:
        iiwa_velocities = None

    diagram = builder.Build()
    simulator = Simulator(diagram)

    # This is important to avoid duplicate publishes to the hardware interface:
    simulator.set_publish_every_time_step(False)

    station_context = diagram.GetMutableSubsystemContext(
        station, simulator.get_mutable_context())

    station.GetInputPort("iiwa_feedforward_torque").FixValue(
        station_context, np.zeros(num_iiwa_joints))

    # If the diagram is only the hardware interface, then we must advance it a
    # little bit so that first LCM messages get processed. A simulated plant is
    # already publishing correct positions even without advancing, and indeed
    # we must not advance a simulated plant until the sliders and filters have
    # been initialized to match the plant.
    if args.hardware:
        simulator.AdvanceTo(1e-6)

    # Eval the output port once to read the initial positions of the IIWA.
    q0 = station.GetOutputPort("iiwa_position_measured").Eval(
        station_context)
    teleop.SetPositions(q0)
    filter.set_initial_output_value(diagram.GetMutableSubsystemContext(
        filter, simulator.get_mutable_context()), q0)

    simulator.set_target_realtime_rate(args.target_realtime_rate)
    simulator.AdvanceTo(args.duration)

    # Ensure that our initialization logic was correct, by inspecting our
    # logged joint velocities.
    if args.test:
        iiwa_velocities_log = iiwa_velocities.FindLog(simulator.get_context())
        for time, qdot in zip(iiwa_velocities_log.sample_times(),
                              iiwa_velocities_log.data().transpose()):
            # TODO(jwnimmer-tri) We should be able to do better than a 40
            # rad/sec limit, but that's the best we can enforce for now.
            if qdot.max() > 0.1:
                print(f"ERROR: large qdot {qdot} at time {time}")
                sys.exit(1)


if __name__ == '__main__':
    main()
