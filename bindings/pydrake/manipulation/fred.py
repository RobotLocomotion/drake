"""
Offers a Meshcat vizualization server that listens for and draws Drake's
legacy LCM vizualization messages.

From a Drake source build, run this as:

  bazel run //bindings/pydrake/manipulation:fred

or

  bazel build //bindings/pydrake/manipulation:fred
  bazel-bin/bindings/pydrake/manipulation/fred &

From a Drake binary release, run this as:

  python3 -m pydrake.manipulation.fred
"""

import logging
import numpy as np

from drake import (
    lcmt_contact_results_for_viz,
    lcmt_viewer_draw,
    lcmt_viewer_geometry_data,
    lcmt_viewer_load_robot,
)
from pydrake.common.eigen_geometry import (
    Quaternion,
)
from pydrake.common.value import (
    AbstractValue,
)
from pydrake.geometry import (
    Box,
    Capsule,
    Cylinder,
    Ellipsoid,
    Mesh,
    Meshcat,
    MeshcatCone,
    Rgba,
    Sphere,
)
from pydrake.lcm import (
    DrakeLcm,
)
from pydrake.math import (
    RigidTransform,
    RotationMatrix,
)


class Fred:
    """
    Offers a Meshcat vizualization server that listens for and draws Drake's
    legacy LCM vizualization messages.
    """

    def __init__(self):
        self._meshcat = Meshcat()

        self._lcm = DrakeLcm()
        lcm_url = self._lcm.get_lcm_url()
        logging.info(f"Listening for LCM messages at {lcm_url}")

        self._subscribe(channel="DRAKE_VIEWER_LOAD_ROBOT",
                        message_type=lcmt_viewer_load_robot,
                        handler=self._on_viewer_load)
        self._subscribe(channel="DRAKE_VIEWER_DRAW",
                        message_type=lcmt_viewer_draw,
                        handler=self._on_viewer_draw)
        self._subscribe(channel="CONTACT_RESULTS",
                        message_type=lcmt_contact_results_for_viz,
                        handler=self._on_contact_results)

        # Used to hide the link geometry between LOAD and DRAW handling,
        # so that we don't display a bucket of parts at the origin.
        self._links_are_visible = False

        # Used to track which contact pairs have illustration geometry already
        # added to the server, and whether or not its visible.
        self._contact_is_visible = dict()

        # By default, don't show contact illustrations.
        self._meshcat.SetProperty("/lcm_contact_forces", "visible", False)

    def _subscribe(self, channel, message_type, handler):
        def _parse_and_handle(data):
            handler(message=message_type.decode(data))
        self._lcm.Subscribe(channel=channel, handler=_parse_and_handle)

    def serve_forever(self):
        while True:
            self._lcm.HandleSubscriptions(timeout_millis=1000)

    def _on_viewer_load(self, message):
        self._meshcat.Delete(path="/lcm_viewer")
        self._meshcat.SetProperty("/lcm_viewer", "visible", False)
        self._links_are_visible = False
        for i, link in enumerate(message.link):
            robot_num = link.robot_num
            link_name = link.name
            base_path = f"/lcm_viewer/{robot_num}/{link_name}"
            for j, geom in enumerate(link.geom):
                path = f"{base_path}/{j}"
                object_kwargs, pose = self._convert_geom(geom)
                if object_kwargs is None:
                    logging.warning(f"Cannot convert {path} for {link.name}")
                    continue
                self._meshcat.SetObject(path=path, **object_kwargs)
                self._meshcat.SetTransform(path=path, X_ParentPath=pose)

    def _convert_geom(self, geom):
        shape = None
        if geom.type == lcmt_viewer_geometry_data.BOX:
            (width, depth, height) = geom.float_data
            shape = Box(width=width, depth=depth, height=height)
        elif geom.type == lcmt_viewer_geometry_data.CAPSULE:
            (radius, length) = geom.float_data
            shape = Capsule(radius=radius, length=length)
        elif geom.type == lcmt_viewer_geometry_data.CYLINDER:
            (radius, length) = geom.float_data
            shape = Cylinder(radius=radius, length=length)
        elif geom.type == lcmt_viewer_geometry_data.ELLIPSOID:
            (a, b, c) = geom.float_data
            shape = Ellipsoid(a=a, b=b, c=c)
        elif geom.type == lcmt_viewer_geometry_data.MESH:
            (scale_x, scale_y, scale_z) = geom.float_data
            filename = geom.string_data
            assert scale_x == scale_y and scale_y == scale_z
            shape = Mesh(absolute_filename=filename, scale=scale_x)
        elif geom.type == lcmt_viewer_geometry_data.SPHERE:
            (radius,) = geom.float_data
            shape = Sphere(radius=radius)
        else:
            return (None, None)
        rgba = Rgba(*geom.color)
        object_kwargs = {
            "shape": shape,
            "rgba": rgba,
        }
        pose = self._to_pose(geom.position, geom.quaternion)
        return (object_kwargs, pose)

    @staticmethod
    def _to_pose(position, quaternion):
        (p_x, p_y, p_z) = position
        (q_w, q_x, q_y, q_z) = quaternion
        return RigidTransform(
            R=RotationMatrix(
                quaternion=Quaternion(
                    w=q_w,
                    x=q_x,
                    y=q_y,
                    z=q_z,
                ),
            ),
            p=(p_x, p_y, p_z))

    def _on_viewer_draw(self, message):
        for i in range(message.num_links):
            link_name = message.link_name[i]
            robot_num = message.robot_num[i]
            base_path = f"/lcm_viewer/{robot_num}/{link_name}"
            pose = self._to_pose(message.position[i], message.quaternion[i])
            self._meshcat.SetTransform(path=base_path, X_ParentPath=pose)
        if not self._links_are_visible:
            self._meshcat.SetProperty("/lcm_viewer", "visible", True)
            self._links_are_visible = True

    def _on_contact_results(self, message):
        for item in message.point_pair_contact_info:
            self._on_point_contact_result(item)

    def _on_point_contact_result(self, message):
        path = "/lcm_contact_forces/{}+{}".format(
            message.body1_name, message.body2_name)

        contact_force = np.array(message.contact_force)
        force_norm = np.linalg.norm(contact_force)
        force_threshold = 0.01
        should_be_visible = force_norm > force_threshold
        meshcat_geometry_already_exists = path in self._contact_is_visible

        # Hide the old contact illustration, if necessary.
        if not should_be_visible:
            if meshcat_geometry_already_exists:
                self._meshcat.SetProperty(path, "visible", False)
                self._contact_is_visible[path] = False
            return

        # Create or unhide the illustration.
        if not meshcat_geometry_already_exists:
            self._add_contact_illustration(path=path)
        elif not self._contact_is_visible[path]:
            self._meshcat.SetProperty(path, "visible", True)
            self._contact_is_visible[path] = True

        # Stretch the cylinder in z.
        newtons_per_meter = 10.0
        height = force_norm / newtons_per_meter
        self._meshcat.SetTransform(
            path=f"{path}/cylinder",
            matrix=np.diag(np.array([1, 1, height, 1])))

        # Pose the illustration.
        b_A = np.array(message.contact_force)
        R = RotationMatrix.MakeFromOneVector(b_A=b_A, axis_index=2)
        p = np.array(message.contact_point)
        self._meshcat.SetTransform(path, RigidTransform(R=R, p=p))

    def _add_contact_illustration(self, *, path):
        self._meshcat.SetObject(
            path=f"{path}/cylinder",
            shape=Cylinder(0.01, 2.0),
            rgba=Rgba(0, 1, 0, 1))
        self._contact_is_visible[path] = True


def _main():
    logging.basicConfig(level=logging.INFO)
    fred = Fred()
    fred.serve_forever()


if __name__ == "__main__":
    _main()
