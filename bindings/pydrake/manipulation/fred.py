"""
Offers a Meshcat vizualization server that listens for and draws Drake's
legacy LCM vizualization messages.

From a Drake source build, run this as:

  bazel run //bindings/pydrake/manipulation:fred

From a Drake binary release, run this as:

  python3 -m pydrake.manipulation.fred
"""

import logging
import numpy as np

from drake import (
    lcmt_viewer_draw,
    lcmt_viewer_geometry_data,
    lcmt_viewer_load_robot,
)
from pydrake.common.eigen_geometry import (
    Quaternion,
)
from pydrake.geometry import (
    Box,
    Cylinder,
    Meshcat,
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
        self._lcm = DrakeLcm()
        lcm_url = self._lcm.get_lcm_url()
        logging.info(f"Listening for LCM messages at {lcm_url}")

        self._subscribe(channel="DRAKE_VIEWER_LOAD_ROBOT",
                        message_type=lcmt_viewer_load_robot,
                        handler=self._on_viewer_load)
        self._subscribe(channel="DRAKE_VIEWER_DRAW",
                        message_type=lcmt_viewer_draw,
                        handler=self._on_viewer_draw)

        self._meshcat = Meshcat()

    def _subscribe(self, channel, message_type, handler):
        def _parse_and_handle(data):
            handler(message=message_type.decode(data))
        self._lcm.Subscribe(channel=channel, handler=_parse_and_handle)

    def serve_forever(self):
        while True:
            self._lcm.HandleSubscriptions(timeout_millis=1000)

    def _on_viewer_load(self, message):
        self._meshcat.Delete(path="/lcm_viewer")
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
        elif geom.type == lcmt_viewer_geometry_data.CYLINDER:
            (radius, length) = geom.float_data
            shape = Cylinder(radius=radius, length=length)
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


def _main():
    logging.basicConfig(level=logging.INFO)
    fred = Fred()
    fred.serve_forever()


if __name__ == "__main__":
    _main()
