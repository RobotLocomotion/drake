import numpy as np

from pydrake.common.value import AbstractValue
from pydrake.math import RigidTransform
from pydrake.perception import BaseField, Fields, PointCloud
from pydrake.systems.framework import LeafSystem


def _TransformPoints(points_Ci, X_CiSi):
    # Make homogeneous copy of points.
    points_h_Ci = np.vstack((points_Ci,
                             np.ones((1, points_Ci.shape[1]))))

    return X_CiSi.dot(points_h_Ci)[:3, :]


def _TileColors(color, dim):
    # Need manual broadcasting.
    return np.tile(np.array([color]).T, (1, dim))


def _ConcatenatePointClouds(points_dict, colors_dict):
    scene_points = None
    scene_colors = None

    for id in points_dict:
        if scene_points is None:
            scene_points = points_dict[id]
        else:
            scene_points = np.hstack((points_dict[id], scene_points))

        if scene_colors is None:
            scene_colors = colors_dict[id]
        else:
            scene_colors = np.hstack((colors_dict[id], scene_colors))

    valid_indices = np.logical_not(np.isnan(scene_points))

    scene_points = scene_points[:, valid_indices[0, :]]
    scene_colors = scene_colors[:, valid_indices[0, :]]

    return scene_points, scene_colors


class PointCloudConcatenation(LeafSystem):
    """
    .. pydrake_system::

        name: PointCloudConcatenation
        input_ports:
        - point_cloud_CiSi_id0
        - X_FCi_id0
        - ...
        - point_cloud_CiSi_idN
        - X_FCi_idN
        output_ports:
        - point_cloud_FS
    """

    def __init__(self, id_list, default_rgb=[255., 255., 255.]):
        """
        A system that takes in N point clouds of points Si in frame Ci, and N
        RigidTransforms from frame Ci to F, to put each point cloud in a common
        frame F. The system returns one point cloud combining all of the
        transformed point clouds. Each point cloud must have XYZs. RGBs are
        optional. If absent, those points will be the provided default color.

        @param id_list A list containing the string IDs of all of the point
            clouds. This is often the serial number of the camera they came
            from, such as "1" for a simulated camera or "805212060373" for a
            real camera.
        @param default_rgb A list of length 3 containing the RGB values to use
            in the absence of PointCloud.rgbs. Values should be between 0 and
            255. The default is white.
        """
        LeafSystem.__init__(self)

        self._point_cloud_ports = {}
        self._transform_ports = {}

        self._id_list = id_list

        self._default_rgb = np.array(default_rgb)

        output_fields = Fields(BaseField.kXYZs | BaseField.kRGBs)

        for id in self._id_list:
            self._point_cloud_ports[id] = self.DeclareAbstractInputPort(
                "point_cloud_CiSi_{}".format(id),
                AbstractValue.Make(PointCloud(fields=output_fields)))

            self._transform_ports[id] = self.DeclareAbstractInputPort(
                "X_FCi_{}".format(id),
                AbstractValue.Make(RigidTransform.Identity()))

        self.DeclareAbstractOutputPort("point_cloud_FS",
                                       lambda: AbstractValue.Make(
                                           PointCloud(fields=output_fields)),
                                       self.DoCalcOutput)

    def _AlignPointClouds(self, context):
        points = {}
        colors = {}

        for id in self._id_list:
            point_cloud = self.EvalAbstractInput(
                context, self._point_cloud_ports[id].get_index()).get_value()
            X_CiSi = self.EvalAbstractInput(
                context, self._transform_ports[id].get_index()).get_value()

            points[id] = _TransformPoints(
                point_cloud.xyzs(), X_CiSi.GetAsMatrix4())

            if point_cloud.has_rgbs():
                colors[id] = point_cloud.rgbs()
            else:
                colors[id] = _TileColors(
                    self._default_rgb, point_cloud.xyzs().shape[1])

        return _ConcatenatePointClouds(points, colors)

    def DoCalcOutput(self, context, output):
        scene_points, scene_colors = self._AlignPointClouds(context)

        output.get_mutable_value().resize(scene_points.shape[1])
        output.get_mutable_value().mutable_xyzs()[:] = scene_points
        output.get_mutable_value().mutable_rgbs()[:] = scene_colors
