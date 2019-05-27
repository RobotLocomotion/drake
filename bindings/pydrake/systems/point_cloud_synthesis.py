import numpy as np

from pydrake.math import RigidTransform
from pydrake.systems.framework import AbstractValue, DiagramBuilder, LeafSystem
import pydrake.perception as mut


class PointCloudSynthesis(LeafSystem):

    def __init__(self, id_list, default_rgb=[255., 255., 255.]):
        """
        A system that takes in N point clouds and N RigidTransforms that
        put each point cloud in world frame. The system returns one point cloud
        combining all of the transformed point clouds. Each point cloud must
        have XYZs. RGBs are optional. If absent, those points will be the
        provided default color.

        @param id_list A list containing the IDs of all of the point clouds.
            This is often the serial number of the camera they came from.
        @param default_rgb A list of length 3containing the RGB values to use
            in the absence of PointCloud.rgbs. Values should be between 0 and
            255. The default is white.

        @system{
          @input_port{point_cloud_id0}
          @input_port{X_WP_id0}
          .
          .
          .
          @input_port{point_cloud_P_idN}
          @input_port{X_WP_idN}
          @output_port{combined_point_cloud_W}
        }
        """
        LeafSystem.__init__(self)

        self.point_cloud_ports = {}
        self.transform_ports = {}

        self.id_list = id_list

        self._default_rgb = np.array(default_rgb)

        output_fields = mut.Fields(mut.BaseField.kXYZs | mut.BaseField.kRGBs)

        for id in self.id_list:
            self.point_cloud_ports[id] = self.DeclareAbstractInputPort(
                "point_cloud_P_{}".format(id),
                AbstractValue.Make(mut.PointCloud(fields=output_fields)))

            self.transform_ports[id] = self.DeclareAbstractInputPort(
                "X_WP_{}".format(id),
                AbstractValue.Make(RigidTransform.Identity()))

        self.DeclareAbstractOutputPort("combined_point_cloud_W",
                                       lambda: AbstractValue.Make(
                                           mut.PointCloud(
                                               fields=output_fields)),
                                       self.DoCalcOutput)

    def _AlignPointClouds(self, context):
        points = {}
        colors = {}

        for id in self.id_list:
            point_cloud = self.EvalAbstractInput(
                context, self.point_cloud_ports[id].get_index()).get_value()
            X_WP = self.EvalAbstractInput(
                context, self.transform_ports[id].get_index()).get_value()

            # Make a homogenous version of the points.
            points_h_P = np.vstack((point_cloud.xyzs(),
                                    np.ones((1, point_cloud.xyzs().shape[1]))))

            points[id] = X_WP.matrix().dot(points_h_P)[:3, :]

            if point_cloud.has_rgbs():
                colors[id] = point_cloud.rgbs()
            else:
                # Need manual broadcasting.
                colors[id] = np.tile(np.array([self._default_rgb]).T,
                                     (1, points[id].shape[1]))

        # Combine all the points and colors into two arrays.
        scene_points = None
        scene_colors = None

        for id in points:
            if scene_points is None:
                scene_points = points[id]
            else:
                scene_points = np.hstack((points[id], scene_points))

            if scene_colors is None:
                scene_colors = colors[id]
            else:
                scene_colors = np.hstack((colors[id], scene_colors))

        valid_indices = np.logical_not(np.isnan(scene_points))

        scene_points = scene_points[:, valid_indices[0, :]]
        scene_colors = scene_colors[:, valid_indices[0, :]]

        return scene_points, scene_colors

    def DoCalcOutput(self, context, output):
        scene_points, scene_colors = self._AlignPointClouds(context)

        output.get_mutable_value().resize(scene_points.shape[1])
        output.get_mutable_value().mutable_xyzs()[:] = scene_points
        output.get_mutable_value().mutable_rgbs()[:] = scene_colors
