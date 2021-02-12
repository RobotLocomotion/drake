# Note that this script runs in the main context of drake-visualizer,
# where many modules and variables already exist in the global scope.

from director import lcmUtils
from director import applogic
from director import objectmodel as om
from director import visualization as vis
import director.vtkAll as vtk

import drake as lcmdrakemsg

from _drake_visualizer_builtin_scripts import scoped_singleton_func


class ExperimentalDeformableMeshVisualizer:
    def __init__(self):
        self._folder_name = "DeformableMesh"
        self._name = "Deformable Mesh Visualizer"
        self._enabled = False
        # The subscribers: one for init and one for update.
        self._subs = None
        self._names = []
        self._vertex_counts = []
        self._poly_data_list = []
        self._poly_item_list = []

        self.set_enabled(True)

    def add_subscriber(self):
        if self._subs is not None:
            return

        self._subs = []
        self._subs.append(
            lcmUtils.addSubscriber(
                "DEFORMABLE_MESHES_INIT",
                messageClass=lcmdrakemsg.
                experimental_lcmt_deformable_tri_meshes_init,
                callback=self.handle_init_message,
            )
        )

        self._subs.append(
            lcmUtils.addSubscriber(
                "DEFORMABLE_MESHES_UPDATE",
                messageClass=lcmdrakemsg.
                experimental_lcmt_deformable_tri_meshes_update,
                callback=self.handle_update_message,
            )
        )

        print(self._name + " subscribers added.")

    def remove_subscriber(self):
        if self._subs is None:
            return

        for sub in self._subs:
            lcmUtils.removeSubscriber(sub)
        self._subs = None
        print(self._name + " subscribers removed.")

    def is_enabled(self):
        return self._enabled

    def set_enabled(self, enable, clear=True):
        print("DeformableVisualizer set_enabled", enable)
        self._enabled = enable
        if enable:
            self.add_subscriber()
        else:
            self.remove_subscriber()
            # Removes the folder completely and resets the known meshes.
            om.removeFromObjectModel(
                om.findObjectByName(self._folder_name)
            )
            self._poly_item_list = []

    def handle_init_message(self, msg):
        """Creates the polydata for the deformable mesh specified in msg."""
        folder = om.getOrCreateContainer(self._folder_name)
        for poly_item in self._poly_item_list:
            om.removeFromObjectModel(poly_item)

        self._names = []
        self._vertex_counts = []
        self._poly_data_list = []
        self._poly_item_list = []
        for mesh in msg.meshes:
            self._names.append(mesh.name)
            # Initial vertex positions are garbage; meaningful values will be
            # set by the update message.
            points = vtk.vtkPoints()
            self._vertex_counts.append(mesh.num_vertices)
            for i in range(mesh.num_vertices):
                points.InsertNextPoint(0, 0, 0)
            triangles = vtk.vtkCellArray()
            for tri in mesh.tris:
                triangles.InsertNextCell(3)
                for i in tri.vertices:
                    triangles.InsertCellPoint(i)
            poly_data = vtk.vtkPolyData()
            poly_data.SetPoints(points)
            poly_data.SetPolys(triangles)
            poly_item = vis.showPolyData(
                poly_data, mesh.name, parent=folder
            )
            poly_item.setProperty("Surface Mode", "Surface with edges")
            self._poly_data_list.append(poly_data)
            self._poly_item_list.append(poly_item)

    def handle_update_message(self, msg):
        """Updates vertex data for the deformable meshes specified in msg."""
        if not len(self._poly_data_list) == msg.num_meshes:
            print(
                "Received a deformable mesh update message with '{}' meshes; "
                "expected {} meshes.".format(
                    msg.num_meshes, len(self._poly_data_list)
                )
            )
            return
        for mesh_id in range(msg.num_meshes):
            mesh = msg.meshes[mesh_id]
            if mesh.name != self._names[mesh_id]:
                print(
                    "The deformable mesh update message contains data for "
                    "a mesh named '{}', expected name '{}'.".format(
                        mesh.name, self._names[mesh_id]
                    )
                )
                return
            if mesh.data_size != self._vertex_counts[mesh_id] * 3:
                print(
                    "The deformable mesh update message contains data for {} "
                    "vertices; expected {}.".format(
                        mesh.data_size // 3, self._vertex_count
                    )
                )
                return
            points = vtk.vtkPoints()
            i = 0
            while i < mesh.data_size:
                points.InsertNextPoint(
                    mesh.data[i], mesh.data[i + 1], mesh.data[i + 2]
                )
                i = i + 3
            # TODO(SeanCurtis-TRI): Instead of creating a new set of points and
            # stomping on the old; can I just update the values? That might
            # improve performance.
            self._poly_data_list[mesh_id].SetPoints(points)
            self._poly_item_list[mesh_id].setPolyData(
                self._poly_data_list[mesh_id]
            )
            self._poly_item_list[mesh_id]._renderAllViews()


@scoped_singleton_func
def init_visualizer():
    # Create a visualizer instance.
    my_visualizer = ExperimentalDeformableMeshVisualizer()
    # Adds to the "Tools" menu.
    applogic.MenuActionToggleHelper(
        "Tools",
        my_visualizer._name,
        my_visualizer.is_enabled,
        my_visualizer.set_enabled,
    )
    return my_visualizer


# Activate the plugin if this script is run directly; store the results to keep
# the plugin objects in scope.
if __name__ == "__main__":
    deform_viz = init_visualizer()
