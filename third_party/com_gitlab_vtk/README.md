# Description

This uses the VTK implementation of `vtkTexturedSphereSource` as a base to
create a `vtkTexturedCapsuleSource`.

Replaces the `vtkCapsuleSource` with `vtkTexturedCapsuleSource`. This
implementation contains texture coordinates for the capsule. It also changes
the wireframe construction of the capsule. Now a single sphere is used where
the poles are parallel to the cylinder body. The `vtkCapsuleSource` has two
spheres with their poles parallel to the normal of the cylinder body. This
change enables adding texture to the capsule body.

Please refer to [VTK MR #9828](https://gitlab.kitware.com/vtk/vtk/-/merge_requests/9828) for additional details.
