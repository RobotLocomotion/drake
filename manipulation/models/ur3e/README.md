# ur3e model

The UR3e models were originally imported from <https://github.com/ros-industrial/universal_robot/tree/melodic-devel/ur_description>,
exact commit unknown but probably near
<https://github.com/ros-industrial/universal_robot/commit/c8c27c15579fad1d817cc6cfac7a8e62a3da081d>.
In the first pass, we converted the mesh format from DAE to OBJ. 

High-level changes:

- Collisions are simplified (e.g. spheres) to work better w/ our planning stack
