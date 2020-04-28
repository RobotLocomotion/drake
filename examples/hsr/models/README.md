This directory contains a drake-compatible description of the HSRB.

The URDF and mesh files of HSR can be found in ToyotaResearchInsitute's open source github

    $ git clone https://github.com/ToyotaResearchInstitute//hsr_description.git

    $ git clone https://github.com/ToyotaResearchInstitute//hsr_meshes.git


 Since the default urdf files are made for the Gazebo simulator, several changes have
 to be made to make the urdf compatible with Drake.
 - All the Gazebo specific tags are removed.
 - All the non-actuated joints are removed, except for the fixed ones. This yields
    a fully actuated robot if the robot base is welded to the ground.
 - Drake only supports primitives and convex objs for collision geometries. If a
    certain collision obj file is not convex, we have run a automatic convex
    decomposition procedure to generate convex objs for the collision geometry
    for that part. The auto decomposition procedure can be referred to [trimesh](https://github.com/mikedh/trimesh)
    and [v-hacd](https://github.com/kmammou/v-hacd).

The resulted file is hsrb4s_simplified.urdf. The original version hsrb4s.obj.urdf is still kept here for reference.