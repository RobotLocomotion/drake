% test KinematicTrajectoryOptimization_constructor

atlas_urdf = fullfile(getDrakePath(),'examples','Atlas','urdf','atlas_convex_hull.urdf');
robot = RigidBodyManipulator(atlas_urdf);

tc = testClass(robot);