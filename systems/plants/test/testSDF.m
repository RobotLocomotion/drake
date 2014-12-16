function testSDF

setenv('GAZEBO_MODEL_PATH','/Users/russt/tmp/gazebo_models'); % note: only works here because the path won't have been set somewhere else

r = RigidBodyManipulator('FallingBrickBetterCollisionGeometry.urdf',struct('floating',true));
r = r.addRobotFromSDF('/Users/russt/tmp/gazebo_models/cinder_block_2/model.sdf',[2;2;0]);

v = r.constructVisualizer();
v.inspector()
