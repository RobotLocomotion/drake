function testSDF

r = RigidBodyManipulator('FallingBrickBetterCollisionGeometry.urdf',struct('floating',true));
r.addRobotFromSDF('/Users/russt/tmp/gazebo_models/cinder_block_2/model.sdf',[2;2;0]);

v = r.constructVisualizer();
v.inspector()
