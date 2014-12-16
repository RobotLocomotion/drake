function testSDF

setenv('GAZEBO_MODEL_PATH',['/Users/russt/tmp/gazebo_models',pathsep,'/Users/russt/tmp/drcsim/drcsim_model_resources/gazebo_models']); % note: only works here because the path won't have been set somewhere else

r = RigidBodyManipulator('FallingBrickBetterCollisionGeometry.urdf',struct('floating',true));
%r = r.addRobotFromSDF('/Users/russt/tmp/gazebo_models/cinder_block_2/model.sdf',[2;2;0]);
r = r.addRobotFromSDF('/Users/russt/tmp/drcsim/drcsim_model_resources/worlds/drc_practice_task_2.world',[0;0;0]);

v = r.constructVisualizer();
x0 = Point(r.getStateFrame());
x0.base_z = 10;
v.inspector(x0)
