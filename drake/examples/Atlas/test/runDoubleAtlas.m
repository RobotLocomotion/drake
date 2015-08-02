function runDoubleAtlas
% just runs it as a passive system

options.view = 'right';
options.twoD = true;
options.terrain = RigidBodyFlatTerrain();

dt = 0.005;
r = TimeSteppingRigidBodyManipulator([],dt,options);

options.floating = true;
%s = 'urdf/simple_atlas_minimal_contact.urdf';
s = '../urdf/atlas_minimal_contact.urdf';
r = addRobotFromURDF(r,s,[0;0;0],[0;0;0],options);
options.namesuffix = 'rotated';

r = addRobotFromURDF(r,s,[1;0;0],[0;0;0],options);
r = r.removeCollisionGroupsExcept({'toe','heel'});
r = compile(r);

%options.viewer = 'BotVisualizer';
v = r.constructVisualizer(options);
v.display_dt = 0.02;

% Run simulation, then play it back at realtime speed
[ytraj,xtraj] = simulate(r,[0 3]);
v.playback(xtraj);
