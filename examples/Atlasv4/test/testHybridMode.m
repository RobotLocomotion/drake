% NOTEST
function testHybridMode

options.floating = true;
options.view = 'right';
s = warning('off','Drake:PlanarRigidBodyManipulator:RemovedJoint');
m = PlanarRigidBodyManipulator('../urdf/atlas_robot_minimal_contact.urdf',options);
warning(s);
p = HybridRigidBodyMode(m,zeros(m.featherstone.NB,1),[1;zeros(3,1)]);

xtraj = simulate(p,[0 4]);
v = p.constructVisualizer();
v.playback(xtraj);