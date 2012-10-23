% NOTEST
function testHybridMode

options.view = 'right';
options.floating = true;
s = warning('off','Drake:PlanarRigidBodyModel:RemovedJoint');
m = PlanarRigidBodyModel('../atlas_robot_minimal_contact.urdf',options);
warning(s);
p = HybridRigidBodyMode(m,zeros(m.featherstone.NB,1),[1;zeros(3,1)]);

xtraj = simulate(p,[0 4]);
v = p.constructVisualizer();
v.playback(xtraj);