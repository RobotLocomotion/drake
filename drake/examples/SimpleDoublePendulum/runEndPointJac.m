function runEndPointJac

r = PlanarRigidBodyManipulator('SimpleDoublePendulum.urdf');
v = r.constructVisualizer();
v.axis = [-2 2 -2 2];

kp = diag([20 20]);
kd = diag([4 4]);
sys = pdcontrol(r,kp,kd);

c = EndPointControl(sys,r);

xtraj = simulate(feedback(sys,c),[0 3]);
v.playback(xtraj);

