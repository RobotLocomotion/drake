function springPendulum

r = RigidBodyManipulator('SpringPendulum.urdf');
x0 = [-pi/2;0];
run_time = 10;
xtraj = simulate(r,[0 run_time],x0);
v = r.constructVisualizer();
v.playback(xtraj);

end

% TIMEOUT 1500
