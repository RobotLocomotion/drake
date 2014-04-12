function doublePendWBiceptSpring

r = RigidBodyManipulator('DoublePendWBiceptSpring.urdf',struct('terrain',[]));
x0 = [0;pi;0;0]+.1*randn(4,1);
run_time = 10;
xtraj = simulate(r,[0 run_time],x0);
v = r.constructVisualizer();
v.playback(xtraj);

xf = xtraj.eval(xtraj.tspan(end));
%valuecheck(xf(2),pi,.2);  % strong enough spring that we expect it to stay in the collapsed state
% note: this currently fails.  and i suspect it's a real bug.  - Russ

end

