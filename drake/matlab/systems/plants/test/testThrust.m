function [p,v,x0,xtraj] = testThrust()
% Thrust testing function
run_time = 3;
options.floating = false;
p = PlanarRigidBodyManipulator('testThrust.urdf', options);
%v = p.constructVisualizer();
g = norm(getGravity(p));

x0 = [pi/2 0 0 0]';
%perfectly counteracts gravity if correct.
constOut = [0 g*sqrt(2) g]';
utraj = PPTrajectory(foh([0 10],[constOut constOut]));
utraj = utraj.setOutputFrame(p.getInputFrame());
sys = cascade(utraj,p);
xtraj = simulate(sys,[0 run_time],x0);
v = p.constructVisualizer();
v.axis = [-3 3 -3 3];
%v.playback_speed = .5;
playback(v,xtraj);
valuecheck(xtraj.eval(run_time), x0, .005);



disp('Testing 3D thrust')
options.floating = false;
p = RigidBodyManipulator('testThrust.urdf', options);
%v = p.constructVisualizer();
g = norm(getGravity(p));

x0 = [pi/2 0 0 0]';
%perfectly counteracts gravity if correct.
constOut = [0 g*sqrt(2) g]';
utraj = PPTrajectory(foh([0 10],[constOut constOut]));
utraj = utraj.setOutputFrame(p.getInputFrame());
sys = cascade(utraj,p);
xtraj = simulate(sys,[0 run_time],x0);
v = p.constructVisualizer();
v.playback_speed = .5;
playback(v,xtraj);
valuecheck(xtraj.eval(run_time), x0, .005);
end
