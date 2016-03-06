function [p, v, X0, Xtraj] = testWing()
%{
length(q) = 5, 10 total states.
[x pos
 z pos
 Theta (negative = pitch up
 Elevator angle
 Leg angle]
%}
tf = 1;
x0  = -5;
z0 = 0;
pitch0 = -.1;
Vx0 = 6;
Vz0 = .1;
Vpitch0 = -.1;
options.floating = true; %'RPY';
pp = PlanarRigidBodyManipulator('TestWing.urdf', options);
% x0=[x  z Pit Vx Vz Vp]
X0p = [x0 z0 pitch0 Vx0 Vz0 Vpitch0]';
Xtraj = pp.simulate([0 tf],X0p);
vp = pp.constructVisualizer();
vp.axis = [-6 1 -5.05 1.5];
vp.playback_speed = .25;
vp.playback(Xtraj);
xfPlanar=Xtraj.eval(tf);

disp('Testing 3D wing now...');
p = RigidBodyManipulator('TestWing.urdf', options);
% x0=[x  y z  R Pitch  Y Vx Vy Vz Vr Vpit   VY]
X0 = [x0 0 z0 0 pitch0 0 Vx0 0 Vz0 0 Vpitch0 0]';
Xtraj = p.simulate([0 tf],X0);
v = p.constructVisualizer();
v.playback_speed = .25;
v.playback(Xtraj);
xf3D=Xtraj.eval(tf);

xf3Dreduced = [xf3D(1);xf3D(3); xf3D(5); xf3D(7); xf3D(9); xf3D(11)];

%Note: this final state vector only holds for a specific test case:
%Ht23.dat, mass = .13, chord = .4, span = 1, quarter chord at xyz=".1 0 0", 
%nominal speed = 10, stall = 12; iyy = .003648;
nominal3Dstate = [.4766 0 -.0594 0 .2380 0 5.3765 0 -1.2962 0 1.0474 0]';
assert(norm(xfPlanar-xf3Dreduced)<.001, 'Failed Wing test. Identical Planar and 3D cases do not produce identical results');
assert(norm(nominal3Dstate-xf3D)<.01, 'Failed Wing test. Please check the parameters of your testWing.xml file, this could cause a false negative!');
%{%}
end

%{
old runGlider:
function [p, v, x0] = runGlider(wing)

if nargin>0 && strcmpi(wing,'nowing')
     p = PlanarRigidBodyManipulator('glider_nowing.URDF', struct('floating', true, 'view', 'right'));
else
     p = PlanarRigidBodyManipulator('glider.URDF', struct('floating', true, 'view', 'right'));
end
v = p.constructVisualizer();
x0 = [0 0 0 0 -3 0 0 0]';
v.draw(0, x0);

%traj = p.simulate([0 2],x0);
end


Testing TimeSteppingRigidBodyManipulator versus PlanarRigidBodyManipulator
pT = TimeSteppingRigidBodyManipulator('TestWing.URDF',.01, struct('floating', true, 'twoD', true))
x0 = [-3 0 0 0 0 0]'
trajT = pT.simulate([0 1], x0)
vT = pT.constructVisualizer()
vT.playback_speed = .3;
vT.axis= [-5 1 -3 3]
vT.playback(trajT)

p = PlanarRigidBodyManipulator('TestWing.URDF', struct('floating', true, 'view', 'right'))
traj = p.simulate([0 1], x0)
v = p.constructVisualizer()
v.playback_speed = .3
v.axis=[-5 1 -3 3]
v.playback(traj)
%}
