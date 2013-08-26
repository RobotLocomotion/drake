function [p, v, x0, xtraj] = testPlane()
%{
length(q) = 5, 10 total states.
[X pos
 Z pos
 Theta (negative = pitch up
 Elevator angle
 Leg angle]
%}
disp('constructing a Plane')
options.floating = true;
p = RigidBodyManipulator('Plane.URDF', options);
%    [X  Y  Z  Rx Ry Rz WL WR EL Rd Vx Vy Vz Vr Vp Vy Velev
x0 = [0  0  1  0  0  0  0  0  0  0  6  0  0  0  0  0  0 0 0 0]';
disp('RBM constructed')
xtraj = p.simulate([0 .3],x0);
disp('simulation finished. constructing visualizer')
v = p.constructVisualizer();
v.axis = [-6 1 -2 1];
v.playback_speed = .25;
disp('playing back trajectory.')
v.playback(xtraj);
%Note: this final state vector only holds for a specific test case:
%Flat plate, mass = .1, chord = .4, span = 1, origins coincident, nominal
%speed = 10,
%finalstate = [6 0 2.6831 0 0 0 6 0 -0.3278 0 0 0]';
%assert(norm(finalstate-xtraj.eval(1))<.01, 'Failed Wing test. Please check the parameters of your testWing.xml file, this could cause a false negative!');
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
