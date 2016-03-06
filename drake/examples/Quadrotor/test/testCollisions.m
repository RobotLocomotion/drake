function testCollisions

tmp = addpathTemporary(fullfile(pwd,'..'));

r = Quadrotor('','quat');
r = r.setTerrain([]);
r = addTrees(r, 25);
% The important trees to create swerving path
r = addTree(r, [.8,.45,1.25], [.20;2.5], pi/4);
r = addTree(r, [.5,.35,1.65], [-.25;5], -pi/6);
r = addTree(r, [.55,.65,1.5], [.25;7.5], pi/4);
r = addTree(r, [.55,.85,1.6], [-1.35;8.5], pi/3.7);
r = addTree(r, [.85,.95,1.65], [-1.85;5.2], -pi/3.7);
r = addTree(r, [.75,.9,1.75], [2;4.4], -pi/5);

x0 = Point(getStateFrame(r));  % initial conditions: all-zeros
x0.base_y = -1.5;
x0.base_z = .8;
x0.base_qw = 1;
x0.base_vy = 5;
x0 = resolveConstraints(r,x0);
u0 = double(nominalThrust(r));

v = constructVisualizer(r);%,struct('use_collision_geometry',true));
v.draw(0,double(x0));

utraj = setOutputFrame(ConstantTrajectory(u0),r.getInputFrame);
xtraj = simulate(cascade(utraj,TimeSteppingRigidBodyManipulator(r,.01)),[0 5],x0);
v.playback(xtraj);

