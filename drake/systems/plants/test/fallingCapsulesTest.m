function fallingCapsulesTest

options.floating = 'quat';
options.terrain = RigidBodyFlatTerrain();
N = 5;
p = TimeSteppingRigidBodyManipulator('Capsule.urdf',.001,options);
for i=2:N,
  options.namesuffix = num2str(N);
  p = p.addRobotFromURDF('Capsule.urdf',[],[],options);
end

x0 = .2*randn(p.getNumStates,1);
for i=1:N
  x0(7*(i-1)+3) = i;  % z coordinate
  x0(7*(i-1)+(4:7)) = uniformlyRandomQuat();
end

v = p.constructVisualizer();
v.drawWrapper(0,x0);

x0 = p.resolveConstraints(x0,v);
v.drawWrapper(0,x0);

traj = simulate(p,[0 2],x0);
v.playback(traj); %,struct('slider',true));
% function collisionDetectGradTest(visualize,n_debris)
%   if nargin < 1
%     visualize = false;
%   end
%   if nargin < 2
%     n_debris = 5;
%   end
%   n_debris = 5;
%   visualize = true;
%   options.floating = true;
%   S = warning('OFF','Drake:RigidBodyManipulator:UnsupportedContactPoints');
%   warning('OFF','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
%   r = RigidBodyManipulator('Cylinder.urdf',options);
%   for i = 1:n_debris
%     r = r.addRobotFromURDF('FallingBrick.urdf',3*(2*rand(3,1)-1),2*pi*rand(3,1));
%     r = r.addRobotFromURDF('ball.urdf',3*(2*rand(3,1)-1),2*pi*rand(3,1));
%     r = r.addRobotFromURDF('Cylinder.urdf',3*(2*rand(3,1)-1),2*pi*rand(3,1));
%   end
%   r = r.addRobotFromURDF('ground_plane.urdf');
%   warning(S);
%
%   if visualize
%     v = r.constructVisualizer();
%   end
%
%   x0 = r.getInitialState;
%   x0 = r.resolveConstraints(x0);
% end
