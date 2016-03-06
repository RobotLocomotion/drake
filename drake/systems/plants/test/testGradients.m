function testGradients

options.grad_method = {'user','taylorvar'};

p = PlanarRigidBodyManipulator('../../../examples/Acrobot/Acrobot.urdf');
for i=1:25
  t = rand;
  x = rand(4,1);
  u = rand;
  
  [xdot,dxdot] = geval(@p.dynamics,t,x,u,options);
end

p = RigidBodyManipulator('../../../examples/FurutaPendulum/FurutaPendulum.urdf');
for i=1:25
  t = rand;
  x = rand(4,1);
  u = rand;
  
  [xdot,dxdot] = geval(@p.dynamics,t,x,u,options);
end

p = RigidBodyManipulator('FallingBrick.urdf',struct('floating','RPY'));
for i=1:25
  t = rand;
  x = rand(12,1);
  u = [];
  
%  [H,C,B,dH,dC,dB] = geval(3,@p.manipulatorDynamics,x(1:6),x(7:12),options);
  
  [xdot,dxdot] = geval(@p.dynamics,t,x,u,options);
end

p = RigidBodyManipulator('FallingBrick.urdf',struct('floating','rpy'));
for i=1:100
  t = rand;
  x = rand(12,1);
  u = [];
  
%  [H,C,B,dH,dC,dB] = geval(3,@p.manipulatorDynamics,x(1:6),x(7:12),options);

  [xdot,dxdot] = geval(@p.dynamics,t,x,u,options);
end

