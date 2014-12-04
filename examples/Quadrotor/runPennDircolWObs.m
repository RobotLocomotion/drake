function [utraj,xtraj,prog,r] = runPennDircolWObs

% simple planning demo which takes the quadrotor from hover at x=0m to a new hover at
% x=2m with minimal thrust.


%plant = 'russ';
plant = 'penn';

if plant == 'russ'
  r = Quadrotor();
  v = constructVisualizer(r);
  disp('using russ rotor!')
elseif plant == 'penn'
  r_temp = Quadrotor();
  
  %terrain = RigidBodyFlatTerrain();
  %terrain = terrain.setGeometryColor([0, 170, 255]'/256);
  %r_temp = r_temp.setTerrain(terrain);
  %oquad = Quadrotor();
  %oquad = addRobotFromURDF(oquad, 'office.urdf');
  %v = constructVisualizer(oquad);
  r_temp = addOcean(r_temp, [.8,.45,1.25], [.20;2.5], pi/4);
  %r_temp = addTree(r_temp, [.5,.35,1.65], [-.25;5], -pi/6);
  %r_temp = addTree(r_temp, [.55,.65,1.5], [.25;7.5], pi/4);
  %r_temp = addTree(r_temp, [.55,.85,1.6], [-1.35;8.5], pi/3.7);
  %r_temp = addTree(r_temp, [.85,.95,1.65], [-1.85;5.2], -pi/3.7);
  %r_temp = addTree(r_temp, [.75,.9,1.75], [2;4.4], -pi/5);
  % Random trees to make forest bigger and more dense
  %r_temp = addTrees(r_temp, 25);
  
  
  v = constructVisualizer(r_temp);
  r = QuadWindPlant(); % Quadrotor constructor
  
  % coordinate transform from r.getOutputFrame to v.getInputFrame
  %ct = CoordinateTransform(
  
  % uncomment this to get plotting back
  %r.setOutputFrame(r_temp.getStateFrame());
  
  %v = v.setInputFrame(r.getOutputFrame());
  disp('using quad plant in wind based on penn plant!!')
end


N = 11;
minimum_duration = .1;
maximum_duration = 4;
prog = DircolTrajectoryOptimization(r,N,[minimum_duration maximum_duration]);

x0 = Point(getStateFrame(r));  % initial conditions: all-zeros

if plant == 'russ'
  x0.base_z = .5; % lift the quad off the ground
elseif plant == 'penn'
  x0.z = .5; % lift the quad off the ground
end

v.draw(0,double(x0));
prog = addPlanVisualizer(r,prog);

u0 = double(nominalThrust(r));

prog = prog.addStateConstraint(ConstantConstraint(double(x0)),1); % DirectTrajectoryOptimization method

bd = 10;

seasurface = Point(getStateFrame(r));
seasurface.x = -bd;
seasurface.y = -bd;
seasurface.z = 0.2;
seasurface.roll = -bd;
seasurface.pitch = -bd;
seasurface.yaw = -bd;
seasurface.xdot = -bd;
seasurface.ydot = -bd;
seasurface.zdot = -bd;
seasurface.rolldot = -bd;
seasurface.pitchdot = -bd;
seasurface.yawdot = -bd;
seasurface.mytime = -bd;

world = Point(getStateFrame(r));
world.x = bd;
world.y = bd;
world.z = bd;
world.roll = bd;
world.pitch = bd;
world.yaw = bd;
world.xdot = bd;
world.ydot = bd;
world.zdot = bd;
world.rolldot = bd;
world.pitchdot = bd;
world.yawdot = bd;
world.mytime = bd;

prog = prog.addStateConstraint(BoundingBoxConstraint(double(seasurface),double(world)),{1,2,3,4,5,6,7,8,9,10}); 

prog = prog.addInputConstraint(ConstantConstraint(u0),1); % DirectTrajectoryOptimization method

xf = x0;                       % final conditions: translated in x


if plant == 'russ'
  xf.base_x = 2;                 % translate x by 2
elseif plant == 'penn'
  xf.x = 6;                 % translate x
  xf.z = 4;                 % translate z
  xf.y = 0;                 % translate x
end



prog = prog.addStateConstraint(ConstantConstraint(double(xf)),N); % DirectTrajectoryOptimization method
prog = prog.addInputConstraint(ConstantConstraint(u0),N); % DirectTrajectoryOptimization method

prog = prog.addRunningCost(@cost); % note: DirCol not Direct!  DircolTrajectoryOptimization method
prog = prog.addFinalCost(@finalCost); % DirectTrajectoryOptimization method

tf0 = 2;                      % initial guess at duration
traj_init.x = PPTrajectory(foh([0,tf0],[double(x0),double(xf)])); % traj.init.x is a PPTrajectory < Trajectory < DrakeSystem
traj_init.u = ConstantTrajectory(u0); % traj_init.u is a ConstantTrajectory < Trajectory < DrakeSystem

info=0;
while (info~=1)
  tic
  snseti ('Verify level', 3);
  [xtraj,utraj,z,F,info] = prog.solveTraj(tf0,traj_init); % DirectTrajectoryOptimization method
  toc
end

if (nargout<1)
  xtraj = xtraj.setOutputFrame(r_temp.getStateFrame());
  v.playback(xtraj,struct('slider',true));
end

end

function [g,dg] = cost(dt,x,u)

R = eye(4);
g = u'*R*u;
%g = sum((R*u).*u,1);
dg = [zeros(1,1+size(x,1)),2*u'*R];
%dg = zeros(1, 1 + size(x,1) + size(u,1));

end

function [h,dh] = finalCost(t,x)

h = t;
dh = [1,zeros(1,size(x,1))];

end

