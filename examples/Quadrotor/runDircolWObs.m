function [r, xtraj, utraj, prog] = runDircolWObs

% simple planning demo which takes the quadrotor from hover at y=0m to a new hover at
% y=10m with minimal thrust. Creates a forest set up to have the Quadrotor
% swerve between trees blocking its path to get to y=10m.

r = Quadrotor();
% The important trees to create swerving path
r = addTree(r, [.8,.45,1.25], [.20;2.5], pi/4);
r = addTree(r, [.5,.35,1.65], [-.25;5], -pi/6);
r = addTree(r, [.55,.65,1.5], [.25;7.5], pi/4);
r = addTree(r, [.55,.85,1.6], [-1.35;8.5], pi/3.7);
r = addTree(r, [.85,.95,1.65], [-1.85;5.2], -pi/3.7);
r = addTree(r, [.75,.9,1.75], [2;4.4], -pi/5);
% Random trees to make forest bigger and more dense
r = addTrees(r, 25);
N = 20;
minimum_duration = 0;
maximum_duration = 2.8;
prog = DircolTrajectoryOptimization(r,N,[minimum_duration maximum_duration]);  


x0 = Point(getStateFrame(r));  % initial conditions: all-zeros
x0.base_y = -1.5;
x0.base_z = .5;
x0.base_ydot = 5;
u0 = double(nominalThrust(r));

%if (nargout<1)
  v = constructVisualizer(r);%,struct('use_collision_geometry',true));
  v.draw(0,double(x0));

  prog = addPlanVisualizer(r,prog);
%end

prog = prog.addStateConstraint(ConstantConstraint(double(x0)),1);
prog = prog.addInputConstraint(ConstantConstraint(u0),1);

xf = x0;                       % final conditions: translated in x
xf.base_y = 11;
prog = prog.addStateConstraint(ConstantConstraint(double(xf)),N);
prog = prog.addInputConstraint(ConstantConstraint(u0),N);

prog = prog.addRunningCost(@cost);
prog = prog.addFinalCost(@final_cost);

collision_constraint = generateConstraint(MinDistanceConstraint(r,0.1),0);
prog = prog.addStateConstraint(collision_constraint{1},1:N,1:getNumPositions(r));

tf0 = 6;                      % initial guess at duration

traj_init.x = PPTrajectory(foh([0,tf0],[double(x0),double(xf)]));
traj_init.u = ConstantTrajectory(u0);

prog = prog.setSolverOptions('snopt','majoroptimalitytolerance',1e-2);
prog = prog.setSolverOptions('snopt','majorfeasibilitytolerance',1e-2);

tic
[xtraj,utraj,z,F,info] = prog.solveTraj(tf0,traj_init);
toc

%if (nargout<1)
  v.playback(xtraj);%,struct('slider',true));
%end

end

function [g,dg] = cost(dt,x,u)

R = eye(4);
g = u'*R*u;
%g = sum((R*u).*u,1);
dg = [zeros(1,1+size(x,1)),2*u'*R];
%dg = zeros(1, 1 + size(x,1) + size(u,1));

end

function [h,dh] = final_cost(t,x)

h = t;
dh = [1,zeros(1,size(x,1))];

end

