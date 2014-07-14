function runDircolWObs

% simple planning demo which takes the quadrotor from hover at y=0m to a new hover at
% y=10m with minimal thrust.

r = Quadrotor();
r = addTrees(r,15);

N = 21;
minimum_duration = 2;
maximum_duration = 10;
prog = DircolTrajectoryOptimization(r,N,[minimum_duration maximum_duration]);  

x0 = Point(getStateFrame(r));  % initial conditions: all-zeros
x0.base_z = .5;
u0 = double(nominalThrust(r));

if (nargout<1)
  v = constructVisualizer(r);%,struct('use_contact_shapes',true));
  v.draw(0,double(x0));

  prog = addPlanVisualizer(r,prog);
end

prog = prog.addStateConstraint(ConstantConstraint(double(x0)),1);
prog = prog.addInputConstraint(ConstantConstraint(u0),1);

xf = x0;                       % final conditions: translated in x
xf.base_y = 10;
prog = prog.addStateConstraint(ConstantConstraint(double(xf)),N);
prog = prog.addInputConstraint(ConstantConstraint(u0),N);

prog = prog.addRunningCost(@cost);
prog = prog.addFinalCost(@final_cost);

tf0 = 6;                      % initial guess at duration

disp('solving w/o obstacles');
traj_init.x = PPTrajectory(foh([0,tf0],[double(x0),double(xf)]));
traj_init.u = ConstantTrajectory(u0);

%traj_opt = setCheckGrad(traj_opt,true);

tic
[xtraj,utraj,z,F,info] = prog.solveTraj(tf0,traj_init);
toc

disp('resolving w/ obstacles');
snprint('snopt.out');
collision_constraint = generateConstraint(MinDistanceConstraint(r,0.1),0);
prog = prog.addStateConstraint(collision_constraint{1},1:N,1:getNumPositions(r));
traj_init.x = xtraj;
traj_init.u = utraj;

tic
[xtraj,utraj,z,F,info] = prog.solveTraj(tf0,traj_init);
toc

if (nargout<1)
  v.playback(xtraj);%,struct('slider',true));
end

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

