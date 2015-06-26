function [utraj,xtraj,prog,r] = runDircol

% simple planning demo which takes the quadrotor from hover at x=0m to a new hover at
% x=2m with minimal thrust.

r = Quadrotor();

N = 95;
minimum_duration = .1;
maximum_duration = 10;
prog = DircolTrajectoryOptimization(r,N,[minimum_duration maximum_duration]);  

x0 = Point(getStateFrame(r));  % initial conditions: all-zeros
nq = numel(double(x0));
x0.base_z = .5;
u0 = double(nominalThrust(r));

prog = prog.addStateConstraint(ConstantConstraint(double(x0)),1);
prog = prog.addInputConstraint(ConstantConstraint(u0),1);

prog = addWaypoint(prog, [1, 0], 0, 12);
prog = addWaypoint(prog, [1.5, -0.866], -2*pi/6, 23);
prog = addWaypoint(prog, [2.366, -1.366], -pi/6, 34);
prog = addWaypoint(prog, [3.366, -1.366], 0, 45);
prog = addWaypoint(prog, [4.2321, -0.866], pi/6, 56);
prog = addWaypoint(prog, [4.7321, 0], 2*pi/6, 67);


xf = x0;
xf.base_x = 5.7321;
xf.base_y = 0;
xf.base_yaw = 0;

prog = prog.addStateConstraint(ConstantConstraint(double(xf)),N);
prog = prog.addInputConstraint(ConstantConstraint(u0),N);

A = zeros(8, 2*nq);
A(1, 4) = 1;
A(1, 4+nq) = -1;
A(2, 5) = 1;
A(2, 5+nq) = -1;
A(3, 7) = 1;
A(3, 7+nq) = -1;
A(4, 8) = 1;
A(4, 8+nq) = -1;
A(5, 9) = 1;
A(5, 9+nq) = -1;
A(6, 10) = 1;
A(6, 10+nq) = -1;
A(7, 11) = 1;
A(7, 11+nq) = -1;
A(8, 12) = 1;
A(8, 12+nq) = -1;

nonCyclicTesselationConstraint = LinearConstraint(zeros(8,1), zeros(8,1), A);
prog = prog.addStateConstraint(nonCyclicTesselationConstraint, {[12 23], [23 34], [34 45], [45 56], [56, 67]});

v = constructVisualizer(r)
v.draw(0,double(x0));
prog = addPlanVisualizer(r,prog);

prog = prog.addRunningCost(@cost);
prog = prog.addFinalCost(@finalCost);

tf0 = 10;                      % initial guess at duration 
traj_init.x = PPTrajectory(foh([0,tf0],[double(x0),double(xf)]));
traj_init.u = ConstantTrajectory(u0);

info=0;
while (info~=1)
  tic
  [xtraj,utraj,z,F,info] = prog.solveTraj(tf0,traj_init);
  toc
end

if (nargout<1)
  v = constructVisualizer(r);
  v.playback(xtraj,struct('slider',true));
end

end

function [g,dg] = cost(dt,x,u)

R = eye(4);
g = u'*R*u;
dg = [zeros(1,1+size(x,1)),2*u'*R];

end

function [h,dh] = finalCost(t,x)

h = t;
dh = [1,zeros(1,size(x,1))];

end


function prog = addWaypoint(prog, xy, yaw, knot)

xm_lb = -Inf*ones(12,1);
xm_ub = Inf*ones(12,1);

xm_lb(1) = xy(1);
xm_ub(1) = xy(1);

xm_lb(2) = xy(2);
xm_ub(2) = xy(2);

xm_lb(3) = 0.5;
xm_ub(3) = 0.5;

xm_lb(6) = yaw;
xm_ub(6) = yaw;

prog = prog.addStateConstraint(BoundingBoxConstraint(xm_lb, xm_ub), knot);

end