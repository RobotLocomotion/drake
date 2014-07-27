function [utraj,xtraj,prog] = runPeriodicDircol

r = Quadrotor();

N = 11;
minimum_duration = .1;
maximum_duration = 4;
prog = DircolTrajectoryOptimization(r,N,[minimum_duration maximum_duration]);  

x0 = Point(getStateFrame(r));  % initial conditions: all-zeros
x0.base_z = .5;
x0.base_roll = .5;	% putting the quad in unstable state to remove the trivial periodic solution
x0.base_yaw = .5;
u0 = double(nominalThrust(r));

prog = prog.addStateConstraint(ConstantConstraint(double(x0)),1);
prog = prog.addInputConstraint(ConstantConstraint(u0),1);

% all the initial and final states the same
prog = prog.addStateConstraint(PeriodicConstraint(zeros(r.getNumStates(),1)),{[1 N]});

prog = prog.addRunningCost(@cost);
prog = prog.addFinalCost(@finalCost);

tf0 = 2;                      % initial guess at duration 
traj_init.x = PPTrajectory(foh([0,tf0],[double(x0),double(x0)]));
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
%g = sum((R*u).*u,1);
dg = [zeros(1,1+size(x,1)),2*u'*R];
%dg = zeros(1, 1 + size(x,1) + size(u,1));

end

function [h,dh] = finalCost(t,x)

h = t;
dh = [1,zeros(1,size(x,1))];

end
