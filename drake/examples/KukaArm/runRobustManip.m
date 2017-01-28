function [xtraj,utraj,xtraj2,utraj2] = runRobustManip()

options.with_weight = true;
options.with_shelf = true;
r = KukaArm(options);

nx = r.getNumStates;
nq = r.getNumPositions;
nu = r.getNumInputs;

v=r.constructVisualizer;

%LQR Controller Stuff
Q = diag([100*ones(nq,1);10*ones(nq,1)]);
R = 0.01*eye(nu);
Qf = 5*Q;

%Robust stuff
D = diag([1 1 1]);
E0 = .1*eye(14);

q0 = [0;-0.683;0;1.77;0;0.88;-1.57];
x0 = [q0;zeros(nq,1)];
xG = double(r.resolveConstraints(zeros(nx,1)));
v.draw(0,x0);
N = 20;
tf0 = 3.0;

traj_init.x = PPTrajectory(foh([0,tf0],[x0,xG]));
traj_init.u = ConstantTrajectory(zeros(nu,1));
  
options.integration_method = DirtranTrajectoryOptimization.MIDPOINT;
prog1 = RobustDirtranTrajectoryOptimization(r,N,D,E0,Q,R,Qf,tf0*[.8 1.2],options);
prog1 = prog1.addStateConstraint(ConstantConstraint(x0),1);
% prog1 = prog1.addStateConstraint(ConstantConstraint(xG),N);
prog1 = prog1.addRunningCost(@cost);
prog1 = prog1.addFinalCost(@finalCost);
% prog1 = addTrajectoryDisplayFunction(prog1,@displayStateTrajectory);

[jlmin,jlmax] = r.getJointLimits;
xmin = [jlmin;-inf(nq,1)];
xmax = [jlmax;inf(nq,1)];
prog1 = prog1.addConstraint(BoundingBoxConstraint(repmat(xmin,N,1),repmat(xmax,N,1)),prog1.x_inds);

tol = [0.03;0.03;0.005;0.01;0.01;0.01];

constraint = FunctionHandleConstraint(-tol,tol,nx,@l_hand_constraint);
constraint = constraint.setName('left_hand_constraint'); 
prog1 = prog1.addConstraint(constraint, {prog1.x_inds(:,N)});

for i=1:N-1
  constraint = FunctionHandleConstraint(zeros(3,1),inf(3,1),nx,@l_hand_shelf_constraint);
  constraint = constraint.setName(sprintf('l_hand_shelf_constraint_%d',i));
  prog1 = prog1.addConstraint(constraint, {prog1.x_inds(:,i)});
end

prog1 = prog1.setSolverOptions('snopt','majoroptimalitytolerance',1e-2);
prog1 = prog1.setSolverOptions('snopt','minoroptimalitytolerance',1e-3);
prog1 = prog1.setSolverOptions('snopt','majorfeasibilitytolerance',1e-3);
prog1 = prog1.setSolverOptions('snopt','minorfeasibilitytolerance',1e-4);
prog1 = prog1.setSolverOptions('snopt','iterationslimit',100000);

tic;
[xtraj,utraj,z,F,info,infeasible] = prog1.solveTraj(tf0,traj_init);
toc

v.playback(xtraj,struct('slider',true));

traj_init.x = xtraj;
traj_init.u = utraj;

%Robust Version
options.integration_method = DirtranTrajectoryOptimization.MIDPOINT;
prog2 = RobustDirtranTrajectoryOptimization(r,N,D,E0,Q,R,Qf,tf0*[.8 1.2],options);
prog2 = prog2.addStateConstraint(ConstantConstraint(x0),1);
% prog2 = prog2.addStateConstraint(ConstantConstraint(xG),N);
prog2 = prog2.addRunningCost(@cost);
prog2 = prog2.addFinalCost(@finalCost);
% prog2 = addTrajectoryDisplayFunction(prog2,@displayStateTrajectory);

prog2 = prog2.addRobustCost(.1*Q,.1*R,Qf);
%prog2 = prog2.addRobustInputConstraint();
%prog2 = prog2.addRobustStateConstraint(collision_constraint,2:(N2-1),1:2);

[jlmin,jlmax] = r.getJointLimits;
xmin = [jlmin;-inf(nq,1)];
xmax = [jlmax;inf(nq,1)];
prog2 = prog2.addConstraint(BoundingBoxConstraint(repmat(xmin,N,1),repmat(xmax,N,1)),prog2.x_inds);
% prog2 = prog2.addRobustStateConstraint(BoundingBoxConstraint(xmin,xmax),2:(N-1),1:14);

tol = [0.03;0.03;0.005;0.01;0.01;0.01];

constraint = FunctionHandleConstraint(-tol,tol,nx,@l_hand_constraint);
constraint = constraint.setName('left_hand_constraint'); 
prog2 = prog2.addConstraint(constraint, {prog2.x_inds(:,N)});
%prog2 = prog2.addRobustStateConstraint(constraint,N-1,1:14);

for i=1:N-1
  constraint = FunctionHandleConstraint(zeros(3,1),inf(3,1),nx,@l_hand_shelf_constraint);
  constraint = constraint.setName(sprintf('l_hand_shelf_constraint_%d',i));
  prog2 = prog2.addConstraint(constraint, {prog2.x_inds(:,i)});
%   if i > 1
%       prog2 = prog2.addRobustStateConstraint(constraint,i,1:14);
%   end
end

prog2 = prog2.setSolverOptions('snopt','majoroptimalitytolerance',1e-2);
prog2 = prog2.setSolverOptions('snopt','minoroptimalitytolerance',1e-3);
prog2 = prog2.setSolverOptions('snopt','majorfeasibilitytolerance',1e-3);
prog2 = prog2.setSolverOptions('snopt','minorfeasibilitytolerance',1e-4);
prog2 = prog2.setSolverOptions('snopt','iterationslimit',100000);

tic;
[xtraj2,utraj2,z,F,info,infeasible] = prog2.solveTraj(tf0,traj_init);
toc

v.playback(xtraj2,struct('slider',true));


%--------- Cost + Constraint Functions ---------%

function displayStateTrajectory(t,x,u)
    ts = [0,cumsum(t)'];
    xtraj = PPTrajectory(foh(ts,x));
    xtraj = xtraj.setOutputFrame(r.getStateFrame);
    v.playback(xtraj);
end

function [g,dg,ddg] = cost(h,x,u)
    Q = diag([zeros(nq,1);.1*ones(nq,1)]);
    R = 1e-5*eye(nu);

    g = (x-xG)'*Q*(x-xG) + u'*R*u;
    dg = [0, 2*(x'*Q -xG'*Q), 2*u'*R];
    ddg = blkdiag(0, 2*Q, 2*R);
end

function [g,dg,ddg] = finalCost(T,x)
    Q = diag([zeros(nq,1);100*ones(nq,1)]);

    g = (x-xG)'*Q*(x-xG);
    dg = [0, 2*(x'*Q -xG'*Q)];
    ddg = blkdiag(0, 2*Q);
end


function [f,df] = l_hand_constraint(x)
    q = x(1:nq);
    kinsol = doKinematics(r, q);
    opt.rotation_type = 1;
    [pl,Jl] = forwardKin(r,kinsol,r.findLinkId('iiwa_link_ee'),[0;0;0],opt);

    f = pl-[0.4;0.0;1.03;pi/2;0;pi/2];
    df = [Jl,zeros(length(f),nq)];
end

function [f,df] = l_hand_shelf_constraint(x)
    q = x(1:nq);
    kinsol = doKinematics(r, q);
    [phi,~,~,~,~,~,~,~,J] = r.contactConstraints(kinsol);

    f = phi;
    df = [J,zeros(length(f),nq)];
end

end