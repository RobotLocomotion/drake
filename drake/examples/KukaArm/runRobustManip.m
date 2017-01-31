function [xtraj1,utraj1,xtraj2,utraj2,xtraj0,utraj0] = runRobustManip()

pad = 0;

options.with_weight = true;
options.with_shelf_and_boxes = true;
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
D = diag([100 100 100]);
E0 = .1*eye(14);

q0 = [0;-0.683;0;1.77;0;0.88;-1.57];
x0 = [q0;zeros(nq,1)];
xG = double(r.resolveConstraints(zeros(nx,1)));
v.draw(0,x0);
N = 40;
tf0 = 3.0;

traj_init.x = PPTrajectory(foh([0,tf0],[x0,xG]));
traj_init.u = ConstantTrajectory(zeros(nu,1));
  
options.integration_method = DirtranTrajectoryOptimization.FORWARD_EULER;
prog1 = DirtranTrajectoryOptimization(r,N,tf0*[1-0.5 1+0.5],options);
prog1 = prog1.addStateConstraint(ConstantConstraint(x0),1);
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

prog1 = prog1.setSolverOptions('snopt','majoroptimalitytolerance',1e-3);
prog1 = prog1.setSolverOptions('snopt','minoroptimalitytolerance',1e-4);
prog1 = prog1.setSolverOptions('snopt','majorfeasibilitytolerance',1e-3);
prog1 = prog1.setSolverOptions('snopt','minorfeasibilitytolerance',1e-4);
prog1 = prog1.setSolverOptions('snopt','iterationslimit',100000);

%Solve without contact constraints to get feasible initial guess
tic;
[xtraj0,utraj0,z,F,info,infeasible] = prog1.solveTraj(tf0,traj_init);
toc

v.playback(xtraj0,struct('slider',true));

%Add contact constraints
for i=2:N-1
  constraint = FunctionHandleConstraint(zeros(4,1),inf*ones(4,1),nx,@l_hand_shelf_constraint);
  constraint = constraint.setName(sprintf('l_hand_shelf_constraint_%d',i));
  prog1 = prog1.addConstraint(constraint, {prog1.x_inds(:,i)});
end

%Use last solution as initial guess
traj_init.x = xtraj0;
traj_init.u = utraj0;

%Re-solve
tic;
[xtraj0,utraj0,z,F,info,infeasible] = prog1.solveTraj(1.5*tf0,traj_init);
toc

v.playback(xtraj0,struct('slider',true));


N = 80;
options.integration_method = DirtranTrajectoryOptimization.MIDPOINT;
prog2 = RobustDirtranTrajectoryOptimization(r,N,D,E0,Q,R,Qf,tf0*[1-0.5 1+0.5],options);
prog2 = prog2.addStateConstraint(ConstantConstraint(x0),1);
prog2 = prog2.addRunningCost(@cost);
prog2 = prog2.addFinalCost(@finalCost);
% prog2 = addTrajectoryDisplayFunction(prog2,@displayStateTrajectory);

[jlmin,jlmax] = r.getJointLimits;
xmin = [jlmin;-inf(nq,1)];
xmax = [jlmax;inf(nq,1)];
prog2 = prog2.addConstraint(BoundingBoxConstraint(repmat(xmin,N,1),repmat(xmax,N,1)),prog2.x_inds);

tol = [0.03;0.03;0.005;0.01;0.01;0.01];

constraint = FunctionHandleConstraint(-tol,tol,nx,@l_hand_constraint);
constraint = constraint.setName('left_hand_constraint'); 
prog2 = prog2.addConstraint(constraint, {prog2.x_inds(:,N)});

prog2 = prog2.setSolverOptions('snopt','majoroptimalitytolerance',1e-3);
prog2 = prog2.setSolverOptions('snopt','minoroptimalitytolerance',1e-4);
prog2 = prog2.setSolverOptions('snopt','majorfeasibilitytolerance',1e-3);
prog2 = prog2.setSolverOptions('snopt','minorfeasibilitytolerance',1e-4);
prog2 = prog2.setSolverOptions('snopt','iterationslimit',100000);

%Add contact constraints
for i=2:N-1
  constraint = FunctionHandleConstraint(zeros(4,1),inf*ones(4,1),nx,@l_hand_shelf_constraint);
  constraint = constraint.setName(sprintf('l_hand_shelf_constraint_%d',i));
  prog2 = prog2.addConstraint(constraint, {prog2.x_inds(:,i)});
end

%Use last solution as initial guess
traj_init.x = xtraj0;
traj_init.u = utraj0;

pad = 0;
%Re-solve
tic;
[xtraj1,utraj1,z,F,info,infeasible] = prog2.solveTraj(1.5*tf0,traj_init);
toc

%---------- Robust Version ----------%

N = 80;
options.integration_method = DirtranTrajectoryOptimization.MIDPOINT;
prog2 = RobustDirtranTrajectoryOptimization(r,N,D,E0,Q,R,Qf,tf0*[1-0.5 1+0.5],options);
prog2 = prog2.addStateConstraint(ConstantConstraint(x0),1);
prog2 = prog2.addFinalCost(@finalCost);
% prog2 = addTrajectoryDisplayFunction(prog2,@displayStateTrajectory);

[jlmin,jlmax] = r.getJointLimits;
xmin = [jlmin;-inf(nq,1)];
xmax = [jlmax;inf(nq,1)];
prog2 = prog2.addConstraint(BoundingBoxConstraint(repmat(xmin,N,1),repmat(xmax,N,1)),prog2.x_inds);

tol = [0.03;0.03;0.005;0.01;0.01;0.01];

constraint = FunctionHandleConstraint(-tol,tol,nx,@l_hand_constraint);
constraint = constraint.setName('left_hand_constraint'); 
prog2 = prog2.addConstraint(constraint, {prog2.x_inds(:,N)});

prog2 = prog2.setSolverOptions('snopt','majoroptimalitytolerance',1e-3);
prog2 = prog2.setSolverOptions('snopt','minoroptimalitytolerance',1e-4);
prog2 = prog2.setSolverOptions('snopt','majorfeasibilitytolerance',1e-3);
prog2 = prog2.setSolverOptions('snopt','minorfeasibilitytolerance',1e-4);
prog2 = prog2.setSolverOptions('snopt','iterationslimit',100000);

%Add contact constraints
for i=2:(N-1)
  constraint = FunctionHandleConstraint(zeros(4,1),inf*ones(4,1),nx,@l_hand_shelf_constraint);
  constraint = constraint.setName(sprintf('l_hand_shelf_constraint_%d',i));
  prog2 = prog2.addConstraint(constraint, {prog2.x_inds(:,i)});
end

prog2 = prog2.addRobustCost(10*ones(14), 1e-2*ones(7), 100*ones(14));

% constraint = FunctionHandleConstraint(zeros(4,1),inf(4,1),nq,@l_hand_shelf_constraint_r,1);
% constraint = constraint.setName('l_hand_shelf_constraint_r');
% prog2 = prog2.addRobustStateConstraint(constraint,2:(N-1),1:nq);

%Use last solution as initial guess
traj_init.x = xtraj1;
traj_init.u = utraj1;

tic;
[xtraj2,utraj2,z,F,info,infeasible] = prog2.solveTraj(1.5*tf0,traj_init);
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
    Qr = diag([zeros(nq,1); ones(nq,1)]);
    Rr = 0.001*eye(nu);

    g = (x-xG)'*Qr*(x-xG) + u'*Rr*u;
    dg = [0, 2*(x'*Qr -xG'*Qr), 2*u'*Rr];
    ddg = blkdiag(0, 2*Qr, 2*Rr);
end

function [g,dg,ddg] = finalCost(T,x)
    Qn = diag([zeros(nq,1); 100*ones(nq,1)]);

    g = (x-xG)'*Qn*(x-xG);
    dg = [0, 2*(x'*Qn -xG'*Qn)];
    ddg = blkdiag(0, 2*Qn);
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
    if pad
        f = phi - [.01 0 0 0]';
    else
        f = phi;
    end
    df = [J,zeros(length(f),nq)];
end

function [f,df] = l_hand_shelf_constraint_r(q)
    kinsol = doKinematics(r, q);
    [phi,~,~,~,~,~,~,~,J] = r.contactConstraints(kinsol);
    f = phi;
    df = J;
end

end