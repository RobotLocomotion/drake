function [xtraj1,utraj1,xtraj2,utraj2] = runRobustManip()

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
D = diag([5 5 5]);
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

constraint = FunctionHandleConstraint(-tol,tol,nq,@l_hand_constraint);
constraint = constraint.setName('left_hand_constraint');
prog1 = prog1.addStateConstraint(constraint,N,1:nq);

constraint = FunctionHandleConstraint(zeros(3,1),inf(3,1),nq,@l_hand_shelf_constraint,1);
constraint = constraint.setName('l_hand_shelf_constraint');
prog1 = prog1.addStateConstraint(constraint,2:(N-1),1:nq);

prog1 = prog1.setSolverOptions('snopt','majoroptimalitytolerance',1e-2);
prog1 = prog1.setSolverOptions('snopt','minoroptimalitytolerance',1e-3);
prog1 = prog1.setSolverOptions('snopt','majorfeasibilitytolerance',1e-2);
prog1 = prog1.setSolverOptions('snopt','minorfeasibilitytolerance',1e-3);
prog1 = prog1.setSolverOptions('snopt','iterationslimit',100000);

tic;
[xtraj,utraj,z,F,info,infeasible] = prog1.solveTraj(tf0,traj_init);
toc

prog1 = prog1.setSolverOptions('snopt','majoroptimalitytolerance',1e-3);
prog1 = prog1.setSolverOptions('snopt','minoroptimalitytolerance',1e-4);
prog1 = prog1.setSolverOptions('snopt','majorfeasibilitytolerance',1e-3);
prog1 = prog1.setSolverOptions('snopt','minorfeasibilitytolerance',1e-4);
prog1 = prog1.setSolverOptions('snopt','iterationslimit',100000);

tic;
[xtraj1,utraj1,z,F,info,infeasible] = prog1.solveTraj(tf0,traj_init);
toc

v.playback(xtraj1,struct('slider',true));

%Robust Version

traj_init.x = xtraj;
traj_init.u = utraj;

prog1 = prog1.addRobustCost(.1*Q,.1*R,.1*Qf);
%prog2 = prog2.addRobustInputConstraint();

constraint = FunctionHandleConstraint(zeros(3,1),inf(3,1),nq,@l_hand_shelf_constraint);
constraint = constraint.setName('robust_shelf_constraint');
prog1 = prog1.addRobustStateConstraint(constraint,2:(N-1),1:nq);

prog1 = prog1.setSolverOptions('snopt','majoroptimalitytolerance',1e-2);
prog1 = prog1.setSolverOptions('snopt','minoroptimalitytolerance',1e-3);
prog1 = prog1.setSolverOptions('snopt','majorfeasibilitytolerance',1e-2);
prog1 = prog1.setSolverOptions('snopt','minorfeasibilitytolerance',1e-3);
prog1 = prog1.setSolverOptions('snopt','iterationslimit',100000);

tic;
[xtraj2,utraj2,z,F,info,infeasible] = prog1.solveTraj(tf0,traj_init);
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


function [f,df] = l_hand_constraint(q)
    kinsol = doKinematics(r, q);
    opt.rotation_type = 1;
    [pl,Jl] = forwardKin(r,kinsol,r.findLinkId('iiwa_link_ee'),[0;0;0],opt);
    f = pl-[0.4;0.0;1.03;pi/2;0;pi/2];
    df = Jl;
end

function [f,df] = l_hand_shelf_constraint(q)
    kinsol = doKinematics(r, q);
    [phi,~,~,~,~,~,~,~,J] = r.contactConstraints(kinsol);
    f = phi;
    df = J;
end

end