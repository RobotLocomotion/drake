function runRobustManip()

N = 45;

options.with_weight = true;
options.with_box = true;
r = KukaArm(options);

nx = r.getNumStates;
nq = r.getNumPositions;
nu = r.getNumInputs;

v=r.constructVisualizer;

q0 = [0;1.57;0;0;0;0;1.57];
x0 = [q0;zeros(nq,1)];
xG = double(r.resolveConstraints(zeros(nx,1)));

%Disturbance + LQR Weights
D = diag([.1 .1 .05].^2);
E0 = .01^2*eye(12);
Q = blkdiag(10*eye(6), 1*eye(6));
R = .1*eye(4);
Qf = 5*Q;

tf0 = 3.0;

traj_init.x = PPTrajectory(foh([0,tf0],[x0,xG]));
traj_init.u = ConstantTrajectory(zeros(nu,1));
  
options.integration_method = DirtranTrajectoryOptimization.MIDPOINT;
prog0 = DirtranTrajectoryOptimization(r,N,tf0*[(1-0.1) (1+0.1)],options);
prog0 = prog0.addStateConstraint(ConstantConstraint(x0),1);
%prog0 = prog0.addStateConstraint(ConstantConstraint(xG),N);
%prog0 = prog0.addRunningCost(@cost);
prog0 = prog0.addFinalCost(@finalCost);

[jlmin,jlmax] = r.getJointLimits;
xmin = [jlmin;-inf(nq,1)];
xmax = [jlmax;inf(nq,1)];
prog0 = prog0.addConstraint(BoundingBoxConstraint(repmat(xmin,N,1),repmat(xmax,N,1)),prog0.x_inds);

constraint = FunctionHandleConstraint(-1e-1*ones(3,1),1e-1*ones(3,1),nx,@l_hand_constraint);
constraint = constraint.setName('left_hand_constraint'); 
prog0 = prog0.addConstraint(constraint, {prog0.x_inds(:,N)});

for i=1:N
  constraint = FunctionHandleConstraint(0,inf,nx,@l_hand_box_constraint);
  constraint = constraint.setName(sprintf('l_hand_box_constraint_%d',i));
  prog0 = prog0.addConstraint(constraint, {prog0.x_inds(:,i)});
end

tic;
[xtraj,utraj,z,F,info,infeasible] = prog0.solveTraj(tf0,traj_init);
toc

v.playback(xtraj,struct('slider','true'))

%Run robust version
options.integration_method = DirtranTrajectoryOptimization.MIDPOINT;
prog1 = RobustDirtranTrajectoryOptimization(r,N,D,E0,Q,R,Qf,tf0*[(1-0.1) (1+0.1)],options);
prog1 = prog1.addStateConstraint(ConstantConstraint(x0),1);
%prog1 = prog1.addStateConstraint(ConstantConstraint(xG),N);
%prog1 = prog1.addRunningCost(@cost);
prog1 = prog1.addFinalCost(@finalCost);
prog1 = prog1.addRobustCost(Q,R,Qf);
% prog1 = prog1.setSolverOptions('snopt','majoroptimalitytolerance',1e-3);
% prog1 = prog1.setSolverOptions('snopt','minoroptimalitytolerance',1e-4);
% prog1 = prog1.setSolverOptions('snopt','majorfeasibilitytolerance',1e-3);
% prog1 = prog1.setSolverOptions('snopt','minorfeasibilitytolerance',1e-4);
% prog1 = prog1.setSolverOptions('snopt','iterationslimit',100000);

[jlmin,jlmax] = r.getJointLimits;
xmin = [jlmin;-inf(nq,1)];
xmax = [jlmax;inf(nq,1)];
prog1 = prog1.addConstraint(BoundingBoxConstraint(repmat(xmin,N,1),repmat(xmax,N,1)),prog1.x_inds);

constraint = FunctionHandleConstraint(-1e-1*ones(3,1),1e-1*ones(3,1),nx,@l_hand_constraint);
constraint = constraint.setName('left_hand_constraint'); 
prog1 = prog1.addConstraint(constraint, {prog0.x_inds(:,N)});

for i=1:N
  constraint = FunctionHandleConstraint(0,inf,nx,@l_hand_box_constraint);
  constraint = constraint.setName(sprintf('l_hand_box_constraint_%d',i));
  prog1 = prog1.addConstraint(constraint, {prog1.x_inds(:,i)});
end

%Run again with box constraint
traj_init.x = xtraj;
traj_init.u = utraj;
tic;
[xtraj,utraj,z,F,info,infeasible] = prog1.solveTraj(tf0,traj_init);
toc

v.playback(xtraj,struct('slider','true'))

keyboard

c = tvlqr(r,xtraj,utraj,Q,R,Qf);

rt = TimeSteppingRigidBodyManipulator(r,0.001);

sys = feedback(rt,c);

if 1
  % Forward simulate dynamics with visulazation, then playback at realtime
  S=warning('off','Drake:DrakeSystem:UnsupportedSampleTime');
  output_select(1).system=1;
  output_select(1).output=1;
  sys = mimoCascade(sys,v,[],[],output_select);
  warning(S);
end
traj=simulate(sys,[0,utraj.tspan(2)],xtraj.eval(xtraj.tspan(1)));
v.playback(traj,struct('slider',true));

keyboard

  function displayStateTrajectory(t,x,u)
    ts = [0,cumsum(t)'];
    xtraj = PPTrajectory(foh(ts,x));
    xtraj = xtraj.setOutputFrame(r.getStateFrame);
    v.playback(xtraj);
  end

  function [g,dg,ddg] = cost(h,x,u)
    Q = diag([zeros(nq,1);zeros(nq,1)]);
    R = 1e-3*eye(nu);

    g = (x-xG)'*Q*(x-xG) + u'*R*u;
    dg = [0, 2*(x'*Q -xG'*Q), 2*u'*R];
    ddg = blkdiag(0, 2*Q, 2*R);
  end

  function [g,dg,ddg] = finalCost(T,x)
    Q = diag([zeros(nq,1);ones(nq,1)]);
    
    g = (x-xG)'*Q*(x-xG);
    dg = [0, 2*(x-xG)'*Q];
    ddg = blkdiag(0, 2*Q);
  end


  function [f,df] = l_hand_constraint(x)
    q = x(1:nq);
    kinsol = doKinematics(r, q);
    [pl,Jl] = forwardKin(r,kinsol,r.findLinkId('iiwa_link_ee'),[0;0;0]);

    f = pl-[0.0;0.0;1.36];
    df = [Jl,zeros(length(f),nq)];
  end

  function [f,df] = l_hand_box_constraint(x)
    q = x(1:nq);
    kinsol = doKinematics(r, q);
    [pl,Jl] = forwardKin(r,kinsol,r.findLinkId('iiwa_link_ee'),[0;0;0.1]);

    box_center = [0.6;0;1.4];
    radius = 0.4 + 0.05;

    % approx
    err = pl - box_center;
    f = err'*err - radius^2;
    df = [2*err'*Jl,zeros(length(f),nq)];
  end

end