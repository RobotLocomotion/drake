function runDirtranManip

options.with_weight = true;
options.with_shelf_and_boxes = true;
r = KukaArm(options);

nx = r.getNumStates;
nq = r.getNumPositions;
nu = r.getNumInputs;

v=r.constructVisualizer;

q0 = [0;-0.683;0;1.77;0;0.88;-1.57];
x0 = [q0;zeros(nq,1)];
xG = double(r.resolveConstraints(zeros(nx,1)));
v.draw(0,x0);
N = 40;
tf0 = 3.0;

traj_init.x = PPTrajectory(foh([0,tf0],[x0,xG]));
traj_init.u = ConstantTrajectory(zeros(nu,1));
  
options.integration_method = DirtranTrajectoryOptimization.FORWARD_EULER;
traj_opt = DirtranTrajectoryOptimization(r,N,tf0*[(1-0.5) (1+0.5)],options);
traj_opt = traj_opt.addStateConstraint(ConstantConstraint(x0),1);
% traj_opt = traj_opt.addStateConstraint(ConstantConstraint(xG),N);
traj_opt = traj_opt.addRunningCost(@cost);
traj_opt = traj_opt.addFinalCost(@finalCost);
% traj_opt = addTrajectoryDisplayFunction(traj_opt,@displayStateTrajectory);

[jlmin,jlmax] = r.getJointLimits;
xmin = [jlmin;-inf(nq,1)];
xmax = [jlmax;inf(nq,1)];
traj_opt = traj_opt.addConstraint(BoundingBoxConstraint(repmat(xmin,N,1),repmat(xmax,N,1)),traj_opt.x_inds);

tol = [0.03;0.03;0.005;0.01;0.01;0.01];

constraint = FunctionHandleConstraint(-tol,tol,nx,@l_hand_constraint);
constraint = constraint.setName('left_hand_constraint'); 
traj_opt = traj_opt.addConstraint(constraint, {traj_opt.x_inds(:,N)});

traj_opt = traj_opt.setSolverOptions('snopt','majoroptimalitytolerance',1e-3);
traj_opt = traj_opt.setSolverOptions('snopt','minoroptimalitytolerance',1e-4);
traj_opt = traj_opt.setSolverOptions('snopt','majorfeasibilitytolerance',1e-3);
traj_opt = traj_opt.setSolverOptions('snopt','minorfeasibilitytolerance',1e-4);
traj_opt = traj_opt.setSolverOptions('snopt','iterationslimit',1000000);

tic;
[xtraj,utraj,z,F,info,infeasible] = traj_opt.solveTraj(tf0,traj_init);
toc

v.playback(xtraj,struct('slider','true'))

for i=2:N-1
  constraint = FunctionHandleConstraint(zeros(5,1),inf*ones(5,1),nx,@l_hand_shelf_constraint);
  constraint = constraint.setName(sprintf('l_hand_shelf_constraint_%d',i));
  traj_opt = traj_opt.addConstraint(constraint, {traj_opt.x_inds(:,i)});
end

traj_init.x = xtraj;
traj_init.u = utraj;
tic;
[xtraj,utraj,z,F,info,infeasible] = traj_opt.solveTraj(tf0,traj_init);
toc

x = z(traj_opt.x_inds);
u = z(traj_opt.u_inds);

v.playback(xtraj,struct('slider','true'))

Q = diag([100*ones(nq,1);10*ones(nq,1)]);
R = 0.01*eye(nu);
Qf = 5*Q;
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
    Q = diag([zeros(nq,1);1*ones(nq,1)]);
    R = 0.001*eye(nu);

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