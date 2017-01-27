function runRobustManip()

options.with_weight = true;
options.with_shelf = true;
r = KukaArm(options);

nx = r.getNumStates;
nq = r.getNumPositions;
nu = r.getNumInputs;

v=r.constructVisualizer;

q0 = [0;-0.683;0;1.77;0;0.88;-1.57];
x0 = [q0;zeros(nq,1)];
xG = double(r.resolveConstraints(zeros(nx,1)));
v.draw(0,x0);
N = 30;
tf0 = 3.0;

traj_init.x = PPTrajectory(foh([0,tf0],[x0,xG]));
traj_init.u = ConstantTrajectory(zeros(nu,1));
  
options.integration_method = DirtranTrajectoryOptimization.MIDPOINT;
prog1 = DirtranTrajectoryOptimization(r,N,tf0*[.8 1.2],options);
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
  constraint = FunctionHandleConstraint(0,inf,nx,@l_hand_shelf_constraint);
  constraint = constraint.setName(sprintf('l_hand_shelf_constraint_%d',i));
  prog1 = prog1.addConstraint(constraint, {prog1.x_inds(:,i)});
end

prog1 = prog1.setSolverOptions('snopt','majoroptimalitytolerance',1e-2);
prog1 = prog1.setSolverOptions('snopt','minoroptimalitytolerance',1e-3);
%prog1 = prog1.setSolverOptions('snopt','majorfeasibilitytolerance',1e-4);
prog1 = prog1.setSolverOptions('snopt','iterationslimit',100000);

tic;
[xtraj,utraj,z,F,info,infeasible] = prog1.solveTraj(tf0,traj_init);
toc


%LQR Controller
Q = diag([100*ones(nq,1);10*ones(nq,1)]);
R = 0.01*eye(nu);
Qf = 5*Q;
c = tvlqr(r,xtraj,utraj,Q,R,Qf);

%Simulate closed-loop system
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

%Run robust version

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