function [p,utraj,xtraj,z,traj_opt]=runDircolCycle
% from an initial balancing condition, take one step and return to the
% balance upright.

p = CompassGaitPlant();

N = [15;15];

options.u_const_across_transitions = true;
options.periodic = true;

traj_opt = HybridTrajectoryOptimization(@DirtranTrajectoryOptimization, p,[1;2],N,{[.2 .5],[.2 .5]}, options);

x0 = [0;0;2;-.4];
t1 = .417;
x1 = [-.326;.22;-.381;-1.1];
tf = .713;
xf = x0;

t_init{1} = linspace(0,t1,N(1));
t_init{2} = linspace(0,tf-t1,N(2));

traj_init{1}.x0 = x0;
traj_init{2}.x0 = x1;

traj_opt = traj_opt.addModeStateConstraint(1,BoundingBoxConstraint([0;-inf(3,1)],[0;inf(3,1)]),1);
traj_opt = traj_opt.addModeStateConstraint(1,BoundingBoxConstraint([.1;-inf(3,1)],inf(4,1)),N(1));

traj_opt = traj_opt.addModeRunningCost(1,@cost);
traj_opt = traj_opt.addModeRunningCost(2,@cost);

traj_opt = traj_opt.compile();
% traj_opt = traj_opt.setCheckGrad(true);
% snprint('snopt.out');
tic
[xtraj,utraj,z,F,info] = solveTraj(traj_opt,t_init,traj_init);
toc

xtraj = shiftFlipAppend(xtraj);

%  
% xtraj2 = xtraj;
% 
% xtraj = HybridTrajectory({xtraj, xtraj2});
if (nargout<1)
  v = CompassGaitVisualizer(p);
  figure(1); clf;
  fnplt(utraj);
  
  figure(2); clf; hold on;
  fnplt(xtraj,[2 4]);
  fnplt(xtraj,[3 5]);
  
  playback(v,xtraj,struct('slider',true));

end

keyboard
end

function [g,dg] = cost(t,x,u);
R = 1;
g = sum((R*u).*u,1);
dg = [zeros(1,1+size(x,1)),2*u'*R];
end

function [h,dh] = finalcost(t,x)
h=t;
dh = [1,zeros(1,size(x,1))];
end

function J = postImpactTrajCost(T,X,U,p)
% encourage post-impact trajectory to leave collision surface orthogonally
t0=T(1); x0=X(:,1); u0=U(:,1);
xdot0=p.modes{2}.dynamics(t0,x0,u0);
dphidx = [1,1,0,0]; % gradient of foot collision guard 1 w/respect to x
J=100*abs(dphidx*xdot0)./norm(dphidx)./norm(xdot0);
end


function xtraj_ = flipLeftRight(xtraj)

    tshift = xtraj.traj{2}.tspan(1);
    mode1_mode_traj = xtraj.traj{2}.trajs{1};
    mode1_mode_traj = mode1_mode_traj.shiftTime(-tshift);
    mode1_xtraj = xtraj.traj{1}.trajs{2};
    mode1_mixed_traj = MixedTrajectory({mode1_mode_traj, mode1_xtraj},xtraj.traj{1}.indices);
    mode1_mixed_traj = mode1_mixed_traj.setOutputFrame(xtraj.traj{2}.getOutputFrame);
        
    mode2_mode_traj = xtraj.traj{1}.trajs{1};
    mode2_mode_traj = mode2_mode_traj.shiftTime(tshift);
    mode2_xtraj = xtraj.traj{2}.trajs{2};
    mode2_mixed_traj = MixedTrajectory({mode2_mode_traj, mode2_xtraj},xtraj.traj{1}.indices);
    mode2_mixed_traj = mode2_mixed_traj.setOutputFrame(xtraj.traj{1}.getOutputFrame);

    xtraj_ = HybridTrajectory({mode1_mixed_traj,mode2_mixed_traj});

end

function xtraj_ = shiftFlipAppend(xtraj)

    xtraj_flip = flipLeftRight(xtraj);
    xtraj_flip = xtraj_flip.shiftTime(xtraj.tspan(2));
    
    xtraj_ = HybridTrajectory({xtraj.traj{1},xtraj.traj{2},xtraj_flip.traj{1},xtraj_flip.traj{2}});
    
end
