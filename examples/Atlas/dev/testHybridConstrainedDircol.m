function [p,v,xtraj,utraj,ltraj,z,F,info,traj_opt] = testHybridConstrainedDircol

warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:WeldedLinkInd');
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
options.terrain = RigidBodyFlatTerrain();
options.floating = true;
options.ignore_self_collisions = true;

p = PlanarRigidBodyManipulator('../urdf/atlas_simple_planar_contact.urdf',options);

v = p.constructVisualizer();
%works with this
% N = [10,5,5];
% duration = {[.2 .5],[.02 .2],[.02 .2]};
% modes = {[1;2],[1;2;3],[1;2;3;4]};

N = [8,3,3,5];
duration = {[.2 .5],[.02 .2],[.02 .2],[.02 .2]};
modes = {[1;2],[1;2;3],[1;2;3;4],[2;3;4]};

x0 = [      0
    0.9371
    0.2000
   -0.4414
    0.2625
   -0.0211
    0.0891
   -0.4997
    0.9273
   -0.6403
    0.2346
   -0.0077
    0.0731
   -0.2012
    0.7876
   -0.6596
    0.1798
   -1.9375
    2.5602
   -2.2319]; 
to_options.lambda_bound = 1e2;
traj_opt = ConstrainedHybridTrajectoryOptimization(p,modes,N,duration,to_options);

l0 = [0;897.3515;0;179.1489];

traj_opt = traj_opt.addModeStateConstraint(1,BoundingBoxConstraint(x0 - [.01*ones(10,1);.1*ones(10,1)],x0+[.01*ones(10,1);.1*ones(10,1)]),1);

t_init{1} = linspace(0,.4,N(1));
traj_init.mode{1}.x = PPTrajectory(foh(t_init{1},repmat(x0,1,N(1))));
traj_init.mode{1}.u = PPTrajectory(foh(t_init{1},randn(7,N(1))));
traj_init.mode{1}.l = PPTrajectory(foh(t_init{1},repmat(l0,1,N(1))));

if length(N) > 1
t_init{2} = linspace(0,.2,N(2));
traj_init.mode{2}.x = PPTrajectory(foh(t_init{2},repmat(x0,1,N(2))));
traj_init.mode{2}.u = PPTrajectory(foh(t_init{2},randn(7,N(2))));
traj_init.mode{2}.l = PPTrajectory(foh(t_init{2},repmat([l0;l0(1:2)],1,N(2))));
end
if length(N) > 2
t_init{3} = linspace(0,.2,N(3));
traj_init.mode{3}.x = PPTrajectory(foh(t_init{3},repmat(x0,1,N(3))));
traj_init.mode{3}.u = PPTrajectory(foh(t_init{3},randn(7,N(3))));
traj_init.mode{3}.l = PPTrajectory(foh(t_init{3},repmat([l0;l0],1,N(3))));
end
if length(N) > 3
t_init{4} = linspace(0,.2,N(4));
traj_init.mode{4}.x = PPTrajectory(foh(t_init{4},repmat(x0,1,N(4))));
traj_init.mode{4}.u = PPTrajectory(foh(t_init{4},randn(7,N(4))));
traj_init.mode{4}.l = PPTrajectory(foh(t_init{4},repmat([l0;l0(1:2)],1,N(4))));
end

traj_opt = traj_opt.setSolverOptions('snopt','print','snopt.out');
traj_opt = traj_opt.setSolverOptions('snopt','MajorIterationsLimit',200);
traj_opt = traj_opt.setSolverOptions('snopt','MinorIterationsLimit',50000);
traj_opt = traj_opt.setSolverOptions('snopt','IterationsLimit',500000);


% lz_inds = reshape([traj_opt.l_inds(2:2:end,:) traj_opt.lc_inds(2:2:end,:)],[],1);
% lx_inds = reshape([traj_opt.l_inds(1:2:end,:) traj_opt.lc_inds(1:2:end,:)],[],1);
% nlz = length(lz_inds);

% traj_opt = traj_opt.addConstraint(BoundingBoxConstraint(zeros(nlz,1),inf(nlz,1)),lz_inds);
% A_fric = [eye(nlz) eye(nlz);eye(nlz) -eye(nlz)];
% traj_opt = traj_opt.addConstraint(LinearConstraint(zeros(2*nlz,1),inf(2*nlz,1),A_fric),[lz_inds;lx_inds]);

% traj_opt = traj_opt.setCheckGrad(true);
for i=1:length(N)
  traj_opt = traj_opt.addModeRunningCost(i,@running_cost_fun);
end
% traj_opt = traj_opt.addModeRunningCost(2,@running_cost_fun);
% traj_opt = traj_opt.addModeRunningCost(3,@running_cost_fun);
% traj_opt = traj_opt.addModeRunningCost(4,@running_cost_fun);
% traj_opt = traj_opt.addFinalCost(@final_cost_fun);


traj_opt = traj_opt.compile();
[xtraj,utraj,ltraj,z,F,info] = traj_opt.solveTraj(t_init,traj_init);



  function [f,df] = running_cost_fun(h,x,u)
    R = 1;
    f = u'*R*u;
    df = [0 zeros(1,20) 2*u'*R];
  end

end