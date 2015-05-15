function [p,v,xtraj,utraj,z,F,info,traj_opt] = testConstrainedDircol

warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:WeldedLinkInd');
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
options.terrain = RigidBodyFlatTerrain();
options.floating = true;
options.ignore_self_collisions = true;

p = PlanarRigidBodyManipulator('../urdf/atlas_simple_planar_contact.urdf',options);

v = p.constructVisualizer();
N = {10,10};
duration = {[.2 .5],[.2 .2]};


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

traj_opt = ConstrainedHybridTrajectoryOptimization(p,{[1;2],[1;2;3;4]},N,duration);

l0 = [0;897.3515;0;179.1489];

traj_opt = traj_opt.addModeStateConstraint(1,BoundingBoxConstraint(x0 - [zeros(10,1);.1*ones(10,1)],x0+[zeros(10,1);.1*ones(10,1)]),1);

t_init{1} = linspace(0,.4,N{1});
traj_init{1}.x = PPTrajectory(foh(t_init,repmat(x0,1,N)));
traj_init{1}.u = PPTrajectory(foh(t_init,randn(7,N)));
traj_init{1}.l = PPTrajectory(foh(t_init,repmat(l0,1,N)));

t_init{2} = linspace(0,.2,N{2});
traj_init{2}.x = PPTrajectory(foh(t_init,repmat(x0,1,N)));
traj_init{2}.u = PPTrajectory(foh(t_init,randn(7,N)));
traj_init{2}.l = PPTrajectory(foh(t_init,repmat(l0,1,N)));

traj_opt = traj_opt.setSolverOptions('snopt','print','snopt.out');
traj_opt = traj_opt.setSolverOptions('snopt','MajorIterationsLimit',500);
traj_opt = traj_opt.setSolverOptions('snopt','MinorIterationsLimit',500000);
traj_opt = traj_opt.setSolverOptions('snopt','IterationsLimit',500000);


% lz_inds = reshape([traj_opt.l_inds(2:2:end,:) traj_opt.lc_inds(2:2:end,:)],[],1);
% lx_inds = reshape([traj_opt.l_inds(1:2:end,:) traj_opt.lc_inds(1:2:end,:)],[],1);
% nlz = length(lz_inds);

% traj_opt = traj_opt.addConstraint(BoundingBoxConstraint(zeros(nlz,1),inf(nlz,1)),lz_inds);
% A_fric = [eye(nlz) eye(nlz);eye(nlz) -eye(nlz)];
% traj_opt = traj_opt.addConstraint(LinearConstraint(zeros(2*nlz,1),inf(2*nlz,1),A_fric),[lz_inds;lx_inds]);

% traj_opt = traj_opt.setCheckGrad(true);

traj_opt = traj_opt.addRunningCost(@running_cost_fun);
% traj_opt = traj_opt.addFinalCost(@final_cost_fun);

[xtraj,utraj,z,F,info] = traj_opt.solveTraj(t_init,traj_init);



  function [f,df] = running_cost_fun(h,x,u)
    R = 1;
    f = u'*R*u;
    df = [0 zeros(1,20) 2*u'*R];
  end

end