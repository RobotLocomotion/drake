function [p,v,xtraj,utraj,ltraj,z,F,info,traj_opt] = testConstrainedDircol(xtraj,utraj,ltraj)
rng(0)
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:WeldedLinkInd');
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
options.terrain = RigidBodyFlatTerrain();
options.floating = true;
options.ignore_self_collisions = true;
options.use_new_kinsol = true;

p = PlanarRigidBodyManipulator('../urdf/atlas_simple_planar_contact.urdf',options);



v = p.constructVisualizer();
% v = [];
N = 10;
T = .4;




% traj_opt = traj_opt.addStateConstraint(BoundingBoxConstraint(x0,x0),1);

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
%  x0(11:20) = 0;
 
 xf = [    0.1321
    0.9324
    0.1876
   -0.1767
    0.1004
   -0.1112
    0.1595
   -0.6579
    0.4326
   -0.0929
    0.5832
   -0.0729
   -0.0005
    0.7547
   -0.0350
   -0.7192
   -0.2887
    1.0315
   -3.0000
    2.4788 ];
 
%  % pin the right foot
% world_body = 1;
% foot_body = 7;
% [phi,normal,xA,xB,idxA,idxB] = p.collisionDetect(x0(1:10));
% assert(isequal(idxB(1:2),[foot_body foot_body])); % check body
% toe_fun = drakeFunction.kinematic.WorldPosition(p,foot_body,xB([1 3],1:2));
% world_pos = reshape(xA([1 3],1:2),[],1);
% 
% world_pos = zeros(4,1);
% 
% foot_constraint = DrakeFunctionConstraint(world_pos,world_pos,toe_fun);
% foot_constraint.grad_level = 2;
% 
% p = p.addPositionEqualityConstraint(foot_constraint);
% 
% to_options.relative_constraints = [true;false;true;false];
% 
% traj_opt = ConstrainedDircolTrajectoryOptimization(p,N,[0 T],to_options);
to_options.contact_q0 = x0(1:10);
to_options.lambda_bound = 1e3;
traj_opt = ContactConstrainedDircolTrajectoryOptimization(p,N,[0 T],[1;2],to_options);

% l0 = [0;0;178/(178+82);0;0;82/(178+82)]*p.getMass*-p.gravity(3);
l0 = [0;897.3515;0;179.1489];

% traj_opt = traj_opt.addStateConstraint(BoundingBoxConstraint(zeros(10,1),zeros(10,1)),{1,N},11:20);

% traj_opt = traj_opt.addStateConstraint(BoundingBoxConstraint(-inf,.8),1,2);
% traj_opt = traj_opt.addStateConstraint(BoundingBoxConstraint(.9,inf),N,2);

traj_opt = traj_opt.addStateConstraint(BoundingBoxConstraint(x0 - [zeros(10,1);.1*ones(10,1)],x0+[zeros(10,1);.1*ones(10,1)]),1);

traj_opt = traj_opt.addStateConstraint(BoundingBoxConstraint(xf(2:10),xf(2:10)),N,2:10);

t_init = linspace(0,T,N);
if nargin < 3
traj_init.x = PPTrajectory(foh(t_init,repmat(x0,1,N)));
traj_init.u = PPTrajectory(foh(t_init,randn(7,N)));
traj_init.l = PPTrajectory(foh(t_init,repmat(l0,1,N)));
else
  traj_init.x = xtraj;
  traj_init.u= utraj;
  traj_init.l = ltraj;
% traj_init.l = PPTrajectory(foh(t_init,repmat(l0,1,N)));
end

traj_opt = traj_opt.setSolverOptions('snopt','print','snopt.out');
traj_opt = traj_opt.setSolverOptions('snopt','MajorIterationsLimit',200);
traj_opt = traj_opt.setSolverOptions('snopt','MinorIterationsLimit',50000);
traj_opt = traj_opt.setSolverOptions('snopt','IterationsLimit',500000);


lz_inds = reshape([traj_opt.l_inds(2:2:end,:) traj_opt.lc_inds(2:2:end,:)],[],1);
lx_inds = reshape([traj_opt.l_inds(1:2:end,:) traj_opt.lc_inds(1:2:end,:)],[],1);
nlz = length(lz_inds);

traj_opt = traj_opt.addConstraint(BoundingBoxConstraint(zeros(nlz,1),inf(nlz,1)),lz_inds);
A_fric = [eye(nlz) eye(nlz);eye(nlz) -eye(nlz)];
traj_opt = traj_opt.addConstraint(LinearConstraint(zeros(2*nlz,1),inf(2*nlz,1),A_fric),[lz_inds;lx_inds]);

% traj_opt = traj_opt.setCheckGrad(true);

traj_opt = traj_opt.addRunningCost(@running_cost_fun);
% traj_opt = traj_opt.addFinalCost(@final_cost_fun);

[xtraj,utraj,ltraj,z,F,info] = traj_opt.solveTraj(t_init,traj_init);



  function [f,df] = running_cost_fun(h,x,u)
    R = 1;
    f = u'*R*u;
    df = [0 zeros(1,20) 2*u'*R];
  end

end