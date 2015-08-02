function [q,info] = approximateIK(obj,q_seed,q_nom,varargin)
% Please refer to the interface of inverseKin
% The inputs are the same as inverseKin, except current implementation does
% not accept QuasiStaticConstraint object.
% @retval q          the IK solution posture
% @retval info       0 on sucess, 1 on failure

use_mex = true;
if(~isa(varargin{end},'IKoptions'))
  ikoptions = IKoptions(obj);
  varargin = varargin(1:end-1);
else
  ikoptions = varargin{end};
end
if(use_mex && checkDependency('gurobi_mex'))
  num_constraint = length(varargin)-1;
  constraint_ptr_cell = cell(1,num_constraint);
  for i = 1:num_constraint
    if(isa(varargin{i},'DrakeConstraintMexPointer'))
      constraint_ptr_cell{i} = varargin{i};
    elseif(isa(varargin{i},'RigidBodyConstraint'))
      constraint_ptr_cell{i} = varargin{i}.mex_ptr;
    else
      error('The input has to be a RigidBodyConstraint object');
    end
  end
  [q,info] = approximateIKmex(obj.mex_model_ptr,q_seed,q_nom,constraint_ptr_cell{:},ikoptions.mex_ptr);
else
checkDependency('gurobi');
t = [];
kc_cell = {};
[joint_min,joint_max] = obj.getJointLimits();
for j = 1:length(varargin)-1
  if(isa(varargin{j},'SingleTimeKinematicConstraint'))
    kc_cell = [kc_cell,{varargin{j}}];
  elseif(isa(varargin{j},'PostureConstraint'))
    [lb,ub] = varargin{j}.bounds([]);
    joint_min = max([joint_min lb],[],2);
    joint_max = min([joint_max ub],[],2);
    if(any(joint_min>joint_max))
      error('Joint maximum should be no smaller than joint minimum, check if the joint limits are consistent');
    end
  end
end
model.Q = sparse(ikoptions.Q);
model.obj = -2*q_nom'*model.Q;
A = [];
model.rhs = [];
model.sense = [];
kinsol = doKinematics(obj,q_seed,false,false);
for j = 1:length(kc_cell)
  kc = kc_cell{j};
  [lb,ub] = kc.bounds(t);
  [c,dc] = kc.eval(t,kinsol);
  equal_idx = lb==ub;
  lb_idx = isinf(ub);
  ub_idx = isinf(lb);
  both_idx = ~(equal_idx|lb_idx|ub_idx);
  cnst = [c(equal_idx);c(lb_idx);c(ub_idx);c(both_idx);c(both_idx)];
  dcnst = [dc(equal_idx,:);dc(lb_idx,:);dc(ub_idx,:);dc(both_idx,:);dc(both_idx,:)];
  model.rhs = [model.rhs;[ub(equal_idx);lb(lb_idx);ub(ub_idx);lb(both_idx);ub(both_idx)]-cnst+dcnst*q_seed];
  A = [A;dcnst];
  model.sense = [model.sense;repmat('=',sum(equal_idx),1);repmat('>',sum(lb_idx),1);repmat('<',sum(ub_idx),1);...
    repmat('>',sum(both_idx),1);repmat('<',sum(both_idx),1)];
end
model.ub = joint_max;
model.lb = joint_min;
model.A = sparse(A);

params.outputflag = 0;
params.method = 2;
params.presolve = 0;
params.bariterlimit = 20;
params.barhomogeneous = 0;
params.barconvtol = 0.005;

result = gurobi(model,params);
info = ~strcmp(result.status,'OPTIMAL');
if (info)
  q = q0;
else
  q = result.x;
end
end
end
