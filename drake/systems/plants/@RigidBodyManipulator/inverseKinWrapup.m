function [q_seed,q_nom,constraint,ikoptions] = inverseKinWrapup(obj,q0,varargin)
% [q_seed,q_nom,constraint,ikoptions] =
% inverseKinWrapup(obj,q0,
%                 body1,body1_pts,body1_pos,...
%                 body1,body2_pts,body2_pos,...
%                 body1,body3_pts,body3_pos,...
%                 ...
%                 bodyN,bodyN_pts,bodyN_pos,...
%                 options
%                 use_mex_constraint)
% The input to the function is the same input list used in the deprecated
% IK call, except the last one, which determines whether to construct the
% constraint in mex only or not.
% @param obj          The robot
% @param q0           The seed guess
% @param body         The body index
% @param body_pts     Points on the body
% @param body_pos     The position of the body_pts, can also include gaze
%                     constraint
% @param options      A struct we used before in the IK call
% @param use_mex_constraint 
%                     A boolean flag, set to true, then all the kinematic
%                     constraint would be a mex object. This input is
%                     optional
%
% This wraps up the old inverseKin interface to the new interface
robotnum = 1;
q_seed = q0;
if(islogical(varargin{end}))
  use_mex_constraint = varargin{end};
  options = varargin{end-1};
  varargin = varargin(1:end-2);
elseif isstruct(varargin{end}) 
  use_mex_constraint = false;
  options = varargin{end};
  varargin=varargin(1:end-1);
else
  use_mex_constraint = false;
  options = struct();
end
if isfield(options,'q_nom') q_nom = options.q_nom; else q_nom = q0; end
if isfield(options,'Q') Q = options.Q; else Q = eye(obj.num_positions); end
if(~isfield(options,'jointLimitMin')) options.jointLimitMin = obj.joint_limit_min; end
if(~isfield(options,'jointLimitMax')) options.jointLimitMax = obj.joint_limit_max; end
if(isfield(options,'quasiStaticFlag'))
    quasiStaticFlag = logical(options.quasiStaticFlag);
else
    quasiStaticFlag = false;
end
if(isfield(options,'shrinkFactor'))
    shrinkFactor = options.shrinkFactor;
else
    shrinkFactor = 0.95;
end
nq = obj.getNumPositions();
ikoptions = IKoptions(obj);
ikoptions = ikoptions.setQ(Q);
qsc = [];
if(use_mex_constraint)
  pc = constructRigidBodyConstraint(RigidBodyConstraint.PostureConstraintType,true,obj.getMexModelPtr);
  pc = updatePtrRigidBodyConstraintmex(pc,'bounds',(1:nq)',options.jointLimitMin,options.jointLimitMax);
else
  pc = PostureConstraint(obj);
  pc = pc.setJointLimits((1:nq)',options.jointLimitMin,options.jointLimitMax);
end
constraint = {pc};

i = 1;
while i < length(varargin)
  if (isa(varargin{i},'RigidBody')) varargin{i} = find(obj.body==varargin{i},1); end
  body_ind = varargin{i};
  if(body_ind == 0)
    pos = varargin{i+1};
    kc_cell = wrapDeprecatedConstraint(obj,body_ind,[0;0;0],pos,struct('use_mex',use_mex_constraint,'robotnum',robotnum));
    constraint = [constraint,kc_cell];
    i = i+2;
  else
    pts = varargin{i+1};
    pos = varargin{i+2};
    [kc_cell,qsc_pts] = wrapDeprecatedConstraint(obj,body_ind,pts,pos,struct('use_mex',use_mex_constraint,'robotnum',robotnum));
    constraint = [constraint, kc_cell];
    if(~isempty(qsc_pts))
      if(isempty(qsc))
        if(use_mex_constraint)
          qsc = constructRigidBodyConstraint(RigidBodyConstraint.QuasiStaticConstraintType,true,obj.getMexModelPtr);
          updatePtrRigidBodyConstraintmex(qsc,'active',quasiStaticFlag);
          updatePtrRigidBodyConstraintmex(qsc,'factor',shrinkFactor);
        else
          qsc = QuasiStaticConstraint(obj);
          qsc = qsc.setActive(quasiStaticFlag);
          qsc = qsc.setShrinkFactor(shrinkFactor);
        end
      end
      if(use_mex_constraint)
        updatePtrRigidBodyConstraintmex(qsc,'contact',body_ind,qsc_pts);
      else
        qsc = qsc.addContact(body_ind,qsc_pts);
      end
    end
    i = i+3;
  end
end
if(~isempty(qsc))
  constraint = [constraint,{qsc}];
end
end