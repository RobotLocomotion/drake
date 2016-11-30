classdef MinDistanceConstraint < SingleTimeKinematicConstraint
  % Constrains the closest distance between all bodies to be greater
  % than  min_distance. 
  %
  % @param min_distance    -- a scalar, the lower bound of the distance
  % @param tspan   -- a 1x2 vector, the time span of the constraint being
  %                   active
  properties
    min_distance
    active_collision_options = struct('body_idx',{},'collision_groups',{});
  end

  methods(Access=protected)
    function [c,dc] = evalValidTime(obj,kinsol)
      if ~kinsol.mex
        error('Drake:MinDistanceConstraint:evalValidTime:NoMexDynamics', ...
          'This method requires a kinsol generated with mex enabled');
      else
        [c,dc] = evalLocal(obj,kinsol);
      end
    end

    function [c,dc] = evalLocal(obj,q_or_kinsol)
      if ~isstruct(q_or_kinsol)
        kinsol = obj.robot.doKinematics(q_or_kinsol);
      else
        kinsol = q_or_kinsol;
      end
      [c,dc] = smoothDistancePenaltymex(obj.robot.mex_model_ptr, kinsol.mex_ptr, obj.min_distance,obj.active_collision_options);
    end
  end

  methods
    function obj = MinDistanceConstraint(robot,min_distance,active_collision_options,tspan)
      if(nargin < 4)
        tspan = [-inf inf];
      end
      if nargin < 3, active_collision_options = struct(); end;
      sizecheck(min_distance,[1,1]);
      assert(min_distance>0);
      checkDependency('bullet');
      obj = obj@SingleTimeKinematicConstraint(robot,tspan);
      obj.type = RigidBodyConstraint.MinDistanceConstraintType;
      obj.min_distance = min_distance;
      if robot.getMexModelPtr~=0 && exist('constructPtrRigidBodyConstraintmex','file')
        obj.mex_ptr = constructPtrRigidBodyConstraintmex(RigidBodyConstraint.MinDistanceConstraintType,robot.getMexModelPtr,min_distance,active_collision_options,tspan);
      end
      obj.num_constraint = 1;
      obj.active_collision_options = active_collision_options;
    end

    function cnstr = generateConstraint(obj,varargin)
      cnstr = generateConstraint@SingleTimeKinematicConstraint(obj,varargin{:});
      if ~isempty(cnstr)
        cnstr{1} = setEvalHandle(cnstr{1},@(q,~) obj.evalLocal(q));
      end
    end

    function [scaled_dist,dscaled_dist_ddist] = scaleDistance(obj,dist)
      recip_min_dist = 1/obj.min_distance;
      scaled_dist = recip_min_dist*dist - 1;
      %scaled_dist = recip_min_dist*dist - 2;
      dscaled_dist_ddist = recip_min_dist*speye(size(dist,1));
    end

    function [cost, dcost_ddist] = penalty(obj,dist)
      % [cost, dcost_ddist] = penalty(obj,dist) applies a smooth hinge loss
      % element-wise to dist. This hinge loss is given by
      %
      % \f[
      % c = 
      % \begin{cases}
      %   -de^{\frac{1}{d}}, & d <   0  \\
      %   0,                & d \ge 0.
      % \end{cases}
      % \f]
      %           
      
      idx_neg = find(dist < 0);
      cost = zeros(size(dist));
      dcost_ddist = zeros(numel(dist));
      exp_recip_dist = exp(dist(idx_neg).^(-1));
      cost(idx_neg) = -dist(idx_neg).*exp_recip_dist;
      dcost_ddist(sub2ind(size(dcost_ddist),idx_neg,idx_neg)) = ...
        exp_recip_dist.*(dist(idx_neg).^(-1) - 1);
    end

    function obj = excludeCollisionGroups(obj, collision_geometry_group_names)
      if ischar(collision_geometry_group_names)
        collision_geometry_group_names = {collision_geometry_group_names};
      end

      typecheck(collision_geometry_group_names,'cell');
      if ~isfield(obj.active_collision_options,'collision_groups') || ...
          isempty(obj.active_collision_options.collision_groups)
        obj.active_collision_options.collision_groups = unique([obj.robot.body.collision_geometry_group_names]);
      end
      obj.active_collision_options.collision_groups = setdiff(obj.active_collision_options.collision_groups,collision_geometry_group_names);

      obj.mex_ptr.delete();
      obj.mex_ptr = constructPtrRigidBodyConstraintmex(RigidBodyConstraint.MinDistanceConstraintType,obj.robot.getMexModelPtr,obj.min_distance,obj.active_collision_options,obj.tspan);
    end

    function num = getNumConstraint(obj,t)
      if obj.isTimeValid(t)
        num = 1;
      else
        num = 0;
      end
    end

    function obj = updateRobot(obj,robot)
      obj.robot = robot;
      obj.mex_ptr = updatePtrRigidBodyConstraintmex(obj.mex_ptr,'robot',robot.getMexModelPtr);
    end

    function [lb,ub] = bounds(obj,t)
      if obj.isTimeValid(t)
        lb = 0;
        ub = 0;
      else
        lb = [];
        ub = [];
      end
    end

    function name_str = name(obj,t)
      name_str = repmat({'Minimum distance constraint'},obj.getNumConstraint(t),1);
    end
  end
end
