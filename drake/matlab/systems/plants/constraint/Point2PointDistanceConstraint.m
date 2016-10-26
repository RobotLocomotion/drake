classdef Point2PointDistanceConstraint < SingleTimeKinematicConstraint
  % constrain the distance between ptA(:,i) on body A to ptB(:,i) on body B to be within the range of
  % [lb(i) ub(i)]. To avoid singularity, the constraint is distance^2 in
  % [lb^2 ub^2]
  properties(SetAccess = protected)
    body_a = struct('idx',[],'name','');
    body_b = struct('idx',[],'name','');
    ptA
    ptB
    dist_lb
    dist_ub
  end
  
  methods(Access = protected)
    function [c,dc] = evalValidTime(obj,kinsol)      
      nq = obj.robot.getNumPositions();
      if(obj.body_a.idx ~= 1)
        [posA,dposA] = forwardKin(obj.robot,kinsol,obj.body_a.idx,obj.ptA,0);
      else
        posA = obj.ptA;
        dposA = zeros(3*obj.num_constraint,nq);
      end
      if(obj.body_b.idx ~= 1)
        [posB,dposB] = forwardKin(obj.robot,kinsol,obj.body_b.idx,obj.ptB,0);
      else
        posB = obj.ptB;
        dposB = zeros(3*obj.num_constraint,nq);
      end
      d = posA-posB;
      dd = dposA-dposB;
      c = sum(d.*d,1)';
      dc = 2*sparse(reshape(bsxfun(@times,1:obj.num_constraint,ones(3,1)),[],1),...
        (1:3*obj.num_constraint)',reshape(d,[],1))*dd;
    end
  end
  
  methods
    function obj = Point2PointDistanceConstraint(robot,body_a,body_b,ptA,ptB,dist_lb,dist_ub,tspan)
      % @param robot          -- RigidBodyManipulator object
      % @param body_a         -- Either:
      %                             * An integer body index or frame id OR
      %                             * A string containing a body or frame name
      % @param body_b         -- Either:
      %                             * An integer body index or frame id OR
      %                             * A string containing a body or frame name
      % @param ptA            -- A 3xnpts vector. The location of ptA in body A frame
      % @param ptB            -- A 3xnpts vector. The location of ptB in body B frame
      % @param dist_lb        -- A 1xnpts nonnegative vector. the lower bound of the distance
      % @param dist_ub        -- A 1xnpts nonnegative vector. the upper bound of the distance
      % @param tspan          -- A 1x2 array, the time span of the constraint. Optional
      %                          argument, default is [-inf inf]
      if(nargin == 7)
        tspan = [-inf inf];
      end
      body_a_idx = robot.parseBodyOrFrameID(body_a);
      body_b_idx = robot.parseBodyOrFrameID(body_b);
      if(body_a_idx == body_b_idx)
        error('Point2PointDistanceConstraint:SameBodies',  ...
        'body_a and body_b should refer different frames or bodies');
      end
      
      obj = obj@SingleTimeKinematicConstraint(robot,tspan);
      obj.body_a.idx = body_a_idx;
      obj.body_b.idx = body_b_idx;
      obj.body_a.name = getBodyOrFrameName(obj.robot, obj.body_a.idx);
      obj.body_b.name = getBodyOrFrameName(obj.robot, obj.body_b.idx);
      if(~isnumeric(ptA))
        error('Point2PointDistanceConstraint: ptA should be numeric');
      end
      if(~isnumeric(ptB))
        error('Point2PointDistanceConstraint: ptB should be numeric');
      end
      npts = size(ptA,2);
      sizecheck(ptA,[3,npts]);
      sizecheck(ptB,[3,npts]);
      obj.ptA = ptA;
      obj.ptB = ptB;
      obj.num_constraint = npts;
      if(~isnumeric(dist_lb))
        error('Point2PointDistanceConstraint: lb should be numeric');
      end
      if(~isnumeric(dist_ub))
        error('Point2PointDistanceConstraint: ub should be numeric');
      end
      sizecheck(dist_lb,[1,npts]);
      sizecheck(dist_ub,[1,npts]);
      if(any(dist_lb>dist_ub))
        error('Point2PointDistanceConstraint: lb must be no larger than ub');
      end
      obj.dist_lb = dist_lb;
      obj.dist_ub = dist_ub;
      obj.type = RigidBodyConstraint.Point2PointDistanceConstraintType;
      if robot.getMexModelPtr~=0 && exist('constructPtrRigidBodyConstraintmex','file')
        obj.mex_ptr = constructPtrRigidBodyConstraintmex(RigidBodyConstraint.Point2PointDistanceConstraintType,robot.getMexModelPtr,body_a_idx,body_b_idx,ptA,ptB,dist_lb,dist_ub,tspan);
      end
    end
    
    function [lb,ub] = bounds(obj,t)
      if(obj.isTimeValid(t))
        lb = reshape(obj.dist_lb.*obj.dist_lb,[],1);
        ub = reshape(obj.dist_ub.*obj.dist_ub,[],1);
      else
        lb = [];
        ub = [];
      end
    end
    
    function name_str = name(obj,t)
      if(obj.isTimeValid(t))
        name_str = cell(obj.num_constraint,1);
        for i = 1:obj.num_constraint
          name_str{i} = sprintf('distance between %s pt %d to %s pt %d',obj.body_a.name,i,obj.body_a.name,i);
        end
      else
        name_str = {};
      end
    end
    
    function joint_idx = kinematicsPathJoints(obj)
      [~,joint_path] = obj.robot.findKinematicPath(obj.body_a.idx,obj.body_b.idx);
      joint_idx = vertcat(obj.robot.body(joint_path).position_num)';
    end
  end
end
