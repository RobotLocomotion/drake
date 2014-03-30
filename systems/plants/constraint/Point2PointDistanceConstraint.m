classdef Point2PointDistanceConstraint < SingleTimeKinematicConstraint
  % constrain the distance between ptA(:,i) on bodyA to ptB(:,i) on bodyB to be within the range of
  % [lb(i) ub(i)]. To avoid singularity, the constraint is distance^2 in
  % [lb^2 ub^2]
  % @param robot          -- 
  % @param bodyA          -- A scalar, the index of bodyA. If it is the world, set bodyA=0
  % @param bodyB          -- A scalar, the index of bodyB. If it is the world, set bodyB=0
  % @param ptA            -- A 3xnpts vector. The location of ptA in bodyA frame
  % @param ptB            -- A 3xnpts vector. The location of ptB in bodyB frame
  % @param dist_lb        -- A 1xnpts nonnegative vector. the lower bound of the distance
  % @param dist_ub        -- A 1xnpts nonnegative vector. the upper bound of the distance
  % @param tspan          -- A 1x2 array, the time span of the constraint. Optional
  %                          argument, default is [-inf inf]
  properties(SetAccess = protected)
    bodyA
    bodyB
    ptA
    ptB
    dist_lb
    dist_ub
    bodyA_name;
    bodyB_name;
  end
  
  methods(Access = protected)
    function [c,dc] = evalValidTime(obj,kinsol)      
      nq = obj.robot.getNumDOF();
      if(obj.bodyA ~= 0)
        [posA,dposA] = forwardKin(obj.robot,kinsol,obj.bodyA,obj.ptA,0);
      else
        posA = obj.ptA;
        dposA = zeros(3*obj.num_constraint,nq);
      end
      if(obj.bodyB ~= 0)
        [posB,dposB] = forwardKin(obj.robot,kinsol,obj.bodyB,obj.ptB,0);
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
    function obj = Point2PointDistanceConstraint(robot,bodyA,bodyB,ptA,ptB,dist_lb,dist_ub,tspan)
      if(nargin == 7)
        tspan = [-inf inf];
      end
      mex_ptr = constructPtrRigidBodyConstraintmex(RigidBodyConstraint.Point2PointDistanceConstraintType,robot.getMexModelPtr,bodyA,bodyB,ptA,ptB,dist_lb,dist_ub,tspan);
      obj = obj@SingleTimeKinematicConstraint(robot,tspan);
      if(~isnumeric(bodyA))
        error('Point2PointDistanceConstraint: bodyA should be numeric');
      end
      if(~isnumeric(bodyB))
        error('Point2PointDistanceConstraint: bodyB should be numeric');
      end
      sizecheck(bodyA,[1,1]);
      sizecheck(bodyB,[1,1]);
      if(bodyA>robot.getNumBodies || bodyA <0)
        error('Point2PointDistanceConstraint: bodyA should be within [0 robot.getNumBodies]');
      end
      if(bodyB>robot.getNumBodies || bodyB <0)
        error('Point2PointDistanceConstraint: bodyB should be within [0 robot.getNumBodies]');
      end
      if(bodyA == bodyB)
        error('Point2PointDistanceConstraint: bodyA and bodyB should be different');
      end
      obj.bodyA = floor(bodyA);
      obj.bodyB = floor(bodyB);
      if(obj.bodyA ~= 0)
        obj.bodyA_name = obj.robot.getBody(obj.bodyA).linkname;
      else
        obj.bodyA_name = 'World';
      end
      if(obj.bodyB ~= 0)
        obj.bodyB_name = obj.robot.getBody(obj.bodyB).linkname;
      else
        obj.bodyB_name = 'World';
      end
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
      obj.mex_ptr = mex_ptr;
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
          name_str{i} = sprintf('distance between %s pt %d to %s pt %d',obj.bodyA_name,i,obj.bodyB_name,i);
        end
      else
        name_str = {};
      end
    end
    
    function joint_idx = kinematicsPathJoints(obj)
      if(obj.bodyA == 0)
        [~,joint_path] = obj.robot.findKinematicPath(1,obj.bodyB);
      elseif(obj.bodyB == 0)
        [~,joint_path] = obj.robot.findKinematicPath(obj.bodyA,1);
      else
        [~,joint_path] = obj.robot.findKinematicPath(obj.bodyA,obj.bodyB);
      end
      joint_idx = vertcat(obj.robot.body(joint_path).dofnum)';
    end
  end
end