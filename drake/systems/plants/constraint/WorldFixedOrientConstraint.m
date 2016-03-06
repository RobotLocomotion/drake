classdef WorldFixedOrientConstraint < MultipleTimeKinematicConstraint
  % Constrain a certain body to be in a fixed orientation for a time
  % interval
  % @param robot           -- A RigidBodyManipulator or a
  %                           TimeSteppingRigidBodyManipulator
  % @param body            -- An int scalar, the body index
  % @param tspan           -- Optional input, a 1x2 double array. The time
  %                           span of this constraint being active. Default
  %                           is [-inf inf];
  properties(SetAccess = protected)
    body
    body_name
  end
  
  methods(Access = protected)
    function [c,dc_valid] = evalValidTime(obj,valid_kinsol_cell)
      num_valid_t = length(valid_kinsol_cell);
      nq = obj.robot.getNumPositions();
      quat = zeros(4,num_valid_t);
      if(nargout == 2)
        J = zeros(4*num_valid_t,nq);
      end
      for i = 1:num_valid_t
        kinsol = valid_kinsol_cell{i};
        if(nargout == 1)
          pos_tmp = forwardKin(obj.robot,kinsol,obj.body,[0;0;0],2);
        elseif(nargout == 2)
          [pos_tmp,J_tmp] = forwardKin(obj.robot,kinsol,obj.body,[0;0;0],2);
          J((i-1)*4+(1:4),:) = J_tmp(4:7,:);
        end
        quat(:,i) = pos_tmp(4:7);
      end
      quat2 = [quat(:,2:end) quat(:,1)];
      c1 = sum(quat.*quat2,1);
      c = sum(c1.^2);
      if(nargout == 2)
        % [dcdquat1' dcdquat2' ...dcdquat_n_breaks'];
        dcdquat = (bsxfun(@times,ones(4,1),2*c1).*quat2+bsxfun(@times,ones(4,1),2*[c1(end) c1(1:end-1)]).*[quat(:,end) quat(:,1:end-1)]);
        dc_valid = sum(reshape(permute(reshape((bsxfun(@times,ones(1,nq),reshape(dcdquat,[],1)).*J)',nq,4,num_valid_t),[2,1,3]),4,nq*num_valid_t),1);
      end
    end
  end
  
  
  methods
    function obj = WorldFixedOrientConstraint(robot,body,tspan)
      if(nargin == 2)
        tspan = [-inf inf];
      end
      ptr = constructPtrRigidBodyConstraintmex(RigidBodyConstraint.WorldFixedOrientConstraintType,robot.getMexModelPtr,body,tspan);
      obj = obj@MultipleTimeKinematicConstraint(robot,tspan);
      sizecheck(body,[1,1]);
      if(~isnumeric(body))
        error('Drake:WorldFixedPositionConstraint: body must be an integer');
      end
      obj.body = floor(body);
      obj.body_name = obj.robot.getBody(obj.body).linkname;
      obj.type = RigidBodyConstraint.WorldFixedOrientConstraintType;
      obj.mex_ptr = ptr;
    end
    
    function num = getNumConstraint(obj,t)
      valid_t = t(obj.isTimeValid(t));
      if(length(valid_t)>=2)
        num = 1;
      else
        num = 0;
      end
    end
    
    
    function [lb,ub] = bounds(obj,t,N)
      % [lb,ub] = bounds(obj,t) returns the upper and lower bounds for this
      % constraint at all valid times given in t.
      %
      % [lb,ub] = bounds(obj,[],N) returns the upper and lower bounds for this
      % constraint for N valid times
      %
      % @param obj  -- WorldFixedOrientationConstraint object
      % @param t    -- Vector of times
      % @param N    -- Integer number of time points
      if isempty(t);
        if(N>=2)
          lb = [0;N];
          ub = [0;N];
        else
          lb = [];
          ub = [];
        end
      else
        valid_t = t(obj.isTimeValid(t));
        if(length(valid_t)>=2)
          num_valid_t = length(valid_t);
          lb = num_valid_t;
          ub = num_valid_t;
        else
          lb = [];
          ub = [];
        end
      end
    end
    
    function name_str = name(obj,t)
      valid_t = t(obj.isTimeValid(t));
      if(length(valid_t)>=2)
        name_str = {sprintf('World fixed orientation constraint for %s',obj.body_name)};
      else
        name_str = {};
      end
    end
    
    function joint_idx = kinematicPathJoints(obj)
      [~,joint_path] = obj.robot.findKinematicPath(1,obj.body);
      joint_idx = vertcat(obj.robot.body(joint_path).position_num)';
    end
  end
end
