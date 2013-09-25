classdef WorldFixedOrientConstraint < MultipleTimeKinematicConstraint
  % Constrain a certain body to be in a fixed orientation for a time
  % interval
  % @param robot           -- A RigidBodyManipulator or a
  %                           TimeSteppingRigidBodyManipulator
  % @param body            -- An int scalar, the body index
  % @param tspan           -- Optional input, a 1x2 double array. The time
  %                           span of this constraint being active. Default
  %                           is [-inf inf];
  properties
    body
    body_name
  end
  
  methods
    function obj = WorldFixedOrientConstraint(robot,body,tspan)
      if(nargin == 2)
        tspan = [-inf inf];
      end
      ptr = constructPtrWorldFixedOrientConstraintmex(robot.getMexModelPtr,body,tspan);
      obj = obj@MultipleTimeKinematicConstraint(robot,tspan);
      sizecheck(body,[1,1]);
      if(~isnumeric(body))
        error('Drake:WorldFixedPositionConstraint: body must be an integer');
      end
      obj.body = floor(body);
      obj.body_name = obj.robot.getBody(obj.body).linkname;
      obj.mex_ptr = ptr;
    end
    
    function num = getNumConstraint(obj,t)
      if(obj.isTimeValid(t))
        num = 1;
      else
        num = 0;
      end
    end
    
    function [c,dc] = eval(obj,t,q)
      if(obj.isTimeValid(t))
        n_breaks = size(t,2);
        nq = obj.robot.getNumDOF();
        sizecheck(q,[nq,n_breaks]);
        quat = zeros(4,n_breaks);
        if(nargout == 2)
          J = zeros(4*n_breaks,nq);
        end
        for i = 1:n_breaks
          kinsol = doKinematics(obj.robot,q(:,i),false,false);
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
          dc = sum(reshape(permute(reshape((bsxfun(@times,ones(1,nq),reshape(dcdquat,[],1)).*J)',nq,4,n_breaks),[2,1,3]),4,nq*n_breaks),1);
        end
      else
        c = [];
        dc = [];
      end
    end
    
    function [lb,ub] = bounds(obj,t)
      if(obj.isTimeValid(t))
        n_breaks = length(t);
        lb = n_breaks;
        ub = n_breaks;
      else
        lb = [];
        ub = [];
      end
    end
    
    function name_str = name(obj,t)
      if(obj.isTimeValid(t))
        name_str = {sprintf('World fixed orientation constraint for %s',obj.body_name)};
      else
        name_str = {};
      end
    end
    
    function ptr = constructPtr(varargin)
      ptr = constructPtrWorldFixedOrientConstraintmex(varargin{:});
    end
  end
end