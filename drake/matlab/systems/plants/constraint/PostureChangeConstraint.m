classdef PostureChangeConstraint < MultipleTimeLinearPostureConstraint
  % This constrains that the change of joint i within time tspan must be within [lb,ub]
  % @param robot
  % @param joint_idx             A nx1 vector, the index of the joints
  % @param lb_change             A nx1 vector, the lower bound of the change of joint angles
  % @param ub_change             A nx1 vector, the upper bound of the change of joint angles
  % @param tspan                 A 1x2 vector, the time span. Optional argument. Default
  %                              is [-inf inf]
  properties(SetAccess = protected)
    joint_ind
    lb_change
    ub_change
  end
  
  methods(Access=protected)
    function obj = setJointChangeBounds(obj,joint_ind,lb_change,ub_change)
      if(any(~isnumeric(joint_ind)))
        error('The joint index must all be numeric');
      end
      if(any(joint_ind<=0)||any(joint_ind>obj.robot.getNumPositions))
        error('The joint index must be within [1 nq]');
      end
      sizecheck(joint_ind,[nan,1]);
      obj.joint_ind = joint_ind;
      sizecheck(lb_change,[nan,1]);
      sizecheck(ub_change,[nan,1]);
      if(any(size(joint_ind)~=size(lb_change)) || any(size(joint_ind)~=size(ub_change)))
        error('The size of input arguments do not match');
      end
      [joint_lb,joint_ub] = obj.robot.getJointLimits();
      lb_change = max([joint_lb(joint_ind)-joint_ub(joint_ind) lb_change],[],2);
      ub_change = min([joint_ub(joint_ind)-joint_lb(joint_ind) ub_change],[],2);
      if any(lb_change>ub_change)
        error('Drake;MultipleTimePostureConstraint: lb_change must be no larger than ub change');
      end
      obj.lb_change = lb_change;
      obj.ub_change = ub_change;
    end
  end
  
  methods
    function obj = PostureChangeConstraint(robot,joint_ind,lb_change,ub_change,tspan)
      if(nargin<5)
        tspan = [-inf inf];
      end
      ptr = constructPtrRigidBodyConstraintmex(RigidBodyConstraint.PostureChangeConstraintType,robot.getMexModelPtr,joint_ind,lb_change,ub_change,tspan);
      obj = obj@MultipleTimeLinearPostureConstraint(robot,tspan);
      obj = obj.setJointChangeBounds(joint_ind,lb_change,ub_change);
      obj.type = RigidBodyConstraint.PostureChangeConstraintType;
      obj.mex_ptr = ptr;
    end
    
    
    function num = getNumConstraint(obj,t)
      valid_flag = obj.isTimeValid(t);
      num_valid_t = sum(valid_flag);
      if(num_valid_t>=2)
        num = length(obj.joint_ind)*(sum(valid_flag)-1);
      else
        num = 0;
      end
    end
    
    function c = feval(obj,t,q)
      % c = [q(obj.joint_ind,2)-q(obj.joint_ind,1);
      % q(obj.joint_ind,3)-q(obj.joint_ind,1);...;q(obj.joint_ind,n)-q(obj.joint_ind,1)];
      valid_flag = obj.isTimeValid(t);
      num_valid_t = sum(valid_flag);
      if(num_valid_t>=2)
        valid_q = q(:,valid_flag);
        c = valid_q(obj.joint_ind,2:end)-bsxfun(@times,valid_q(obj.joint_ind,1),ones(1,num_valid_t-1));
        c = c(:);
      else
        c = [];
      end
    end
    
    function [iAfun,jAvar,A] = geval(obj,t)
      % sparse(iAfun,jAvar,A,num_constraint,length(q)) is the gradient of the function feval w.r.t q
      valid_flag = obj.isTimeValid(t);
      num_valid_t = sum(valid_flag);
      if(num_valid_t >=2)
        num_joints = length(obj.joint_ind);
        t_ind = (1:length(t));
        valid_t_ind = t_ind(valid_flag);
        nq = obj.robot.getNumPositions();
        first_valid_idx = find(valid_flag,1);
        iAfun = [(1:num_joints*(num_valid_t-1))';(1:num_joints*(num_valid_t-1))'];
        jAvar = [reshape(bsxfun(@times,(first_valid_idx-1)*nq+obj.joint_ind,ones(1,num_valid_t-1)),[],1);...
          reshape(bsxfun(@plus,(valid_t_ind(2:end)-1)*nq,obj.joint_ind),[],1)];
        A = [-ones(num_joints*(num_valid_t-1),1);ones(num_joints*(num_valid_t-1),1)];
      else
        iAfun = [];
        jAvar = [];
        A = [];
      end
    end
    
    function [lb,ub] = bounds(obj,t)
      valid_flag = obj.isTimeValid(t);
      num_valid_t = sum(valid_flag);
      if(num_valid_t>=2)
        lb = reshape(bsxfun(@times,obj.lb_change,ones(1,num_valid_t-1)),[],1);
        ub = reshape(bsxfun(@times,obj.ub_change,ones(1,num_valid_t-1)),[],1);
      else
        lb = [];
        ub = [];
      end
    end
    
    function name_str = name(obj,t)
      valid_flag = obj.isTimeValid(t);
      num_valid_t = sum(valid_flag);
      if(num_valid_t>=2)
        valid_t = t(valid_flag);
        nc = obj.getNumConstraint(t);
        name_str = cell(nc,1);
        cnstr_idx = 1;
        for i = 2:num_valid_t
          for j = 1:length(obj.joint_ind)
            name_str{cnstr_idx} = sprintf('Posture change for joint %d at time %5.3f',obj.joint_ind(j),valid_t(i));
            cnstr_idx = cnstr_idx+1;
          end
        end
      end
    end
    
  end
end