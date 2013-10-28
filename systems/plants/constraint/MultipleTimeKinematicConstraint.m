classdef MultipleTimeKinematicConstraint < Constraint
  % A abstract class, that its eval function takes multiple time points as
  % the input, instead of being evluated at a single time.
  properties(SetAccess = protected)
    tspan % a 1x2 vector
    robot
    mex_ptr
  end
  
  properties(Constant)
    WorldFixedPositionConstraint = 1;
    WorldFixedOrientConstraint = 2;
    WorldFixedBodyPoseConstraint = 3;
  end
    
  methods
    function obj = MultipleTimeKinematicConstraint(robot,tspan)
      obj = obj@Constraint(Constraint.MultipleTimeKinematicConstraintType);
      if(nargin<2)
        tspan = [-inf,inf];
      end
      if(isempty(tspan))
        tspan = [-inf,inf];
      end
      if(tspan(1)>tspan(end))
        error('tspan(1) should be no larger than tspan(end)')
      end
      obj.tspan = [tspan(1) tspan(end)];
      obj.robot = robot;
    end
    
    function flag = isTimeValid(obj,t)
      n_breaks = size(t,2);
      if(n_breaks <=1)
        error('Drake:WorldFixedPositionConstraint: t must have more than 1 entry');
      end
      flag = t>=obj.tspan(1)&t<=obj.tspan(end);
    end
    
    function tspan = getTspan(obj)
      tspan = obj.tspan;
    end
    
    function [c,dc] = eval(obj,t,q)
      valid_t_idx = obj.isTimeValid(t);
      valid_t = t(valid_t_idx);
      valid_q = q(:,valid_t_idx);
      nq = obj.robot.getNumDOF();
      if(length(valid_t)>=2)
        num_valid_t = size(valid_t,2);
        [c,dc_valid] = eval_valid(obj,valid_t,valid_q);
        nc = obj.getNumConstraint(t);
        dc = zeros(nc,nq,length(t));
        dc_valid = reshape(dc_valid,nc,nq,num_valid_t);
        dc(:,:,valid_t_idx) = dc_valid;
        dc = reshape(dc,nc,nq*length(t));
      else
        c = [];
        dc = [];
      end
    end
  end
  
  methods(Abstract)
    % t is an array instead of a scalar
    num = getNumConstraint(obj,t);
    [c,dc] = eval_valid(obj,valid_t,valid_q);
    [lb,ub] = bounds(obj,t)
    name_str = name(obj,t)
    ptr = constructPtr(varargin);
  end
end