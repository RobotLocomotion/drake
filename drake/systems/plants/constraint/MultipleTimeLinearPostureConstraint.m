classdef MultipleTimeLinearPostureConstraint < RigidBodyConstraint
  % A linear constraint on the robot posture for multiple times

  methods
    function obj = MultipleTimeLinearPostureConstraint(robot)
      obj = obj@RigidBodyConstraint(RigidBodyConstraint.MultipleTimeLinearPostureConstraintCategory,robot);
    end

    function flag = isTimeValid(obj,t)
      % TODO(sam.creasey) Understand the callers of this function well enough to get rid of it.
      tspan = [-inf inf];
      flag = t>=tspan(1)&t<=tspan(end);
    end

    function [c,dc] = eval(obj,t,q)
      c = obj.feval(t,q);
      [iAfun,jAvar,A] = geval(obj,t);
      num_cnst = obj.getNumConstraint(t);
      dc = sparse(iAfun,jAvar,A,num_cnst,numel(q));
    end

    function cnstr = generateConstraint(obj,t)
      [iAfun,jAvar,A] = obj.geval(t);
      num_cnstr = obj.getNumConstraint(t);
      nq = obj.robot.getNumPositions();
      [lb,ub] = obj.bounds(t);
      cnstr = {LinearConstraint(lb,ub,sparse(iAfun,jAvar,A,num_cnstr,length(t)*nq))};
      name_str = obj.name(t);
      cnstr{1} = cnstr{1}.setName(name_str);
    end
  end

  methods(Abstract)
    num = getNumConstraint(obj,t);
    c = feval(obj,t,q);
    [iAfun,jAvar,A] = geval(obj,t);
    % @retval iAfun,jAvar,A  - The sparse matrix determined by
    % sparse(iAfun,jAvar,A,num_constraint,numel(q)) is the gradient of c w.r.t q
    [lb,ub] = bounds(obj,t);
    name_str = name(obj,t);
  end
end
