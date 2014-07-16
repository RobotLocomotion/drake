function interpolated_constraint = generateInterpolatedMinDistanceConstraint(constraint,interpolation_parameter)
  % interpolated_constraint = ...
  %   generateInterpolatedMinDistanceConstraint(constraint, ...
  %     interpolation_parameter) returns cell-array of
  % FunctionHandleConstraint objects that enforce the given
  % MinDistanceConstraint at interpolated points between two knots:
  %
  %   q_interp(i) = (1 - interpolation_parameter(i)) *q1 
  %                 + interpolation_parameter(i)*q2
  %
  % @param constraint               -- MinDistanceConstraint object
  % @param interpolation parameter  -- Numeric vector with values
  %                                    between 0 and 1
  %
  % @retval interpolated_constraint -- Cell-array of 
  %                                    FunctionHandleConstraint objects
  [lb,ub] = constraint.bounds(0);
  interpolated_constraint = cell(size(interpolation_parameter));
  for i = 1:numel(interpolation_parameter)
    interpolated_constraint{i} = FunctionHandleConstraint(lb, ub, 2*constraint.robot.getNumPositions(),  ...
      @(q1,q2)interpEvalWrapper(constraint,interpolation_parameter(i),q1,q2));
  end
end

function [c,dc] = interpEvalWrapper(constraint,interpolation_parameter,q1,q2)
  nq = numel(q1);
  [c,dc_dq_interp] = constraint.eval(0,(1-interpolation_parameter)*q1+interpolation_parameter*q2);
  dc = dc_dq_interp*[(1-interpolation_parameter)*eye(nq),interpolation_parameter*eye(nq)];
end
