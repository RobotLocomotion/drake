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
  interpolated_constraint{1} = FunctionHandleConstraint(lb, ub, 2*constraint.robot.getNumPositions(),  ...
    @(q1,q2)interpEvalWrapper(constraint,interpolation_parameter,q1,q2));
end

function [c,dc] = interpEvalWrapper(constraint,interpolation_parameter,q1,q2)
  nq = numel(q1);
  n_interp = numel(interpolation_parameter); 
  c = 0;
  dc = zeros(1,2*nq);
  for i = 1:n_interp
    q_interp = (1-interpolation_parameter(i))*q1+interpolation_parameter(i)*q2;
    kinsol = constraint.robot.doKinematics(q_interp);
    [c_i,dc_dq_interp] = constraint.eval(0,kinsol);
    dc_i = dc_dq_interp*[(1-interpolation_parameter(i))*eye(nq),interpolation_parameter(i)*eye(nq)];
    c = c+(1/n_interp)*c_i;
    dc = dc+(1/n_interp)*dc_i;
    %c = c+c_i;
    %dc = dc+:c_i;
  end
end
