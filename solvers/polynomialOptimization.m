function [xstar,objval,exitflag,output] = polynomialOptimization(decision_variables,objective,equality_constraints,inequality_constraints,options)

% Wrapper method to provide a common interface for polynomial optimization.
%  
% Using x to denote the decision variables, attempts to solve the problem
%
%   minimize_x objective(x)
%   subject to 
%         equality_constraints(x) = 0 
%         inequality_constraint(x) >= 0
%
% @param decision_variables a simple msspoly description of x
% @param objective an msspoly