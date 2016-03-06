function [plan, solvertime] = footstepAlternatingMIQP(obj, varargin)
% NOTEST
% Wrapper provided for backwards-compatibility
[plan, solvertime] = footstepPlanner.alternatingMIQP(obj, varargin{:});
end
