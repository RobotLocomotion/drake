function runAtlasWalkingSplit(varargin)
%NOTEST
% This was originally the unit test for the new PlanEval/InstQP split controller. That's now the default controller, so this is just a placeholder. 

warning('Drake:runAtlasWalkingSplit:Deprecated', 'runAtlasWalkingSplit is deprecated. The new split controller is now enabled in runAtlasWalking.m');
runAtlasWalking(varargin{:});
end