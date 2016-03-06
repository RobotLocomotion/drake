% NOTEST
function testReachability

p = addpath(fullfile(pwd,'..'));

% Run airplane example
Vsall = runReachability;

% Check to see that all Vs are greater than 1
if ~all(Vsall > 1)
  path(p);
  error('Reachability funnel is violated. Something is wrong!')
else
  disp('Passed test: All trajectories stay inside funnel')
end

path(p);
