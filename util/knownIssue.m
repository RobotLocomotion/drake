function knownIssue(testname,ex)

err_id = ex.identifier;
if strcmp(ex.identifier,'Simulink:SFunctions:SFcnErrorStatus')
  % then try to extract the original identifier in the message
  ids = regexp(ex.message,'error (.+) in MATLAB callback','tokens');
  if ~isempty(ids), err_id = ids{1}; end
end

switch testname
  case {'systems/plants/test/fallingBrickLCP', ...
      'systems/plants/test/multiRobotTest', ...
      'systems/plants/test/momentumTest', ...
      'systems/plants/test/terrainTest', ...
      'systems/plants/test/testFloatingBaseDynamics'}
    if strcmp(err_id,'PathLCP:FailedToSolve')
      bugzilla(1095); return;
    end
  case {'systems/plants/constraint/test/testKinCnst', ...
      'systems/plants/constraint/test/testKinCnstWaff', ...
      'systems/plants/test/testIK',...
      'systems/plants/test/testIKtraj',...
      'examples/Atlas/test/testIK'}
    bugzilla(1897); return;
end

fprintf('\nThis issue is not listed as known.  Please seriously consider reporting it on <a href="http://groups.csail.mit.edu/locomotion/bugs/">bugzilla</a>.\n\n');
      
end
      
function bugzilla(bug_id)
  fprintf('\nThis is a known issue.  See <a href="http://groups.csail.mit.edu/locomotion/bugs/show_bug.cgi?id=%d">Bugzilla bug %d</a> for more information.\n\n',bug_id,bug_id);
end

function github(issue_id)
  error('need to insert the right url in here');
  fprintf('\nThis is a known issue.  See <a href="http://github.com/RobotLocomotion/drake/issues?id=%d">Drake issue %d</a> for more information.\n\n',bug_id,bug_id);
end
