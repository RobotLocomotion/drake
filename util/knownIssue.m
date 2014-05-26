function knownIssue(testname,ex)

err_id = ex.identifier;
if strcmp(ex.identifier,'Simulink:SFunctions:SFcnErrorStatus')
  % then try to extract the original identifier in the message
  ids = regexp(ex.message,'error (.+) in MATLAB callback','tokens');
  if ~isempty(ids), err_id = ids{1}; end
end

switch testname
  case {'systems/plants/test/fallingBrickLCP', ...
      'systems/plants/test/fallingCapsulesTest',...
      'systems/plants/test/multiRobotTest', ...
      'systems/plants/test/momentumTest', ...
      'systems/plants/test/terrainTest', ...
      'systems/plants/test/testFloatingBaseDynamics'}
    if strcmp(err_id,'PathLCP:FailedToSolve')
      github(18); return;
    end
  case {'examples/Airplane2D/runDircolWithObs', ...
      'examples/Airplane2D/runFunnelWithObstacles'}
    if strcmp(err_id,'Airplane2D:SNOPT:INFO3')
      github(125); return;
    end
end

fprintf('\nThis issue is not listed as known.  Please seriously consider reporting it on <a href="http://groups.csail.mit.edu/locomotion/bugs/">bugzilla</a>.\n\n');
      
end
      
function bugzilla(bug_id)
  fprintf('\nThis is a known issue.  See <a href="http://groups.csail.mit.edu/locomotion/bugs/show_bug.cgi?id=%d">Bugzilla bug %d</a> for more information.\n\n',bug_id,bug_id);
end

function github(issue_id)
  fprintf('\nThis is a known issue.  See <a href="https://github.com/RobotLocomotion/drake/issues/%d">Drake issue %d</a> for more information.\n\n',issue_id,issue_id);
end
