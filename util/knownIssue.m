function knownIssue(testname,ex)

err_id = ex.identifier;
if strcmp(ex.identifier,'Simulink:SFunctions:SFcnErrorStatus')
  % then try to extract the original identifier in the message
  ids = regexp(ex.message,'error (.+) in MATLAB callback','tokens');
  if ~isempty(ids), err_id = ids{1}; end
end

if isempty(err_id)
  switch err_id
    case 'PathLCP:FailedToSolve'
      if any(strcmp(testname,{'systems/plants/test/fallingBrickLCP', ...
          'systems/plants/test/fallingCapsulesTest',...
          'systems/plants/test/multiRobotTest', ...
          'systems/plants/test/momentumTest', ...
          'systems/plants/test/terrainTest', ...
          'systems/plants/test/testFloatingBaseDynamics', ...
          'examples/ZMP/CartTable.run'}))
        github(18); return;
      end
    case 'Airplane2D:SNOPT:INFO3'
      if any(strcmp(testname,{'examples/Airplane2D/runDircolWithObs', ...
          'examples/Airplane2D/runFunnelWithObstacles'}))
        github(125); return;
      end
    case 'Drake:ValueCheck'
      if any(strcmp(testname,{'solvers/test/testNLPWConstraint'}))
        github(135); return;
      elseif any(strcmp(testname,{'solvers/test/testKinCnst'}))
        github(136); return;
      end
    case {'Simulink:SFunctions:SFcnErrorStatus','Drake:Manipulator:ResolveConstraintsFailed'}
      if any(strcmp(testname,{'systems/plants/test/coordinateSystemTest', ...
          'systems/plants/test/fallingBrickLCP', ...
          'systems/plants/test/fallingCapsulesTest', ...
          'systems/plants/test/momentumTest',...
          'systems/plants/test/multiRobotTest',...
          'systems/plants/test/testFloatingBaseDynamics',...
          'examples/Atlas/runAtlasDynamics',...
          'examples/Atlas/runAtlasFrontalDynamics',...
          'examples/Atlas/runAtlasSagittalDynamics',...
          'examples/Atlas/test/runDoubleAtlas'}))
        github(146); return;
      end
  end
end

fprintf('\nThis issue is not listed as known.  Please seriously consider reporting it on <a href="http://groups.csail.mit.edu/locomotion/bugs/">bugzilla</a>.\n\n');
      
end
      
function bugzilla(bug_id)
  fprintf('\nThis is a known issue.  See <a href="http://groups.csail.mit.edu/locomotion/bugs/show_bug.cgi?id=%d">Bugzilla bug %d</a> for more information.\n\n',bug_id,bug_id);
end

function github(issue_id)
  fprintf('\nThis is a known issue.  See <a href="https://github.com/RobotLocomotion/drake/issues/%d">Drake issue %d</a> for more information.\n\n',issue_id,issue_id);
  fprintf('\n<DartMeasurement name="GitHubIssue" type="link/url">https://github.com/RobotLocomotion/drake/issues/%d</DartMeasurement>\n\n',issue_id);
end
