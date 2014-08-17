function knownIssue(testname,ex)

err_id = ex.identifier;
if strcmp(ex.identifier,'Simulink:SFunctions:SFcnErrorStatus')
  % then try to extract the original identifier in the message
  ids = regexp(ex.message,'error (.+) in MATLAB callback','tokens');
  if ~isempty(ids), err_id = ids{1}; end
end

if ~isempty(err_id)
  switch err_id
    case 'PathLCP:FailedToSolve'
      if any(strcmp(testname,{'systems/plants/test/fallingBrickLCP', ...
          'systems/plants/test/fallingCapsulesTest',...
          'systems/plants/test/multiRobotTest', ...
          'systems/plants/test/momentumTest', ...
          'systems/plants/test/terrainTest', ...
          'systems/plants/test/testFloatingBaseDynamics', ...
          'examples/Quadrotor/Quadrotor.runOpenLoop', ...
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
      elseif any(strcmp(testname,'examples/RimlessWheel/test/testContactGradients'))
        github(303); return;
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
    case 'MATLAB:nonExistentCellElement'
      if any(strcmp(testname,{'systems/plants/constraint/test/RelativeQuatConstraintTest',...
          'systems/plants/constraint/test/RelativeGazeTargetConstraintTest'}))
        github(286); return;
      end
  end
else
  switch testname
    case 'solvers/test/testNLPWConstraint'
      if strcmp(ex.message,'fmincon failed') && ~verLessThan('matlab','8.3.0')
        github(255); return;
      end
    case 'systems/plants/test/testRigidBodyInertialMeasurementUnit'
      github(274); return;
  end
end

fprintf('\nThis issue is not listed as known.  Please seriously consider reporting it on <a href="http://github.com/RobotLocomotion/drake/issues">GitHub</a>.\n');
fprintf('testname: %s\n',testname);
fprintf('error ID: %s\n\n',err_id);
fprintf('error msg: %s\n\n',ex.message);

end

function bugzilla(bug_id)
  fprintf('\nThis is a known issue.  See <a href="http://groups.csail.mit.edu/locomotion/bugs/show_bug.cgi?id=%d">Bugzilla bug %d</a> for more information.\n\n',bug_id,bug_id);
end

function github(issue_id)
  fprintf('\nThis is a known issue.  See <a href="https://github.com/RobotLocomotion/drake/issues/%d">Drake issue %d</a> for more information.\n\n',issue_id,issue_id);
  fprintf('\n<DartMeasurement name="GitHubIssue" type="link/url">https://github.com/RobotLocomotion/drake/issues/%d</DartMeasurement>\n\n',issue_id);
end
