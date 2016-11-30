function simulinkDependency
% tests that I can load the RBM without loading simulink

checkDependency('simulink');  

if verLessThan('matlab','8.3') % 2014a
  disp('I know it doesn''t work on 2012b, but haven''t tested 2013');
  return;
end

a = strfind(inmem,'Simulink');
if ~isempty([a{:}])
  disp('simulink is already loaded. test is invalid');
  return;
end

r = RigidBodyManipulator(fullfile(getDrakePath,'examples','Acrobot','Acrobot.urdf'));

a = strfind(inmem,'Simulink');
if ~isempty([a{:}])
  error('oops.  simulink got loaded');
end

checkDependency('simulink','disable');
xtraj = simulate(r,[0 1]);  % this should run w/ simulateODE

a = strfind(inmem,'Simulink');
if ~isempty([a{:}])
  error('oops.  simulink got loaded');
end

checkDependency('simulink','enable');
xtraj = simulate(r,[0 1]);  % this should run w/ simulink

a = strfind(inmem,'Simulink');
if isempty([a{:}])
  error('simulink should have been loaded by now.  i must not be testing well enough');
end



