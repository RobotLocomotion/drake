function simulinkDependency
% tests that I can load the RBM without loading simulink

if ~checkDependency('simulink')
  disp('no worries.  you don''t even have simulink installed');
  return;
end

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

xtraj = simulate(r,[0 1]);

a = strfind(inmem,'Simulink');
if isempty([a{:}])
  error('simulink should have been loaded by now.  i must not be testing well enough');
end



