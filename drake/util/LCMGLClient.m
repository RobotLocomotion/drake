function lcmgl = LCMGLClient(name)
  if nargin < 1, name = 'Drake'; end
  checkDependency('lcmgl');
  lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),name);
end
