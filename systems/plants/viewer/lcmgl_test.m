function lcmgl_test
% matlab version which should achieve the same as the .cpp version
  checkDependency('lcmgl');
  lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(),'lcmgl-matlab-example');

  lcmgl.glColor3f(0, 0, 1);  % blue

  xyz = zeros(1,3);

  lcmgl.sphere(xyz, .1, 36, 36);
  lcmgl.switchBuffers();
end