function lcmgl_test
% matlab version which should achieve the same as the .cpp version

  lcmgl = bot_lcmgl_init('lcmgl-matlab-example');

  bot_lcmgl_color3f(lcmgl, 0, 0, 1);  % blue

  xyz = zeros(1,3);

  bot_lcmgl_sphere(lcmgl, xyz, .1, 36, 36);

  bot_lcmgl_switch_buffer(lcmgl);

  bot_lcmgl_destroy(lcmgl);

end