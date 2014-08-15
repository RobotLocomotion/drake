function runPassive

options.floating = true;
options.view = 'right';
r = TimeSteppingRigidBodyManipulator('Hubo_description/urdf/jaemiHubo_minimalcontact.urdf',.001,options);
v = r.constructVisualizer();
v.display_dt=0.05;

xtraj= simulate(r,[0 1]);
v.playback_speed = 0.2;
v.playback(xtraj);

end