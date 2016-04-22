function runPassive

options.floating = false;
options.terrain = RigidBodyFlatTerrain();
r = TimeSteppingRigidBodyManipulator('urdf/lbr_iiwa.urdf',0.001,options);

v = r.constructVisualizer;
v.display_dt = .05;

s = warning('off','Drake:DrakeSystem:UnsupportedSampleTime');  % we are knowingly breaking out to a simulink model with the cascade on the following line.
sys = cascade(r,v);
warning(s);
simulate(sys,[0 2]);

v.playback(xtraj,struct('slider',true));
