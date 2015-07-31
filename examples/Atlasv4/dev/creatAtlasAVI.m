
path_handle = addpathTemporary(fullfile(getDrakePath,'examples','Atlasv4'));

load('data/atlas_alt3mode_K10B2_passive_traj_exec.mat')

options.twoD = true;
options.view = 'right';
options.floating = true;
options.ignore_self_collisions = true;
options.enable_fastqp = false;
s = '../urdf/atlas_simple_spring_ankle_planar_contact.urdf';
options.terrain = RigidBodyFlatTerrain();
w = warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
r = Atlas(s,options);
r = r.setOutputFrame(AtlasXZState(r));
r = r.setStateFrame(AtlasXZState(r));
warning(w);

v = r.constructVisualizer;
v.display_dt = 0.01;
v.xlim=[-0.35,1.7];
v.ylim=[-0.05,1.85];

traj = traj.setOutputFrame(r.getStateFrame);

v.playbackAVI(traj,'data/atlas_passive_4steps.avi');