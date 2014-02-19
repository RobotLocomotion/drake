function runZMPFeedback

addpath(fullfile(pwd,'..'));

options.floating = true;
options.dt = 0.001;
r = Atlas('../urdf/atlas_minimal_contact.urdf',options);

% set initial state to fixed point
load('../data/atlas_fp.mat');
r = r.setInitialState(xstar);

v = r.constructVisualizer();
v.display_dt = .05;
v.draw(0,xstar);

if (0)
  [Kp,Kd] = getPDGains(r);
  sys = pdcontrol(r,Kp,Kd);
else
  r = enableIdealizedPositionControl(r,true);
  r = compile(r);
  sys = r;
end

c = ZMPTrackingControl(r,xstar);
c = setOutputFrame(c,getInputFrame(sys));
c = setInputFrame(c,getStateFrame(sys));
S = warning('off','Drake:DrakeSystem:UnsupportedSampleTime');
sys = feedback(sys,c);
warning(S);

tspan = c.getTspan; % sec

S=warning('off','Drake:DrakeSystem:UnsupportedSampleTime');
output_select(1).system=1;
output_select(1).output=1;
sys = mimoCascade(sys,v,[],[],output_select);
warning(S);
traj = simulate(sys,tspan);
save openloop_stepping.mat v traj;
playback(v,traj,struct('slider',true));
