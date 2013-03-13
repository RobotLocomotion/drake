% NOTEST
function runPD()

options.floating = true;
options.dt = 0.001;
r = Atlas('urdf/atlas_minimal_contact.urdf',options);

% set initial state to fixed point
load('data/atlas_fp.mat');
%load('data/atlas_pinned_config.mat');
r = r.setInitialState(xstar);

v = r.constructVisualizer;
v.display_dt = 0.05;

[kp,kd] = getPDGains(r); 
sys = pdcontrol(r,kp,kd);


B = r.getB();
theta_des = B' * xstar(1:r.getNumStates()/2);
c = ConstOrPassthroughSystem(theta_des); % command a constant desired theta
c = c.setOutputFrame(sys.getInputFrame);
sys = cascade(c,sys); 

T = 2.0; % sec
if (0)
  tic;
  traj = simulate(sys,[0 T]); 
  toc
  playback(v,traj,struct('slider',true));
else
  % we are knowingly breaking out to a simulink model with the cascade on the following line.
  warning('off','Drake:DrakeSystem:UnsupportedSampleTime'); 
  sys = cascade(sys,v);
  simulate(sys,[0 T]);
end

end




