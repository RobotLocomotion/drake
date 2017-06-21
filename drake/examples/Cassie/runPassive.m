function runPassive

options.use_bullet = false;
options.floating = true;
urdf = fullfile(getDrakePath(), 'examples', 'Cassie', 'urdf', 'cassie_no_loops.urdf');
r = Cassie(urdf,options);

v = r.constructVisualizer;

load(fullfile(getDrakePath(), 'examples', 'Cassie', 'data', 'standing_state.mat'));
x0=xstar;

% v.inspector

% Forward simulate dynamics with visulazation, then playback at realtime
S=warning('off','Drake:DrakeSystem:UnsupportedSampleTime');
output_select(1).system=1;
output_select(1).output=1;
sys = mimoCascade(r,v,[],[],output_select);
warning(S);
xtraj = simulate(sys,[0 2],x0);
playback(v,xtraj,struct('slider',true));

end