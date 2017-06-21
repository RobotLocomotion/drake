function runQP

simple = false;

options.use_bullet = false;
options.floating = true;
if ~simple
    urdf = fullfile(getDrakePath(), 'examples', 'Cassie', 'urdf', 'cassie_no_loops.urdf');
    load(fullfile(getDrakePath(), 'examples', 'Cassie', 'data', 'standing_state.mat'));    
else
    urdf = fullfile(getDrakePath(), 'examples', 'Cassie', 'urdf', 'cassie_simple.urdf');
    load(fullfile(getDrakePath(), 'examples', 'Cassie', 'data', 'standing_state_simple.mat'));    
end

r = Cassie(urdf,options);
v = r.constructVisualizer;

x0=xstar;
q0=x0(1:r.getNumPositions);

kinsol = doKinematics(r,q0);
com = r.getCOM(kinsol);

c = DiscreteQP(r,[com;0*com],x0);

x0(3) = x0(3)+0.0;
sys = feedback(r,c);
% Forward simulate dynamics with visulazation, then playback at realtime
S=warning('off','Drake:DrakeSystem:UnsupportedSampleTime');
output_select(1).system=1;
output_select(1).output=1;
sys = mimoCascade(sys,v,[],[],output_select);
warning(S);
xtraj = simulate(sys,[0 2],x0);
playback(v,xtraj,struct('slider',true));

end