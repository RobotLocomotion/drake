function runStandingDemo

options.floating = true;
options.dt = 0.001;
r = Atlas('urdf/atlas_minimal_contact.urdf',options);

% set initial state to fixed point
load('data/atlas_fp.mat');
r = r.setInitialState(xstar);

v = r.constructVisualizer();
v.display_dt = .05;

if (1)
  [Kp,Kd] = getPDGains(r);
  sys = pdcontrol(r,Kp,Kd);
else
  r = enableIdealizedPositionControl(r,true);
  r = compile(r);
  sys = r;
end

c = StandingEndEffectorControl(sys,r);

sys = sys.setInputFrame(c.getOutputFrame());

% set up MIMO connections
sys_to_c(1).from_output = 1;
sys_to_c(1).to_input = 4;
c_to_sys(1).from_output = 1;
c_to_sys(1).to_input = 1;
ins(1).system = 2;
ins(1).input = 1;
ins(2).system = 2;
ins(2).input = 2;
ins(3).system = 2;
ins(3).input = 3;
outs(1).system = 1;
outs(1).output = 1;
sys = mimoFeedback(sys,c,sys_to_c,c_to_sys,ins,outs);
clear ins outs;

% create support body generator
supp = ConstOrPassthroughSystem(1.0*~cellfun(@isempty,strfind(r.getLinkNames(),'foot')));
supp = supp.setOutputFrame(sys.getInputFrame.frame{1});
connection(1).from_output = 1;
connection(1).to_input = 1;
ins(1).system = 2;
ins(1).input = 2;
ins(2).system = 2;
ins(2).input = 3;
outs(1).system = 2;
outs(1).output = 1;
sys = mimoCascade(supp,sys,connection,ins,outs);
clear ins outs;

% create COM goal generator
%comg = SimpleCOMGoalGenerator(r);
comg = SimpleAlternatingCOMGoalGenerator(r);
comg = comg.setOutputFrame(sys.getInputFrame.frame{1});
sys_to_comg(1).from_output = 1;
sys_to_comg(1).to_input = 1;
comg_to_sys(1).from_output = 1;
comg_to_sys(1).to_input = 1;
ins(1).system = 1;
ins(1).input = 2;
outs(1).system = 1;
outs(1).output = 1;
sys = mimoFeedback(sys,comg,sys_to_comg,comg_to_sys,ins,outs);
clear ins outs;

% nominal position goal
x0 = r.getInitialState();
qgen = ConstOrPassthroughSystem(x0(7:r.getNumDOF()));
qgen = qgen.setOutputFrame(sys.getInputFrame());
sys = cascade(qgen,sys);

T = 1; % sec
S=warning('off','Drake:DrakeSystem:UnsupportedSampleTime');
output_select(1).system=1;
output_select(1).output=1;
sys = mimoCascade(sys,v,[],[],output_select);
warning(S);
traj = simulate(sys,[0 T]);
playback(v,traj,struct('slider',true));


end
