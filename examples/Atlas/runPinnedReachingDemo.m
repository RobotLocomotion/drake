function runPinnedReachingDemo
%

% NOTEST

options.floating = false;
options.dt = 0.001;
r = Atlas('urdf/atlas_minimal_contact.urdf',options);

% set initial state to fixed point
load('data/atlas_pinned_config.mat');
r = r.setInitialState(xstar);

v = r.constructVisualizer();
v.display_dt = .05;

[Kp,Kd] = getPDGains(r);
sys = pdcontrol(r,Kp,Kd);

c = PinnedEndEffectorControl(sys,r);

% create end effectors
joint_names = r.getJointNames();
joint_names = joint_names(2:end); % get rid of null string at beginning..
right_ee = EndEffector(r,'atlas','r_hand',[0;0;0],'R_HAND_GOAL');
right_ee = right_ee.setMask(~cellfun(@isempty,strfind(joint_names,'r_arm')));
left_ee = EndEffector(r,'atlas','l_hand',[0;0;0],'L_HAND_GOAL');
left_ee = left_ee.setMask(~cellfun(@isempty,strfind(joint_names,'l_arm')));
head_ee = EndEffector(r,'atlas','head',[0.1;0;0],'GAZE_GOAL');
headmask = ~cellfun(@isempty,strfind(joint_names,'neck')) + ...
          ~cellfun(@isempty,strfind(joint_names,'back_bkz'));
head_ee = head_ee.setMask(headmask);
head_ee = head_ee.setGain(0.25);

% add end effectors
c = c.addEndEffector(right_ee);
c = c.addEndEffector(left_ee);
c = c.addEndEffector(head_ee);

sys = sys.setInputFrame(c.getOutputFrame());
% set up MIMO connections
sys_to_c(1).from_output = 1;
sys_to_c(1).to_input = 5;
c_to_sys(1).from_output = 1;
c_to_sys(1).to_input = 1;
ins(1).system = 2;
ins(1).input = 1;
ins(2).system = 2;
ins(2).input = 2;
ins(3).system = 2;
ins(3).input = 3;
ins(4).system = 2;
ins(4).input = 4;
outs(1).system = 1;
outs(1).output = 1;
sys = mimoFeedback(sys,c,sys_to_c,c_to_sys,ins,outs);
clear ins;

% effector goal generators
head_gen = SimpleEEGoalGenerator(r,'atlas','head','GAZE_GOAL');
head_gen = head_gen.setGoal([0.2; 0.25; 0.5]);
head_gen = head_gen.setOutputFrame(sys.getInputFrame.frame{1});
sys_to_hg(1).from_output = 1;
sys_to_hg(1).to_input = 1;
hg_to_sys(1).from_output = 1;
hg_to_sys(1).to_input = 1;
ins(1).system = 1;
ins(1).input = 2;
ins(2).system = 1;
ins(2).input = 3;
ins(3).system = 1;
ins(3).input = 4;
sys = mimoFeedback(sys,head_gen,sys_to_hg,hg_to_sys,ins,outs);
clear ins;

lhand_gen = SimpleEEGoalGenerator(r,'atlas','l_hand','L_HAND_GOAL');
lhand_gen = lhand_gen.setGoal([0.2; 0.35; 0.45]);
lhand_gen = lhand_gen.setOutputFrame(sys.getInputFrame.frame{1});
sys_to_lhg(1).from_output = 1;
sys_to_lhg(1).to_input = 1;
lhg_to_sys(1).from_output = 1;
lhg_to_sys(1).to_input = 1;
ins(1).system = 1;
ins(1).input = 2;
ins(2).system = 1;
ins(2).input = 3;
sys = mimoFeedback(sys,lhand_gen,sys_to_lhg,lhg_to_sys,ins,outs);
clear ins;

rhand_gen = SimpleEEGoalGenerator(r,'atlas','r_hand','R_HAND_GOAL');
rhand_gen = rhand_gen.setGoal([0.2; -0.3; 0.0]);
rhand_gen = rhand_gen.setOutputFrame(sys.getInputFrame.frame{1});
sys_to_rhg(1).from_output = 1;
sys_to_rhg(1).to_input = 1;
rhg_to_sys(1).from_output = 1;
rhg_to_sys(1).to_input = 1;
ins(1).system = 1;
ins(1).input = 2;
sys = mimoFeedback(sys,rhand_gen,sys_to_rhg,rhg_to_sys,ins,outs);
clear ins;

x0 = r.getInitialState();
qgen = ConstOrPassthroughSystem(x0(1:r.getNumDOF()));
qgen = qgen.setOutputFrame(sys.getInputFrame());
sys = cascade(qgen,sys);

T = 5.0; % sec
S=warning('off','Drake:DrakeSystem:UnsupportedSampleTime');
output_select(1).system=1;
output_select(1).output=1;
sys = mimoCascade(sys,v,[],[],output_select);
warning(S);
traj = simulate(sys,[0 T]);
playback(v,traj,struct('slider',true));

end
