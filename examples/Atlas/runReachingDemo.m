function runReachingDemo

options.floating = true;
options.dt = 0.001;
r = Atlas('urdf/atlas_minimal_contact.urdf',options);

% set initial state to fixed point
load('data/atlas_fp.mat');
r = r.setInitialState(xstar);

v = r.constructVisualizer();
v.display_dt = .05;

[Kp,Kd] = getPDGains(r);
sys = pdcontrol(r,Kp,Kd);

c = StandingEndEffectorControl(sys,r);

% create end effectors
joint_names = r.getJointNames();
joint_names = joint_names(2:end); % get rid of null string at beginning..
right_ee = EndEffector(r,'atlas','r_hand',[0;0;0],'R_HAND_GOAL');
right_ee = right_ee.setMask(~cellfun(@isempty,strfind(joint_names,'r_arm')));
left_ee = EndEffector(r,'atlas','l_hand',[0;0;0],'L_HAND_GOAL');
left_ee = left_ee.setMask(~cellfun(@isempty,strfind(joint_names,'l_arm')));

% add end effectors
c = c.addEndEffector(right_ee);
c = c.addEndEffector(left_ee);

sys = sys.setInputFrame(c.getOutputFrame());
% set up MIMO connections
sys_to_c(1).from_output = 1;
sys_to_c(1).to_input = 6;
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
ins(5).system = 2;
ins(5).input = 5;
outs(1).system = 1;
outs(1).output = 1;
sys = mimoFeedback(sys,c,sys_to_c,c_to_sys,ins,outs);
clear ins;

% left end effector goal generator
lhand_gen = SimpleEEGoalGenerator(r,'atlas','L_HAND_GOAL');
lhand_gen = lhand_gen.setGoal([0.2; 0.25; 1.5]);
lhand_gen = lhand_gen.setOutputFrame(sys.getInputFrame.frame{1});

% set up MIMO connections
sys_to_lhg(1).from_output = 1;
sys_to_lhg(1).to_input = 1;
lhg_to_sys(1).from_output = 1;
lhg_to_sys(1).to_input = 1;
ins(1).system = 1;
ins(1).input = 2;
ins(2).system = 1;
ins(2).input = 3;
ins(3).system = 1;
ins(3).input = 4;
ins(4).system = 1;
ins(4).input = 5;
sys = mimoFeedback(sys,lhand_gen,sys_to_lhg,lhg_to_sys,ins,outs);
clear ins;

rhand_gen = SimpleEEGoalGenerator(r,'atlas','R_HAND_GOAL');
rhand_gen = rhand_gen.setGoal([0.2; -0.3; 1.0]);
rhand_gen = rhand_gen.setOutputFrame(sys.getInputFrame.frame{1});
sys_to_rhg(1).from_output = 1;
sys_to_rhg(1).to_input = 1;
rhg_to_sys(1).from_output = 1;
rhg_to_sys(1).to_input = 1;
ins(1).system = 1;
ins(1).input = 2;
ins(2).system = 1;
ins(2).input = 3;
ins(3).system = 1;
ins(3).input = 4;
sys = mimoFeedback(sys,rhand_gen,sys_to_rhg,rhg_to_sys,ins,outs);
clear ins outs;

% create support body generator
supp = ConstOrPassthroughSystem(1.0*~cellfun(@isempty,strfind(r.getLinkNames(),'foot')));
supp = supp.setOutputFrame(sys.getInputFrame.frame{1});

% set up MIMO connections
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


% COM goal generator
comg = SimpleCOMGoalGenerator(r);
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
qgen = ConstOrPassthroughSystem(x0(7:r.getNumStates()/2));
qgen = qgen.setOutputFrame(sys.getInputFrame());
sys = cascade(qgen,sys);

T = 10.0; % sec
if (0)
  traj = simulate(sys,[0 T]); 
  playback(v,traj,struct('slider',true));
else
  warning('off','Drake:DrakeSystem:UnsupportedSampleTime'); 
  sys = cascade(sys,v);
  simulate(sys,[0 T]);
end

end