%% load in
fprintf('Setup...\n');
dt = 0.001;
options.dt = dt;
options.floating = false;
options.base_offset = [0;0;0]; %[-0.5, 0, 1.5]'; %For now positions on ground
options.base_rpy = [-pi/2, 0, 0]';
options.ignore_self_collisions = true;
options.collision = false;
options.hands = 'none';
r = IRB140('urdf/irb_140.urdf', options);
options_hand = options;
options_hand.hands = 'robotiq';
r_hand = IRB140('urdf/irb_140.urdf', options_hand);

r_hand = r_hand.removeCollisionGroupsExcept({'palm', 'knuckle'});
r_hand = r_hand.compile();

v=r.constructVisualizer();

%% initial config
% x0_hand = r_hand.getInitialState();
x0 = r.getInitialState();
x0_hand = r_hand.getInitialState();


%% final pose
fprintf('Generating target traj\n');
options.visualize = true;
target_xtraj = runPlanning(x0(1:r.getNumPositions), [0.5, 0.0, 0.5]', options);


pd_control = irb140_trajfollow_block(r, target_xtraj);
clear ins; clear connection_1to2;
clear outs; clear connection_2to1;
ins(1).system = 1;
ins(1).input = 2;
outs(1).system = 1;
outs(1).output = 1;
outs(2).system = 1;
outs(2).output = 2;
connection_1to2(1).from_output = 1;
connection_1to2(1).to_input = 1;
connection_2to1(1).from_output = 1;
connection_2to1(1).to_input = 1;
sys = mimoFeedback(r_hand, pd_control, connection_1to2, connection_2to1, ins,outs);

%% simulate
v=r_hand.constructVisualizer();
v.display_dt = 0.001;
S=warning('off','Drake:DrakeSystem:UnsupportedSampleTime');
output_select(1).system=1;
output_select(1).output=1;
output_select(2).system=1;
output_select(2).output=2;
sys = mimoCascade(sys,v,[],[],output_select);
warning(S);

traj = simulate(sys,[0 2],x0_hand);

% This doesn't see hand movements. Why?
playback(v,traj,struct('slider',true));