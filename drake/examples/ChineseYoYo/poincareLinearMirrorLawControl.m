%build up hybrid plant
p = SoftPaddleHybrid();
% utraj = utraj.setOutputFrame(getInputFrame(p));
% xtraj = xtraj.setOutputFrame(getOutputFrame(p));

% for j =1:2
%   
% jlmin = p.modes{j}.sys1.joint_limit_min;
% 
% jlmax = p.modes{j}.sys1.joint_limit_max;
% jlmin(1) = 0;
% jlmax(1) = 0;
% p.modes{j}.sys1 = p.modes{j}.sys1.setJointLimits(jlmin,jlmax);
% 
% end
% 

jlmin = p.in_contact.joint_limit_min;
jlmax = p.in_contact.joint_limit_max;

jlmin(1) = 0;
jlmax(1) = 0;
p.in_contact = p.in_contact.setJointLimits(jlmin,jlmax);
p.in_contact = compile( p.in_contact);

jlmin = p.no_contact.joint_limit_min;
jlmax = p.no_contact.joint_limit_max;

jlmin(1) = -0.0001;
jlmax(1) = +0.0001;
p.no_contact = p.no_contact.setJointLimits(jlmin,jlmax);

p.no_contact = compile( p.no_contact);

%% TODO: Try state constraints instead??? 

%define initial state in frame of paddle:
%inversekinematics call

% forward simulate
x0 = getInitialState(p);
v = p.constructVisualizer();

v.drawWrapper(0,x0);
[ytraj,xtraj] = simulate(p,[0 1],x0); %simulate only until mode changed again

v.playback(ytraj,struct('slider',true));
%       v.playbackAVI(ytraj,'soft_juggler_passive')
