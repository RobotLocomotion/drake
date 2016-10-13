%build up hybrid plant

p = SoftPaddleHybrid();
porig = p;
plantSim = SimulinkModel(p.getModel());
% take the frames from the simulink model and use those for the simulation of plant and controller
p = setOutputFrame(p, getOutputFrame(plantSim));
p = setInputFrame(p, getInputFrame(plantSim));

c = SoftPaddlePositionController(p);

output_select(1).system = 1;
output_select(1).output = plantSim.getOutputFrame();
output_select(2).system = 2;
output_select(2).output = c.getOutputFrame();

sys = mimoFeedback(plantSim,c,[],[],[],output_select);
x0 = Point(getStateFrame(p));
x0.m = 1;
x0.load_x = -0.02905;
x0.load_z = 3.9999;
x0.load_zdot = -4;
x0 = double(x0);
x0(2:end) = resolveConstraints(p.in_contact,x0(2:end));

v = p.constructVisualizer();

v.drawWrapper(0,x0);
tic
[ytraj,xtraj] = simulate(sys,[0 1.45],x0);
toc
tt=getBreaks(ytraj);
yAll = ytraj.eval(tt);

%Find changes
jumpIdx = find(diff(yAll(1,:)));

xn = yAll(2:9,1);

xnplus1 = yAll(2:9,jumpIdx(2));

%Now perturb and redot it for other set values...
%TODO


% [A,B,C,D] = linearize(sys,0,zeros(2,1),0);
% 
% if (any(any(abs([A,B,C,D] - [[1,2;3,4],[5;6],eye(2),zeros(2,1)])>1e-9)))
%   error('something''s not right');
% end

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
% 
% jlmin = p.in_contact.joint_limit_min;
% jlmax = p.in_contact.joint_limit_max;
% 
% jlmin(1) = 0;
% jlmax(1) = 0;
% p.in_contact = p.in_contact.setJointLimits(jlmin,jlmax);
% p.in_contact = compile( p.in_contact);
% 
% jlmin = p.no_contact.joint_limit_min;
% jlmax = p.no_contact.joint_limit_max;
% 
% jlmin(1) = -0.0001;
% jlmax(1) = +0.0001;
% p.no_contact = p.no_contact.setJointLimits(jlmin,jlmax);
% 
% p.no_contact = compile( p.no_contact);

%% TODO: Try state constraints instead??? 

%define initial state in frame of paddle:
%inversekinematics call

% forward simulate

v.playback(ytraj,struct('slider',true));
%       v.playbackAVI(ytraj,'soft_juggler_passive')
