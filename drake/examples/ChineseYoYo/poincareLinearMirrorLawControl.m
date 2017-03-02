%build up hybrid plant

p = SoftPaddleHybrid();
porig = p;
plantSim = SimulinkModel(p.getModel());
% take the frames from the simulink model and use those for the simulation of plant and controller
p = setOutputFrame(p, getOutputFrame(plantSim));
p = setInputFrame(p, getInputFrame(plantSim));

c = SoftPaddlePositionController(p);

%Adjust the set angle here:
c.psiDes = 0;

output_select(1).system = 1;
output_select(1).output = plantSim.getOutputFrame();
output_select(2).system = 2;
output_select(2).output = c.getOutputFrame();

sys = mimoFeedback(plantSim,c,[],[],[],output_select);
dSpatial = 0.0005;

v = p.constructVisualizer();
x0 = Point(getStateFrame(p));
x0.m = 1;
x0.load_x = -0.031;
x0.load_z = 4.5;
x0 = double(x0);
x0(2:end) = resolveConstraints(p.no_contact,x0(2:end));
v.drawWrapper(0,x0);

%% Perturb along the x direction
x0 = Point(getStateFrame(p));
x0.m = 1;
x0.load_x = -0.031-dSpatial/2;
x0.load_z = 4.5;
% x0.load_zdot = -4;
x0 = double(x0);
x0(2:end) = resolveConstraints(p.no_contact,x0(2:end));

tic
[ytraj,xtraj] = simulate(sys,[0 1.75],x0);
toc
tt=getBreaks(ytraj);
yAll = ytraj.eval(tt);

%Find changes
jumpIdx = find(diff(yAll(1,:)));
xn = yAll(2:9,jumpIdx(1));
xnplus1 = yAll(2:9,jumpIdx(3));


x0 = Point(getStateFrame(p));
x0.m = 1;
x0.load_x = -0.031+dSpatial/2;
x0.load_z = 4.5;
% x0.load_zdot = -4;
x0 = double(x0);
x0(2:end) = resolveConstraints(p.no_contact,x0(2:end));


tic
[ytraj,xtraj] = simulate(sys,[0 1.75],x0);
toc
tt=getBreaks(ytraj);
yAll = ytraj.eval(tt);

%Find changes
jumpIdx = find(diff(yAll(1,:)));
xn = yAll(2:9,jumpIdx(1));
xnplus2 = yAll(2:9,jumpIdx(3));

deltax = (xnplus2 -xnplus1)/dSpatial;


%% Perturb along the xdot direction
x0 = Point(getStateFrame(p));
x0.m = 1;
x0.load_x = -0.031;
x0.load_x = -0.045;
x0.load_z = 4.5;
x0.load_xdot = 0-dSpatial/2;
x0.load_zdot = 0;
x0 = double(x0);
x0(2:end) = resolveConstraints(p.no_contact,x0(2:end));

tic
[ytraj,xtraj] = simulate(sys,[0 1.75],x0);
toc
tt=getBreaks(ytraj);
yAll = ytraj.eval(tt);

%Find changes
jumpIdx = find(diff(yAll(1,:)));
xn = yAll(2:9,jumpIdx(1));
xnplus1 = yAll(2:9,jumpIdx(3));


x0 = Point(getStateFrame(p));
x0.m = 1;
x0.load_x = -0.031;
x0.load_x = -0.045;
x0.load_z = 4.5;
x0.load_xdot = 0+dSpatial/2;
x0.load_zdot = 0;
x0 = double(x0);
x0(2:end) = resolveConstraints(p.no_contact,x0(2:end));

tic
[ytraj,xtraj] = simulate(sys,[0 1.75],x0);
toc
tt=getBreaks(ytraj);
yAll = ytraj.eval(tt);

%Find changes
jumpIdx = find(diff(yAll(1,:)));
xn = yAll(2:9,jumpIdx(1));
xnplus2 = yAll(2:9,jumpIdx(3));

deltaxdot = (xnplus2 -xnplus1)/dSpatial;



%% Perturb along the z direction
x0 = Point(getStateFrame(p));
x0.m = 1;
x0.load_x = -0.031;
x0.load_x = -0.045;
x0.load_z = 4.5-dSpatial/2;
x0.load_xdot = 0;
x0.load_zdot = 0;
x0 = double(x0);
x0(2:end) = resolveConstraints(p.no_contact,x0(2:end));

tic
[ytraj,xtraj] = simulate(sys,[0 1.75],x0);
toc
tt=getBreaks(ytraj);
yAll = ytraj.eval(tt);

%Find changes
jumpIdx = find(diff(yAll(1,:)));
xn = yAll(2:9,jumpIdx(1));
xnplus1 = yAll(2:9,jumpIdx(3));


x0 = Point(getStateFrame(p));
x0.m = 1;
x0.load_x = -0.031;
x0.load_x = -0.045;
x0.load_z = 4.5+dSpatial/2;
x0.load_xdot = 0;
x0.load_zdot = 0;
x0 = double(x0);
x0(2:end) = resolveConstraints(p.no_contact,x0(2:end));

tic
[ytraj,xtraj] = simulate(sys,[0 1.75],x0);
toc
tt=getBreaks(ytraj);
yAll = ytraj.eval(tt);

%Find changes
jumpIdx = find(diff(yAll(1,:)));
xn = yAll(2:9,jumpIdx(1));
xnplus2 = yAll(2:9,jumpIdx(3));

deltaz = (xnplus2 -xnplus1)/dSpatial;



%% Perturb along the zdot direction
x0 = Point(getStateFrame(p));
x0.m = 1;
x0.load_x = -0.031;
x0.load_x = -0.045;
x0.load_z = 4.5;
x0.load_xdot = 0;
x0.load_zdot = 0-dSpatial/2;
x0 = double(x0);
x0(2:end) = resolveConstraints(p.no_contact,x0(2:end));

tic
[ytraj,xtraj] = simulate(sys,[0 1.75],x0);
toc
tt=getBreaks(ytraj);
yAll = ytraj.eval(tt);

%Find changes
jumpIdx = find(diff(yAll(1,:)));
xn = yAll(2:9,jumpIdx(1));
xnplus1 = yAll(2:9,jumpIdx(3));


x0 = Point(getStateFrame(p));
x0.m = 1;
x0.load_x = -0.031;
x0.load_x = -0.045;
x0.load_z = 4.5;
x0.load_xdot = 0;
x0.load_zdot = 0+dSpatial/2;
x0 = double(x0);
x0(2:end) = resolveConstraints(p.no_contact,x0(2:end));

tic
[ytraj,xtraj] = simulate(sys,[0 1.75],x0);
toc
tt=getBreaks(ytraj);
yAll = ytraj.eval(tt);

%Find changes
jumpIdx = find(diff(yAll(1,:)));
xn = yAll(2:9,jumpIdx(1));
xnplus2 = yAll(2:9,jumpIdx(3));

deltazdot = (xnplus2 -xnplus1)/dSpatial;

%% Perturb along the psi direction
x0 = Point(getStateFrame(p));
x0.m = 1;
x0.paddle_angle = 0-dSpatial/2;
x0.load_x = -0.031;
x0.load_x = -0.045;
x0.load_z = 4.5;
x0.load_xdot = 0;
x0.load_zdot = 0;
x0 = double(x0);
x0(2:end) = resolveConstraints(p.no_contact,x0(2:end));

%Adjust the set angle here:
c.psiDes = 0-dSpatial/2;

output_select(1).system = 1;
output_select(1).output = plantSim.getOutputFrame();
output_select(2).system = 2;
output_select(2).output = c.getOutputFrame();

sys = mimoFeedback(plantSim,c,[],[],[],output_select);


tic
[ytraj,xtraj] = simulate(sys,[0 1.75],x0);
toc
tt=getBreaks(ytraj);
yAll = ytraj.eval(tt);

%Find changes
jumpIdx = find(diff(yAll(1,:)));
xn = yAll(2:9,jumpIdx(1));
xnplus1 = yAll(2:9,jumpIdx(3));


x0 = Point(getStateFrame(p));
x0.m = 1;
x0.paddle_angle = 0+dSpatial/2;
x0.load_x = -0.031;
x0.load_x = -0.045;
x0.load_z = 4.5;
x0.load_xdot = 0;
x0.load_zdot = 0;
x0 = double(x0);
x0(2:end) = resolveConstraints(p.no_contact,x0(2:end));


c.psiDes = 0+dSpatial/2;

output_select(1).system = 1;
output_select(1).output = plantSim.getOutputFrame();
output_select(2).system = 2;
output_select(2).output = c.getOutputFrame();

sys = mimoFeedback(plantSim,c,[],[],[],output_select);

tic
[ytraj,xtraj] = simulate(sys,[0 1.75],x0);
toc
tt=getBreaks(ytraj);
yAll = ytraj.eval(tt);

%Find changes
jumpIdx = find(diff(yAll(1,:)));
xn = yAll(2:9,jumpIdx(1));
xnplus2 = yAll(2:9,jumpIdx(3));

deltapsi = (xnplus2 -xnplus1)/dSpatial;


%% Perturb along the psidot direction
%% TODO: NEED TO Add velocity controlled plant to allow psidot to stay constnat
x0 = Point(getStateFrame(p));
x0.m = 1;
x0.paddle_angledot = 0-dSpatial/2;
x0.load_x = -0.031;
x0.load_x = -0.045;
x0.load_z = 4.5;
x0.load_xdot = 0;
x0.load_zdot = 0;
x0 = double(x0);
x0(2:end) = resolveConstraints(p.no_contact,x0(2:end));

tic
[ytraj,xtraj] = simulate(sys,[0 1.75],x0);
toc
tt=getBreaks(ytraj);
yAll = ytraj.eval(tt);

%Find changes
jumpIdx = find(diff(yAll(1,:)));
xn = yAll(2:9,jumpIdx(1));
xnplus1 = yAll(2:9,jumpIdx(3));


x0 = Point(getStateFrame(p));
x0.m = 1;
x0.paddle_angledot = 0+dSpatial/2;
x0.load_x = -0.031;
x0.load_x = -0.045;
x0.load_z = 4.5;
x0.load_xdot = 0;
x0.load_zdot = 0;
x0 = double(x0);
x0(2:end) = resolveConstraints(p.no_contact,x0(2:end));

tic
[ytraj,xtraj] = simulate(sys,[0 1.75],x0);
toc
tt=getBreaks(ytraj);
yAll = ytraj.eval(tt);

%Find changes
jumpIdx = find(diff(yAll(1,:)));
xn = yAll(2:9,jumpIdx(1));
xnplus2 = yAll(2:9,jumpIdx(3));

deltapsidot = (xnplus2 -xnplus1)/dSpatial;


A = [deltax([3:4,7:8]),deltaz([3:4,7:8]),deltaxdot([3:4,7:8]),deltazdot([3:4,7:8])];
B = deltapsi([3:4,7:8]);
B2 = [deltapsi([3:4,7:8]), deltapsidot([3:4,7:8])]; %TODO: fix how it is generated

%% Calculate DLQR
Q = diag([1,1,1,1]);
R = 1e-3;
R2 = 1e-3*diag([1,1]);
[K,S,E] = dlqr(A,B,Q,R);
[K2,S2,E2] = dlqr(A,B2,Q,R2);%TODO: fix how it is generated
z = eig(A-B*K);
z2 = eig(A-B2*K2);%TODO: fix how it is generated

save('poincareLinearMirrorLawOutput.mat','A','B','Q','R','K','S','E');

figure(4), clf, hold on
h = ezplot('x^2 + y^2 = 1');
set(h, 'Color', [0 0 0], 'LineWidth', 2)
plot(z, 'x', 'MarkerSize', 10)
axis([-1,1,-1,1])
axis('square')

Ts = 1;
sys = ss(A-B*K, B, eye(4), zeros(4,1),Ts);
t = 0:Ts:10;
u = zeros(1,length(t));
x0 = [0.1; 0.5; -0.3; 1];

figure(5), clf
lsim(sys, u, t, x0);


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

% v.playback(ytraj,struct('slider',true));
%       v.playbackAVI(ytraj,'soft_juggler_passive')
