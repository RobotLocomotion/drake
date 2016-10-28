close all

p = PendulumPlant();
v = PendulumVisualizer();

N = 51;

[utraj1,xtraj1,z1,prog1] = p.swingUpDirtran(N);

options.integration_method = RobustDirtranTrajectoryOptimization.MIDPOINT;
D = .2^2; % w'*D*w <=1. This corresponds to +/-.2 uncertainty in mass (20%)
[utraj2,xtraj2,z2,prog2] = p.robustSwingUpTrajectory(N,D,options);

%Playback
v.playback(xtraj1);
%keyboard
v.playback(xtraj2);
%keyboard

%Closed-loop simulation
Q = [10 0; 0 1];
R = .1;
Qf = 100*eye(2);

m = .5:.1:1.5;

%p = p.setMass(1);
c1 = tvlqr(p,xtraj1,utraj1,Q,R,Qf);
for k = 1:length(m)
p = p.setMass(m(k));
clsys1 = feedback(p,c1);
xcl1 = clsys1.simulate([utraj1.tspan(1) utraj2.tspan(2)], [0 0]');
v.playback(xcl1);
keyboard
end

p = p.setMass(1);
c2 = tvlqr(p,xtraj2,utraj2,Q,R,Qf);
for k = 1:length(m)
p = p.setMass(m(k));
clsys2 = feedback(p,c2);
xcl2 = clsys2.simulate(utraj2.tspan, [0 0]');
v.playback(xcl2);
keyboard
end

% %Write movie files
% %v.playbackAVI(xcl1, 'swing1.avi');
% %v.playbackAVI(xcl2, 'swing2.avi');
% % setenv('PATH', [getenv('PATH') ':/usr/local/bin']);
% % setenv('PATH', [getenv('PATH') ':/Library/TeX/texbin']);
% % v.playbackSWF(xcl1, 'swing1.swf');
% % v.playbackSWF(xcl2, 'swing2.swf');
% 
%Plots
h1 = z1(prog1.h_inds);
t1 = [0; cumsum(h1)];
x1 = z1(prog1.x_inds);
u1 = z1(prog1.u_inds);

h2 = z2(prog2.h_inds);
t2 = [0; cumsum(h2)];
x2 = z2(prog2.x_inds);
u2 = z2(prog2.u_inds);

close all 
addpath('/Users/Zac/Documents/MATLAB/TikZ/');

figure(2);
subplot(2,1,1);
plot(t2,x2(1,:),'LineWidth',1.5);
hold on;
plot(t1,x1(1,:),'LineWidth',1.5);
xlim([0 2.5]);
ylim([-3.5 3.5]);
ylabel('$\theta$','Interpreter','latex');
legend('DIRTREL', 'DIRTRAN');
box off
subplot(2,1,2);
plot(t2(1:end-1),u2,'LineWidth',1.5);
hold on;
plot(t1(1:end),u1,'LineWidth',1.5);
xlim([0 2.5]);
ylim([-3.5 3.5]);
ylabel('$u$','Interpreter','latex');
xlabel('Time (s)');
box off
cleanfigure();
matlab2tikz('PendulumTraj.tikz', 'height', '\figureheight', 'width', '\figurewidth');


figure(3);
subplot(2,1,1);
plot(t1,x1(1,:));
hold on
plot(t1,x1(2,:));
ylabel('x_{dirtran}');
xlim([0 t2(end)]);
l = legend('$\theta$', '$\dot{\theta}$');
set(l,'Interpreter','latex')
subplot(2,1,2);
plot(t2,x2(1,:));
hold on
plot(t2,x2(2,:));
ylabel('x_{robust}');
xlim([0 t2(end)]);
