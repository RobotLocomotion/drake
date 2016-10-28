close all
clear all

p = CartPolePlant();
p.friction_on = 0;
p = setInputLimits(p,-10,10);
v = CartPoleVisualizer(p);

N = 101;

[utraj1,xtraj1,z1,prog1] = p.swingUpDirtran(N);
v.playback(xtraj1);

D = 2^2;
E0 = zeros(4);
Q = diag([10 10 1 1]);
R = .1;
Qf = 100*eye(4);
[utraj2,xtraj2,z2,prog2] = p.robustSwingUp(N,D,E0,Q,R,Qf);
v.playback(xtraj2);

%Plots
h1 = z1(prog1.h_inds);
t1 = [0; cumsum(h1)];
x1 = xtraj1.eval(t1);
u1 = utraj1.eval(t1);

h2 = z2(prog2.h_inds);
t2 = [0; cumsum(h2)];
x2 = xtraj2.eval(t2);
u2 = utraj2.eval(t2);

figure(2);
subplot(3,1,1);
plot(t2,x2(1,:),'LineWidth',1.5);
hold on;
plot(t1,x1(1,:),'LineWidth',1.5);
ylabel('$x$','Interpreter','latex');
xlim([0 5]);
box off
subplot(3,1,2);
plot(t2,x2(2,:),'LineWidth',1.5);
hold on;
plot(t1,x1(2,:),'LineWidth',1.5);
xlim([0 5]);
ylabel('$\theta$','Interpreter','latex');
legend('DIRTREL', 'DIRTRAN');
box off
subplot(3,1,3);
plot(t2,u2,'LineWidth',1.5);
hold on;
plot(t1,u1,'LineWidth',1.5);
xlim([0 5]);
ylim([-10 10]);
ylabel('$u$','Interpreter','latex');
xlabel('Time (s)');
box off

% addpath('/Users/Zac/Documents/MATLAB/TikZ/');
% cleanfigure();
% matlab2tikz('CartPoleTraj.tikz', 'height', '\figureheight', 'width', '\figurewidth');

%Closed-loop simulation
c1 = tvlqr(p,xtraj1,utraj1,Q,R,Qf);
c2 = tvlqr(p,xtraj2,utraj2,Q,R,Qf);

p.friction_on = 1;
p.mu = 0.2;

clsys1 = feedback(p,c1);
xcl1 = clsys1.simulate([utraj1.tspan(1) utraj1.tspan(2)], [0 0 0 0]');
v.playback(xcl1);

clsys2 = feedback(p,c2);
xcl2 = clsys2.simulate([utraj2.tspan(1) utraj2.tspan(2)], [0 0 0 0]');
v.playback(xcl2);

% %Write movie files
% %v.playbackAVI(xcl1, 'swing1.avi');
% %v.playbackAVI(xcl2, 'swing2.avi');
% % setenv('PATH', [getenv('PATH') ':/usr/local/bin']);
% % setenv('PATH', [getenv('PATH') ':/Library/TeX/texbin']);
% % v.playbackSWF(xcl1, 'swing1.swf');
% % v.playbackSWF(xcl2, 'swing2.swf');
%



